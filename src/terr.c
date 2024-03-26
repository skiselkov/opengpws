/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
*/
/*
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#include <ctype.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include <shapefil.h>
#include <cairo.h>

#include <XPLMGraphics.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dsf.h>
#include <acfutils/glew.h>
#include <acfutils/glutils.h>
#include <acfutils/helpers.h>
#include <acfutils/list.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/png.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/shader.h>
#include <acfutils/worker.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include <cglm/cglm.h>

#include "dbg_log.h"
#include "egpws.h"
#include "terr.h"
#include "xplane.h"

#define	LOAD_DEM_WORKER_INTVAL	1			/* seconds */
#define	TERR_TILE_WORKER_INTVAL	1			/* seconds */
#define	EARTH_CIRC		(2 * EARTH_MSL * M_PI)	/* meters */
#define	TILE_HEIGHT		(EARTH_CIRC / 360.0)	/* meters */
#define	MAX_LAT			80			/* degrees */
#define	MIN_RES			2500			/* meters */
#define	MAX_RES			108			/* meters */
#define	MIN_RNG			5000			/* meters */
#define	DFL_RNG			50000			/* meters */
#define	MAX_RNG			600000			/* meters */
#define	MAX_TILE_PIXELS		10000			/* pixels */
#define	DSF_DEMI_MIN_RES	2			/* pixels */
#define	ALT_QUANT_STEP		FEET2MET(20)		/* quantization step */

#define	ELEV_V_OFF		10000			/* meters */
#define	ELEV_READ(pixel)	((pixel) - ELEV_V_OFF)
#define	ELEV_WRITE(elev)	((elev) + ELEV_V_OFF)

/* Earth Orbit Texture constants */
#define	EOT_ELEV_MAX		FEET2MET(29029)		/* Everest */
#define	EOT_ELEV_MIN		FEET2MET(-1419)		/* Dead sea shore */

typedef struct {
	int		lat;		/* latitude in degrees */
	int		lon;		/* longitude in degrees */
	double		min_elev;	/* minimum elevation (meters) */
	double		max_elev;	/* maximum elevation (meters) */
	/* protected by tile cache lock */
	double		load_res;	/* resolution at load time */
	bool_t		dirty;		/* pending upload to GPU */
	GLsync		in_flight;	/* data in flight to GPU */
	bool_t		empty;		/* tile contains no data */
	bool_t		seen;		/* seen in presence check */
	bool_t		remove;		/* mark for removal */

	/* protected using dem_tile_cache_lock */
	unsigned	pix_width;	/* tile width in pixels */
	unsigned	pix_height;	/* tile height in pixels */
	int16_t		*pixels;	/* elevation data samples */

	const uint8_t	*water_mask;	/* water intensity in `pixels' above */
	int		water_mask_stride;
	cairo_surface_t	*water_mask_surf;

	int		cur_tex;
	GLuint		tex[2];
	GLuint		pbo;

	glutils_quads_t	quads;
	vect2_t 	quads_v[4];

	avl_node_t	cache_node;
} dem_tile_t;

typedef struct {
	char		*path;
	list_node_t	node;
} srch_path_t;

typedef struct {
	char		fname[16];
	char		*path;
	avl_node_t	tnlc_node;
} tnlc_ent_t;

static bool_t inited = B_FALSE;
static list_t srch_paths;

static char *dsf_paths[180][360];

static mutex_t glob_pos_lock;
static geo_pos3_t glob_pos = NULL_GEO_POS3;

static worker_t	load_dem_worker_wk;

static mutex_t dem_tile_cache_lock;
static avl_tree_t dem_tile_cache;

static GLuint DEM_prog = 0;

/*
 * The tile name lookup cache. Makes lookups for previously seen tiles faster.
 * Only accessed from tile loader thread during normal ops, so no need to lock.
 */
static avl_tree_t tnlc;

static const egpws_range_t dfl_ranges[] = {
	{ DFL_RNG, MAX_RES },
	{ NAN, NAN }
};
static const egpws_range_t *ranges = dfl_ranges;

static fpp_t terr_render_get_fpp_impl(const egpws_render_t *render,
    bool apply_rot);

static int
dem_tile_compar(const void *a, const void *b)
{
	const dem_tile_t *ta = a, *tb = b;

	if (ta->lat < tb->lat)
		return (-1);
	if (ta->lat > tb->lat)
		return (1);
	if (ta->lon < tb->lon)
		return (-1);
	if (ta->lon > tb->lon)
		return (1);
	return (0);
}

static int
tnlc_compar(const void *a, const void *b)
{
	const tnlc_ent_t *ta = a, *tb = b;
	int res = strcmp(ta->fname, tb->fname);

	if (res < 0)
		return (-1);
	if (res > 0)
		return (1);
	return (0);
}

static bool_t
dem_dsf_check(const dsf_t *dsf, const dsf_atom_t **demi_p,
    const dsf_atom_t **demd_p)
{
	return (dsf != NULL && (*demi_p = dsf_lookup(dsf, DSF_ATOM_DEMS, 0,
	    DSF_ATOM_DEMI, 0, 0)) != NULL && (*demd_p = dsf_lookup(dsf,
	    DSF_ATOM_DEMS, 0, DSF_ATOM_DEMD, 0, 0)) != NULL &&
	    (*demi_p)->demi_atom.width >= DSF_DEMI_MIN_RES &&
	    (*demi_p)->demi_atom.height >= DSF_DEMI_MIN_RES);
}

static dsf_t *
load_dem_dsf(int lat, int lon, const dsf_atom_t **demi_p,
    const dsf_atom_t **demd_p)
{
	char dname[16];
	char fname[16];
	tnlc_ent_t srch;
	tnlc_ent_t *te;
	avl_index_t where;

	snprintf(dname, sizeof (dname), "%+03.0f%+04.0f", 
	    floor(lat / 10.0) * 10.0, floor(lon / 10.0) * 10.0);
	snprintf(fname, sizeof (fname), "%+03d%+04d.dsf", lat, lon);

	strlcpy(srch.fname, fname, sizeof (srch.fname));
	te = avl_find(&tnlc, &srch, &where);
	if (te != NULL) {
		dsf_t *dsf = dsf_init(te->path);
		if (dem_dsf_check(dsf, demi_p, demd_p)) {
			dbg_log(tile, 3, "load dsf (cached) %d x %d = %s",
			    lat, lon, te->path);
			return (dsf);
		}
		if (dsf != NULL)
			dsf_fini(dsf);
		logMsg("DSF load error: we've seen %s previously, but now "
		    "it's gone. Please do not change your scenery while "
		    "X-Plane is running, no good will come of it.", te->path);
		return (NULL);
	}

	dbg_log(tile, 3, "load dsf %d x %d = (%s/%s)", lat, lon, dname, fname);

	for (srch_path_t *sp = list_head(&srch_paths); sp != NULL;
	    sp = list_next(&srch_paths, sp)) {
		char *path = mkpathname(sp->path, dname, fname, NULL);
		dsf_t *dsf = dsf_init(path);

		if (dem_dsf_check(dsf, demi_p, demd_p)) {
			ASSERT3P(te, ==, NULL);
			te = safe_calloc(1, sizeof (*te));
			strlcpy(te->fname, fname, sizeof (te->fname));
			te->path = path;
			avl_add(&tnlc, te);

			dbg_log(tile, 3, "load dsf %d x %d = %s", lat, lon,
			    path);
			return (dsf);
		}
		if (dsf != NULL)
			dsf_fini(dsf);
		free(path);

		if (!load_dem_worker_wk.run)
			break;
	}

	dbg_log(tile, 3, "load dsf %d x %d = (not found)", lat, lon);

	return (NULL);
}

/*
 * Renders a water-filled empty water mask. This is for cases where we
 * don't have a shape file. This can happen when using Earth Orbit Textures
 * for the height data (which cover te entire globe, but we don't have
 * water shapefiles for ocean tiles).
 */
static cairo_surface_t *
render_empty_water_mask(unsigned pix_width, unsigned pix_height)
{
	cairo_surface_t *surf = cairo_image_surface_create(CAIRO_FORMAT_A8,
	    pix_width, pix_height);
	cairo_t *cr = cairo_create(surf);

	cairo_set_source_rgb(cr, 1, 1, 1);
	cairo_paint(cr);
	cairo_surface_flush(surf);
	cairo_destroy(cr);

	return (surf);
}

/*
 * Reads the water shape file and rasterizes it into an 8-bit water mask.
 * The pixels are the presence of water, from 0 for "no water" to 255
 * "all water". Partway values allow for gradual coloring of shorelines.
 */
static cairo_surface_t *
render_water_mask(int lat, int lon, int pix_width, int pix_height)
{
	char dname[16];
	char fname[16];
	char *path;
	cairo_surface_t *surf = NULL;
	cairo_t *cr;
	SHPHandle shp = NULL;
	int n_ent, shp_type;
	bool_t isdir;

	snprintf(dname, sizeof (dname), "%+03.0f%+04.0f",
	    floor(lat / 10.0) * 10.0, floor(lon / 10.0) * 10.0);
	snprintf(fname, sizeof (fname), "%+03d%+04d.shp", lat, lon);
	path = mkpathname(get_xpdir(), "Resources", "map data", "water",
	    dname, fname, NULL);
	if (!file_exists(path, &isdir) || isdir) {
		surf = render_empty_water_mask(pix_width, pix_height);
		goto out;
	}

	shp = SHPOpen(path, "rb");
	if (shp == NULL) {
		surf = render_empty_water_mask(pix_width, pix_height);
		goto out;
	}
	SHPGetInfo(shp, &n_ent, &shp_type, NULL, NULL);
	/* We only support polygons, given that that's what X-Plane uses. */
	if (shp_type != SHPT_POLYGON) {
		surf = render_empty_water_mask(pix_width, pix_height);
		goto out;
	}

	surf = cairo_image_surface_create(CAIRO_FORMAT_A8, pix_width,
	    pix_height);
	cr = cairo_create(surf);
	cairo_scale(cr, pix_width, pix_height);
	cairo_set_source_rgb(cr, 1, 1, 1);

	/*
	 * The shape file consists of a series of objects, each consisting of
	 * a series of disconnected parts, which nicely map to cairo paths.
	 * This is used in the shape file to cut out inset portions. So we
	 * constract subpaths for each new part and Cairo handles all the
	 * cutting & joining of rendered pieces of the path.
	 */
	for (int i = 0; i < n_ent; i++) {
		SHPObject *obj = SHPReadObject(shp, i);

		if (obj == NULL)
			continue;
		cairo_new_path(cr);
		for (int j = 0; j < obj->nParts; j++) {
			int start_k, end_k;

			start_k = obj->panPartStart[j];
			if (j + 1 < obj->nParts)
				end_k = obj->panPartStart[j + 1];
			else
				end_k = obj->nVertices;
			cairo_new_sub_path(cr);
			/*
			 * Note that the rendered image will be "upside down"
			 * when viewed as a PNG. That's because cairo & PNG
			 * address the image from the top left, but our terrain
			 * coordinates start at the bottom left. So we flip the
			 * Y axis, so that increasing row numbers will
			 * correspond to increasing latitude.
			 */
			cairo_move_to(cr, obj->padfX[start_k] - lon,
			    obj->padfY[start_k] - lat);
			for (int k = start_k + 1; k < end_k; k++) {
				cairo_line_to(cr, obj->padfX[k] - lon,
				    obj->padfY[k] - lat);
			}
		}
		cairo_fill(cr);
		SHPDestroyObject(obj);
	}
	cairo_surface_flush(surf);
	cairo_destroy(cr);
out:
	if (shp != NULL)
		SHPClose(shp);
	lacf_free(path);

	return (surf);
}

static inline double
demd_read(const dsf_atom_t *demi, const dsf_atom_t *demd,
    unsigned row, unsigned col)
{
	double v = 0;

#define	DEMD_READ(data_type, val) \
	do { \
		(val) = ((data_type *)demd->payload)[row * \
		    demi->demi_atom.width + col] * \
		    demi->demi_atom.scale + demi->demi_atom.offset; \
	} while (0)
	switch (demi->demi_atom.flags & DEMI_DATA_MASK) {
	case DEMI_DATA_FP32:
		DEMD_READ(float, v);
		break;
	case DEMI_DATA_SINT:
		switch (demi->demi_atom.bpp) {
		case 1:
			DEMD_READ(int8_t, v);
			break;
		case 2:
			DEMD_READ(int16_t, v);
			break;
		case 4:
			DEMD_READ(int32_t, v);
			break;
		}
		break;
	case DEMI_DATA_UINT:
		switch (demi->demi_atom.bpp) {
		case 1:
			DEMD_READ(uint8_t, v);
			break;
		case 2:
			DEMD_READ(uint16_t, v);
			break;
		case 4:
			DEMD_READ(uint32_t, v);
			break;
		}
		break;
	default:
		VERIFY(0);
	}

#undef	DEMD_READ

	return (v);
}

static void
tile_set_data(dem_tile_t *tile, unsigned pix_width, unsigned pix_height,
    double load_res, int16_t *pixels, cairo_surface_t *water_mask_surf)
{
	ASSERT(tile != NULL);
	ASSERT(pixels != NULL);

	mutex_enter(&dem_tile_cache_lock);

	tile->pix_width = pix_width;
	tile->pix_height = pix_height;
	free(tile->pixels);
	tile->pixels = pixels;
	tile->load_res = load_res;

	if (tile->water_mask_surf != NULL) {
		cairo_surface_destroy(tile->water_mask_surf);
		tile->water_mask = NULL;
		tile->water_mask_stride = 0;
		tile->water_mask_surf = NULL;
	}
	if (water_mask_surf != NULL) {
		tile->water_mask_surf = water_mask_surf;
		tile->water_mask =
		    cairo_image_surface_get_data(water_mask_surf);
		tile->water_mask_stride =
		    cairo_image_surface_get_stride(water_mask_surf);
	}

	mutex_exit(&dem_tile_cache_lock);
}

static void
render_dem_tile(dem_tile_t *tile, const dsf_atom_t *demi,
    const dsf_atom_t *demd, double load_res)
{
	double tile_width = (EARTH_CIRC / 360) * cos(DEG2RAD(tile->lat));
	unsigned dsf_width, dsf_height;
	unsigned pix_width, pix_height;
	double dsf_res_x, dsf_res_y;
	double dsf_scale_x, dsf_scale_y;
	int16_t *pixels;
	cairo_surface_t *water_mask_surf = NULL;

	dsf_width = demi->demi_atom.width;
	dsf_height = demi->demi_atom.height;
	dsf_res_x = tile_width / dsf_width;
	dsf_res_y = TILE_HEIGHT / dsf_height;
	ASSERT3F(dsf_res_x, >=, DSF_DEMI_MIN_RES);
	ASSERT3F(dsf_res_y, >=, DSF_DEMI_MIN_RES);
	dsf_scale_x = load_res / dsf_res_x;
	dsf_scale_y = load_res / dsf_res_y;

	pix_width = tile_width / load_res;
	pix_height = TILE_HEIGHT / load_res;
	ASSERT3U(pix_width, >=, 3);
	ASSERT3U(pix_width, <=, MAX_TILE_PIXELS);
	ASSERT3U(pix_height, >=, 3);
	ASSERT3U(pix_height, <=, MAX_TILE_PIXELS);
	pixels = safe_malloc(pix_width * pix_height * sizeof (*pixels));

	dbg_log(tile, 3, "DSF render tile %d x %d (%d x %d)",
	    tile->lat, tile->lon, pix_width, pix_height);

	for (unsigned y = 0; y < pix_height; y++) {
		for (unsigned x = 0; x < pix_width; x++) {
			int dsf_x = clampi(round(x * dsf_scale_x),
			    0, demi->demi_atom.width - 1);
			int dsf_y = clampi(round(y * dsf_scale_y),
			    0, demi->demi_atom.height - 1);
			int elev = demd_read(demi, demd, dsf_y, dsf_x);

			pixels[y * pix_width + x] =
			    ELEV_WRITE(clampi(elev, INT16_MIN, INT16_MAX));
		}
	}

	water_mask_surf = render_water_mask(tile->lat, tile->lon,
	    pix_width, pix_height);

	tile_set_data(tile, pix_width, pix_height, load_res, pixels,
	    water_mask_surf);
}

static bool_t
load_earth_orbit_tex(dem_tile_t *tile, double load_res)
{
	double tile_width = (EARTH_CIRC / 360) * cos(DEG2RAD(tile->lat));
	double scale_x, scale_y;
	unsigned pix_width, pix_height;
	int png_width, png_height, color_type, bit_depth;
	double tile_off_lat, tile_off_lon;
	double tile_off_x, tile_off_y;
	int16_t *pixels;
	cairo_surface_t *water_mask_surf = NULL;
	uint8_t *png_pixels;
	char *path;
	char filename[32];

	snprintf(filename, sizeof (filename), "%+03.0f%+04.0f-nrm.png",
	    floor(tile->lat / 10.0) * 10, floor(tile->lon / 10.0) * 10);
	path = mkpathname(get_xpdir(), "Resources", "bitmaps",
	    "Earth Orbit Textures", filename, NULL);
	png_pixels = png_load_from_file_rgba_auto(path,
	    &png_width, &png_height, &color_type, &bit_depth);
	lacf_free(path);
	if (png_pixels == NULL)
		return (B_FALSE);
	if (png_width < 1024 || png_height < 1024) {
		lacf_free(png_pixels);
		return (B_FALSE);
	}

	pix_width = tile_width / load_res;
	pix_height = TILE_HEIGHT / load_res;
	pixels = safe_malloc(pix_width * pix_height * sizeof (*pixels));
	scale_x = (png_width / 10.0) / pix_width;
	scale_y = (png_height / 10.0) / pix_height;
	/* PNG origin point is on the top left, not bottom left */
	tile_off_lat = 9 - (tile->lat - (floor(tile->lat / 10.0) * 10));
	tile_off_lon = tile->lon - (floor(tile->lon / 10.0) * 10);
	ASSERT3F(tile_off_lat, >=, 0.0);
	ASSERT3F(tile_off_lat, <, 10.0);
	tile_off_x = png_width * (tile_off_lon / 10.0);
	tile_off_y = png_height * (tile_off_lat / 10.0);

	dbg_log(tile, 3, "EOT render tile %d x %d (%d x %d)",
	    tile->lat, tile->lon, pix_width, pix_height);

	for (unsigned y = 0; y < pix_height; y++) {
		for (unsigned x = 0; x < pix_width; x++) {
			double png_x = (x * scale_x + tile_off_x);
			double png_y = (y * scale_y + tile_off_y);
			int png_x_lo = clamp(floor(png_x), 0, png_width - 1);
			int png_x_hi = clamp(ceil(png_x), 0, png_width - 1);
			int png_y_lo = clamp(floor(png_y), 0, png_height - 1);
			int png_y_hi = clamp(ceil(png_y), 0, png_height - 1);
			double x_fract = png_x - png_x_lo;
			double y_fract = png_y - png_y_lo;
			uint8_t ul = png_pixels[sizeof (uint32_t) *
			    (png_x_lo + png_y_lo * png_width) + 3];
			uint8_t ur = png_pixels[sizeof (uint32_t) *
			    (png_x_hi + png_y_lo * png_width) + 3];
			uint8_t ll = png_pixels[sizeof (uint32_t) *
			    (png_x_lo + png_y_hi * png_width) + 3];
			uint8_t lr = png_pixels[sizeof (uint32_t) *
			    (png_x_hi + png_y_hi * png_width) + 3];
			double raw_val = wavg(wavg(ul, ur, x_fract),
			    wavg(ll, lr, x_fract), y_fract);
			double elev = fx_lin(raw_val, 0, EOT_ELEV_MAX,
			    255, EOT_ELEV_MIN);

			pixels[x + (pix_height - y - 1) * pix_width] =
			    ELEV_WRITE(elev);
		}
	}

	water_mask_surf = render_water_mask(tile->lat, tile->lon,
	    pix_width, pix_height);

	tile_set_data(tile, pix_width, pix_height, load_res, pixels,
	    water_mask_surf);

	lacf_free(png_pixels);

	return (B_TRUE);
}

static int
load_dem_tile(int lat, int lon, double load_res)
{
	dem_tile_t srch = { .lat = lat, .lon = lon };
	dem_tile_t *tile;
	bool_t existing;
	dsf_t *dsf = NULL;
	const dsf_atom_t *demi = NULL, *demd = NULL;

	ASSERT3F(load_res, >, 0);

	mutex_enter(&dem_tile_cache_lock);
	tile = avl_find(&dem_tile_cache, &srch, NULL);
	if (tile != NULL && tile->remove) {
		int bytes = tile->pix_width * tile->pix_height * 2;
		/* Tile being removed, don't touch it until it is gone */
		mutex_exit(&dem_tile_cache_lock);
		return (bytes);
	}
	mutex_exit(&dem_tile_cache_lock);

	existing = (tile != NULL);

	if (tile != NULL &&
	    (tile->empty || tile->dirty || tile->load_res == load_res)) {
		int bytes = tile->pix_width * tile->pix_height * 2;
		tile->seen = B_TRUE;
		return (bytes);
	}

	if (tile == NULL) {
		tile = safe_calloc(1, sizeof (*tile));
		tile->lat = lat;
		tile->lon = lon;
		tile->cur_tex = -1;
	}

	dsf = load_dem_dsf(lat, lon, &demi, &demd);

	dbg_log(tile, 2, "load tile %d x %d (dsf: %p)", lat, lon, dsf);

	if (dsf != NULL) {
		ASSERT(!tile->empty);
		render_dem_tile(tile, demi, demd, load_res);
		ASSERT(tile->pixels != NULL);
		/* marks the tile for GPU upload */
		tile->dirty = B_TRUE;
	} else if (load_earth_orbit_tex(tile, load_res)) {
		ASSERT(!tile->empty);
		ASSERT(tile->pixels != NULL);
		/* marks the tile for GPU upload */
		tile->dirty = B_TRUE;
	} else {
		tile->empty = B_TRUE;
	}

	if (!existing) {
		mutex_enter(&dem_tile_cache_lock);
		avl_add(&dem_tile_cache, tile);
		mutex_exit(&dem_tile_cache_lock);
	} else {
		dbg_log(tile, 2, "tile res chg %f -> %f",
		    tile->load_res, load_res);
	}

	if (dsf != NULL)
		dsf_fini(dsf);

	tile->seen = B_TRUE;

	return (tile->pix_width * tile->pix_height *
	    (2 + (tile->water_mask != NULL ? 1 : 0)));
}

static double
select_tile_res(int tile_lat, int tile_lon, fpp_t fpp)
{
	double dist;
	const egpws_range_t *rngs = ranges;
	vect2_t tile_poly[5];
	vect2_t tile_ctr, isect;

	tile_poly[0] = geo2fpp(GEO_POS2(tile_lat, tile_lon), &fpp);
	tile_poly[1] = geo2fpp(GEO_POS2(tile_lat + 1, tile_lon), &fpp);
	tile_poly[2] = geo2fpp(GEO_POS2(tile_lat + 1, tile_lon + 1), &fpp);
	tile_poly[3] = geo2fpp(GEO_POS2(tile_lat, tile_lon + 1), &fpp);
	tile_poly[4] = tile_poly[0];
	tile_ctr = geo2fpp(GEO_POS2(tile_lat + 0.5, tile_lon + 0.5), &fpp);

	for (int i = 0; i < 4; i++) {
		isect = vect2vect_isect(tile_ctr, ZERO_VECT2,
		    vect2_sub(tile_poly[i + 1], tile_poly[i]), tile_poly[i],
		    B_TRUE);
		if (!IS_NULL_VECT(isect))
			break;
	}
	if (!IS_NULL_VECT(isect))
		dist = vect2_abs(isect);
	else
		/* we must be inside the tile */
		dist = 0;

	for (int i = 0; !isnan(rngs[i].range); i++) {
		if (dist < rngs[i].range) {
			dbg_log(tile, 2, "tile res %d x %d (dist: %.0f) = %.0f",
			    tile_lat, tile_lon, dist, rngs[i].resolution);
			ASSERT3F(rngs[i].resolution, >, 0);
			return (rngs[i].resolution);
		}
	}

	dbg_log(tile, 2, "tile res fallback %d x %d = min", tile_lat, tile_lon);

	return (MIN_RES);
}

static void
load_nrst_dem_tiles(void)
{
	int tile_width;
	int h_tiles, v_tiles;
	const egpws_range_t *rngs = ranges;
	double rng = MIN_RNG;
	int n_tiles = 0, bytes = 0;
	geo_pos3_t pos;

	mutex_enter(&glob_pos_lock);
	pos = glob_pos;
	mutex_exit(&glob_pos_lock);

	for (int i = 0; !isnan(rngs[i].range); i++)
		rng = MAX(rng, rngs[i].range);

	ASSERT3F(ABS(pos.lat), <=, MAX_LAT);
	tile_width = (EARTH_CIRC / 360) * cos(DEG2RAD(pos.lat));
	h_tiles = MAX(rng / tile_width, 1);
	v_tiles = MAX(rng / TILE_HEIGHT, 1);

	dbg_log(tile, 1, "load nrst tiles %d x %d", h_tiles, v_tiles);

	for (int v = -v_tiles; v <= v_tiles; v++) {
		for (int h = -h_tiles; h <= h_tiles; h++) {
			double res;
			int tile_lat = floor(pos.lat + v);
			int tile_lon = floor(pos.lon + h);
			fpp_t fpp;

			/*
			 * Refresh the position to grab the tile resolution.
			 * We need to do this because tile loading is a pretty
			 * slow process, and if the aircraft is dragged around
			 * the map during a single run through this function
			 * (which can take minutes to complete), we could be
			 * loading tiles with the wrong resolution given the
			 * updated aircraft position.
			 */
			mutex_enter(&glob_pos_lock);
			pos = glob_pos;
			mutex_exit(&glob_pos_lock);

			fpp = stereo_fpp_init(GEO3_TO_GEO2(pos), 0, &wgs84,
			    B_FALSE);

			if (!load_dem_worker_wk.run)
				return;

			res = select_tile_res(tile_lat, tile_lon, fpp);
			bytes += load_dem_tile(tile_lat, tile_lon, res);
			n_tiles++;
		}
	}
	dbg_log(tile, 1, "loaded %d tiles, total %d MB", n_tiles, bytes >> 20);
}

static void
free_dem_tile(dem_tile_t *tile)
{
	dbg_log(tile, 2, "free tile %d x %d", tile->lat, tile->lon);
	for (int i = 0; i < 2; i++) {
		if (tile->tex[i] != 0)
			glDeleteTextures(1, &tile->tex[i]);
	}
	if (tile->pbo != 0)
		glDeleteBuffers(1, &tile->pbo);
	glutils_destroy_quads(&tile->quads);
	if (tile->water_mask_surf != NULL)
		cairo_surface_destroy(tile->water_mask_surf);
	free(tile->pixels);
	free(tile);
}

static void
append_cust_srch_paths(void)
{
	char *contents;
	char **lines;
	size_t n_lines;

	contents = file2str(get_xpdir(), "Custom Scenery",
	    "scenery_packs.ini", NULL);

	if (contents == NULL)
		return;
	lines = strsplit(contents, "\n", B_TRUE, &n_lines);
	for (size_t i = 0; i < n_lines; i++) {
		char buf[256];
		char *subpath;
		bool_t is_dir;

		strip_space(lines[i]);
		if (strstr(lines[i], "SCENERY_PACK") != lines[i] ||
		    !isspace(lines[i][12]))
			continue;
		strlcpy(buf, &lines[i][12], sizeof (buf));
		strip_space(buf);
		subpath = mkpathname(get_xpdir(), buf, "Earth nav data", NULL);

		if (file_exists(subpath, &is_dir) && is_dir) {
			srch_path_t *path = safe_calloc(1, sizeof (*path));

			path->path = subpath;
			list_insert_tail(&srch_paths, path);
		} else {
			free(subpath);
		}
	}

	free(contents);
}

static void
append_glob_srch_paths(void)
{
	char *path = mkpathname(get_xpdir(), "Global Scenery", NULL);
	DIR *dp = opendir(path);
	struct dirent *de;

	if (dp == NULL)
		goto errout;

	while ((de = readdir(dp)) != NULL) {
		char *subpath = mkpathname(path, de->d_name, "Earth nav data",
		    NULL);
		bool_t is_dir;

		if (file_exists(subpath, &is_dir) && is_dir) {
			srch_path_t *path = safe_calloc(1, sizeof (*path));
			path->path = subpath;
			list_insert_tail(&srch_paths, path);
			dbg_log(tile, 2, "srch path %s\n", subpath);
		} else {
			free(subpath);
		}
	}
	closedir(dp);
errout:
	free(path);
}

static bool_t
load_dem_worker(void *unused)
{
	geo_pos3_t pos;

	UNUSED(unused);

	dbg_log(tile, 4, "loop");

	/* No search paths yet, initialize them */
	if (list_head(&srch_paths) == NULL) {
		append_cust_srch_paths();
		append_glob_srch_paths();
	}

	mutex_enter(&glob_pos_lock);
	pos = glob_pos;
	mutex_exit(&glob_pos_lock);

	/*
	 * Mark all tiles as unseen. The load process will mark
	 * the ones we want to keep.
	 */
	mutex_enter(&dem_tile_cache_lock);
	for (dem_tile_t *tile = avl_first(&dem_tile_cache);
	    tile != NULL; tile = AVL_NEXT(&dem_tile_cache, tile)) {
		tile->seen = B_FALSE;
	}
	mutex_exit(&dem_tile_cache_lock);

	if (!IS_NULL_GEO_POS(pos))
		load_nrst_dem_tiles();

	/* Unload unseen tiles */
	mutex_enter(&dem_tile_cache_lock);
	for (dem_tile_t *tile = avl_first(&dem_tile_cache),
	    *next = NULL; tile != NULL; tile = next) {
		next = AVL_NEXT(&dem_tile_cache, tile);
		if (!tile->seen) {
			/*
			 * Must be removed by foreground thread to properly
			 * dispose of OpenGL objects.
			 */
			tile->remove = B_TRUE;
		}
	}
	mutex_exit(&dem_tile_cache_lock);

	return (B_TRUE);
}

void
terr_init(void)
{
	ASSERT(!inited);
	inited = B_TRUE;

	mutex_init(&glob_pos_lock);

	mutex_init(&dem_tile_cache_lock);
	avl_create(&dem_tile_cache, dem_tile_compar, sizeof (dem_tile_t),
	    offsetof(dem_tile_t, cache_node));
	avl_create(&tnlc, tnlc_compar, sizeof (tnlc_ent_t),
	    offsetof(tnlc_ent_t, tnlc_node));

	list_create(&srch_paths, sizeof (srch_path_t),
	    offsetof(srch_path_t, node));

	worker_init(&load_dem_worker_wk, load_dem_worker,
	    SEC2USEC(LOAD_DEM_WORKER_INTVAL), NULL, "OpenGPWS_terr_wk");

	terr_reload_gl_progs();
}

void
terr_fini(void)
{
	srch_path_t *path;
	dem_tile_t *tile;
	void *cookie;
	tnlc_ent_t *te;

	if (!inited)
		return;
	inited = B_FALSE;

	worker_fini(&load_dem_worker_wk);

	for (int i = 0; i < 180; i++) {
		for (int j = 0; j < 360; j++)
			free(dsf_paths[i][j]);
	}

	while ((path = list_remove_head(&srch_paths)) != NULL) {
		free(path->path);
		free(path);
	}
	list_destroy(&srch_paths);

	cookie = NULL;
	while ((tile = avl_destroy_nodes(&dem_tile_cache, &cookie)) != NULL)
		free_dem_tile(tile);
	avl_destroy(&dem_tile_cache);
	mutex_destroy(&dem_tile_cache_lock);

	cookie = NULL;
	while ((te = avl_destroy_nodes(&tnlc, &cookie)) != NULL) {
		free(te->path);
		free(te);
	}
	avl_destroy(&tnlc);

	mutex_destroy(&glob_pos_lock);

	if (DEM_prog != 0) {
		glDeleteProgram(DEM_prog);
		DEM_prog = 0;
	}
}

void
terr_set_pos(geo_pos3_t new_pos)
{
	mutex_enter(&glob_pos_lock);

	if (ABS(new_pos.lat) >= MAX_LAT) {
		glob_pos = NULL_GEO_POS3;
	} else {
		new_pos.elev = round(new_pos.elev / ALT_QUANT_STEP) *
		    ALT_QUANT_STEP;
		glob_pos = new_pos;
	}
	dbg_log(terr, 2, "set pos %.4f x %.4f x %.0f",
	    glob_pos.lat, glob_pos.lon, glob_pos.elev);

	mutex_exit(&glob_pos_lock);
}

void
terr_set_ranges(const egpws_range_t *new_ranges)
{
	if (new_ranges != NULL)
		ranges = new_ranges;
	else
		ranges = dfl_ranges;
}

double
terr_get_elev(geo_pos2_t pos)
{
	double elev = NAN;
	dem_tile_t srch = { .lat = floor(pos.lat), .lon = floor(pos.lon) };
	dem_tile_t *tile;

	mutex_enter(&dem_tile_cache_lock);
	tile = avl_find(&dem_tile_cache, &srch, NULL);
	if (tile != NULL && !tile->empty) {
		int x = clampi((pos.lon - tile->lon) * tile->pix_width,
		    0, tile->pix_width - 1);
		int y = clampi((pos.lat - tile->lat) * tile->pix_height,
		    0, tile->pix_height - 1);
		ASSERT(tile->pixels != NULL);
		elev = ELEV_READ(tile->pixels[y * tile->pix_width + x]);
	}
	mutex_exit(&dem_tile_cache_lock);

	dbg_log(terr, 3, "get elev (%.4fx%.4f) = %.0f\n", pos.lat, pos.lon,
	    elev);

	return (elev);
}

double
terr_get_elev_wide(geo_pos2_t pos, geo_pos3_t terr_pos[9])
{
	double elev = NAN;
	dem_tile_t srch = { .lat = floor(pos.lat), .lon = floor(pos.lon) };
	dem_tile_t *tile;
	int x, y, i;

	mutex_enter(&dem_tile_cache_lock);

	tile = avl_find(&dem_tile_cache, &srch, NULL);
	if (tile == NULL || tile->empty) {
		dbg_log(terr, 3, "get elev (%.4fx%.4f) = nan\n",
		    pos.lat, pos.lon);
		mutex_exit(&dem_tile_cache_lock);
		return (NAN);
	}

	x = clampi(round((pos.lon - tile->lon) * tile->pix_width),
	    0, tile->pix_width - 1);
	y = clampi(round((pos.lat - tile->lat) * tile->pix_height),
	    0, tile->pix_height - 1);
	i = 0;
	ASSERT(tile->pixels != NULL);

	for (int xi = -1; xi <= 1; xi++) {
		for (int yi = -1; yi <= 1; yi++) {
			int xx = clampi(x + xi, 0, tile->pix_width - 1);
			int yy = clampi(y + yi, 0, tile->pix_height - 1);
			double e =
			    ELEV_READ(tile->pixels[yy * tile->pix_width + xx]);

			if (terr_pos != NULL) {
				terr_pos[i++] = GEO_POS3(
				    tile->lat + ((double)yy / tile->pix_height),
				    tile->lon + ((double)xx / tile->pix_width),
				    e);
			}

			if (isnan(elev))
				elev = e;
			else if (e > elev)
				elev = e;
		}
	}

	mutex_exit(&dem_tile_cache_lock);

	dbg_log(terr, 3, "get elev (%.4fx%.4f) = %.0f\n",
	    pos.lat, pos.lon, elev);

	return (elev);
}

static void
update_tiles(void)
{
	mutex_enter(&dem_tile_cache_lock);

	dbg_log(terr, 3, "Updating %lu tiles", avl_numnodes(&dem_tile_cache));

	for (dem_tile_t *tile = avl_first(&dem_tile_cache), *next_tile = NULL;
	    tile != NULL; tile = next_tile) {
		int next_tex = !tile->cur_tex;

		next_tile = AVL_NEXT(&dem_tile_cache, tile);

		if (tile->remove) {
			avl_remove(&dem_tile_cache, tile);
			free_dem_tile(tile);
			continue;
		}
		if (tile->empty || !tile->dirty)
			continue;
		if (tile->tex[next_tex] == 0) {
			glGenTextures(1, &tile->tex[next_tex]);
			glBindTexture(GL_TEXTURE_2D, tile->tex[next_tex]);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			    GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			    GL_NEAREST);
		}
		if (tile->pbo == 0)
			glGenBuffers(1, &tile->pbo);

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, tile->pbo);

		if (tile->in_flight == 0) {
			size_t sz = tile->pix_width * tile->pix_height *
			    sizeof (*tile->pixels);
			void *ptr;

			glBufferData(GL_PIXEL_UNPACK_BUFFER, sz, 0,
			    GL_STREAM_DRAW);
			ptr = glMapBuffer(GL_PIXEL_UNPACK_BUFFER,
			    GL_WRITE_ONLY);
			if (ptr == NULL) {
				logMsg("Error updating tile %d x %d: "
				    "glMapBuffer returned NULL", tile->lat,
				    tile->lon);
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
				continue;
			}
			memcpy(ptr, tile->pixels, sz);

			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
			tile->in_flight =
			    glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
		} else if (glClientWaitSync(tile->in_flight, 0, 0) !=
		    GL_TIMEOUT_EXPIRED) {
			glBindTexture(GL_TEXTURE_2D, tile->tex[next_tex]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RG, tile->pix_width,
			    tile->pix_height, 0, GL_RG, GL_UNSIGNED_BYTE, NULL);

			tile->dirty = B_FALSE;
			glDeleteSync(tile->in_flight);
			tile->in_flight = 0;
			tile->cur_tex = next_tex;
		}

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	mutex_exit(&dem_tile_cache_lock);
}

static double
render_get_range(const egpws_render_t *render)
{
	double range, r;

	ASSERT(render != NULL);

	range = vect2_abs(render->offset);
	r = vect2_abs(vect2_sub(render->disp_sz, render->offset));
	range = MAX(r, range);
	r = vect2_abs(vect2_sub(VECT2(0, render->disp_sz.y), render->offset));
	range = MAX(r, range);
	r = vect2_abs(vect2_sub(VECT2(render->disp_sz.x, 0), render->offset));
	range = MAX(r, range);

	return (range / render->scale);
}

/*
 * The maximum diagonal dimension of a 1x1 degree tile on Earth.
 * The maximum one-side dimension is 40,000 km (Earth's circumference)
 * divided into 360 pieces (111.111km). Take half of that (55.555 km)
 * and treat it as two sides of a right-angle triangle. The hypotenuse
 * is the maximum we could be from the tile's centerpoint coordinate
 * (half-degree midpoint) while still being "inside" of the tile. Any
 * further out, and we're definitely outside of it.
 */
#define	TILE_MAX_DIAG_SEMI_DIM	78567	/* m */

static void
draw_tiles(const egpws_render_t *render)
{
	fpp_t fpp;
	const egpws_conf_t *conf = egpws_get_conf();
	mat4 pvm;
	GLfloat hgt_rngs_ft[4 * 2];
	GLfloat hgt_colors[4 * 4];
	GLuint prog;
	double range;

	ASSERT(render != NULL);
	fpp = terr_render_get_fpp_impl(render, false);
	range = render_get_range(render);

	glm_ortho(0, render->disp_sz.x, 0, render->disp_sz.y, 0, 1, pvm);
	glm_translate(pvm, (vec3){render->offset.x, render->offset.y, 0});
	glm_rotate_z(pvm, DEG2RAD(render->rotation), pvm);

	for (int i = 0; i < 4; i++) {
		const egpws_terr_color_t *color = &conf->terr_colors[i];
		hgt_rngs_ft[i * 2] = MET2FEET(color->min_hgt);
		hgt_rngs_ft[i * 2 + 1] = MET2FEET(color->max_hgt);
		for (int j = 0; j < 4; j++)
			hgt_colors[i * 4 + j] = color->rgba[j];
	}

	glActiveTexture(GL_TEXTURE0);

	prog = (render->prog != 0 ? render->prog : DEM_prog);
	glUseProgram(prog);

	glUniformMatrix4fv(glGetUniformLocation(prog, "pvm"),
	    1, GL_FALSE, (GLfloat *)pvm);
	glUniform1i(glGetUniformLocation(prog, "tex"), 0);
	glUniform1f(glGetUniformLocation(prog, "acf_elev_ft"),
	    MET2FEET(glob_pos.elev));
	glUniform2fv(glGetUniformLocation(prog, "hgt_rngs_ft"),
	    4 * 2, hgt_rngs_ft);
	glUniform4fv(glGetUniformLocation(prog, "hgt_colors"),
	    4 * 4, hgt_colors);

	mutex_enter(&dem_tile_cache_lock);
	for (dem_tile_t *tile = avl_first(&dem_tile_cache);
	    tile != NULL; tile = AVL_NEXT(&dem_tile_cache, tile)) {
		static const vect2_t t[4] = {
		    VECT2(0, 0), VECT2(0, 1), VECT2(1, 1), VECT2(1, 0)
		};
		vect2_t v[4];
		geo_pos2_t tile_ctr;

		if (tile->cur_tex == -1)
			continue;
		/*
		 * Check if the tile is outside of display range.
		 */
		tile_ctr = GEO_POS2(tile->lat + 0.5, tile->lon + 0.5);
		if (gc_distance(TO_GEO2(render->position), tile_ctr) >
		    range + TILE_MAX_DIAG_SEMI_DIM) {
			continue;
		}
		v[0] = geo2fpp(GEO_POS2(tile->lat, tile->lon), &fpp);
		v[1] = geo2fpp(GEO_POS2(tile->lat + 1, tile->lon), &fpp);
		v[2] = geo2fpp(GEO_POS2(tile->lat + 1, tile->lon + 1), &fpp);
		v[3] = geo2fpp(GEO_POS2(tile->lat, tile->lon + 1), &fpp);
		for (int i = 0; i < 4; i++) {
			v[i].x = round(v[i].x * 2) / 2;
			v[i].y = round(v[i].y * 2) / 2;
		}
		glBindTexture(GL_TEXTURE_2D, tile->tex[tile->cur_tex]);
		/*
		 * Only upload new data when the tile has moved.
		 */
		if (memcmp(tile->quads_v, v, sizeof (tile->quads_v)) != 0) {
			memcpy(tile->quads_v, v, sizeof (tile->quads_v));
			if (!glutils_quads_inited(&tile->quads))
				glutils_init_2D_quads(&tile->quads, v, t, 4);
			else
				glutils_update_2D_quads(&tile->quads, v, t, 4);
		}
		glutils_draw_quads(&tile->quads, prog);
	}
	mutex_exit(&dem_tile_cache_lock);

	glUseProgram(0);
}

void
terr_render(const egpws_render_t *render)
{
	ASSERT(inited);
#ifdef	FAST_DEBUG
	glutils_reset_errors();
#endif

	update_tiles();

	if (render->do_draw)
		draw_tiles(render);
#ifdef	FAST_DEBUG
	VERIFY3U(glGetError(), ==, GL_NO_ERROR);
#endif
}

static fpp_t
terr_render_get_fpp_impl(const egpws_render_t *render, bool apply_rot)
{
	fpp_t fpp;

	ASSERT(render != NULL);
	ASSERT(!IS_NULL_GEO_POS2(render->position));
	ASSERT(isfinite(render->rotation));

	fpp = ortho_fpp_init(TO_GEO2(render->position),
	    apply_rot ? render->rotation : 0, NULL, true);
	fpp_set_scale(&fpp, VECT2(render->scale, render->scale));

	return (fpp);
}

fpp_t
terr_render_get_fpp(const egpws_render_t *render)
{
	return (terr_render_get_fpp_impl(render, true));
}

static void
compute_terr_norm_vector(dem_tile_t *tile, unsigned x, unsigned y,
    double elev, vect3_t *out_norm)
{
	double elev_west, elev_east, elev_north, elev_south;
	double dist_lat, dist_lon, slope_lon, slope_lat;
	vect3_t norm_lat, norm_lon;

	if (x == 0) {
		elev_east =
		    ELEV_READ(tile->pixels[y * tile->pix_width + x + 1]);
		elev_west = elev;
		dist_lon = tile->load_res;
	} else if (x + 1 == tile->pix_width) {
		elev_east = elev;
		elev_west =
		    ELEV_READ(tile->pixels[y * tile->pix_width + x - 1]);
		dist_lon = tile->load_res;
	} else {
		elev_east =
		    ELEV_READ(tile->pixels[y * tile->pix_width + x - 1]);
		elev_west =
		    ELEV_READ(tile->pixels[y * tile->pix_width + x + 1]);
		dist_lon = 2 * tile->load_res;
	}
	slope_lon = RAD2DEG(atan((elev_east - elev_west) / dist_lon));
	norm_lon = vect3_rot(VECT3(0, 0, 1), slope_lon, 1);

	if (y == 0) {
		elev_north = elev;
		elev_south =
		    ELEV_READ(tile->pixels[(y + 1) * tile->pix_width + x]);
		dist_lat = tile->load_res;
	} else if (y + 1 == tile->pix_height) {
		elev_north =
		    ELEV_READ(tile->pixels[(y - 1) * tile->pix_width + x]);
		elev_south = elev;
		dist_lat = tile->load_res;
	} else {
		elev_north =
		    ELEV_READ(tile->pixels[(y - 1) * tile->pix_width + x]);
		elev_south =
		    ELEV_READ(tile->pixels[(y + 1) * tile->pix_width + x]);
		dist_lat = 2 * tile->load_res;
	}
	slope_lat = RAD2DEG(atan((elev_south - elev_north) / dist_lat));
	norm_lat = vect3_rot(VECT3(0, 0, 1), slope_lat, 0);

	*out_norm = vect3_unit(vect3_add(norm_lat, norm_lon), NULL);
}

static double
elev_filter_lin(dem_tile_t *tile, geo_pos2_t pos)
{
	double x_f = clamp((pos.lon - tile->lon) * tile->pix_width,
	    0, tile->pix_width - 1);
	double y_f = clamp(((pos.lat - tile->lat) * tile->pix_height),
	    0, tile->pix_height - 1);
	unsigned x_lo = floor(x_f), x_hi = ceil(x_f);
	unsigned y_lo = floor(y_f), y_hi = ceil(y_f);
	double elev1, elev2, elev3, elev4;

	elev1 = ELEV_READ(tile->pixels[y_lo * tile->pix_width + x_lo]);
	elev2 = ELEV_READ(tile->pixels[y_lo * tile->pix_width + x_hi]);
	elev3 = ELEV_READ(tile->pixels[y_hi * tile->pix_width + x_lo]);
	elev4 = ELEV_READ(tile->pixels[y_hi * tile->pix_width + x_hi]);

	return (wavg(wavg(elev1, elev2, x_f - x_lo),
	    wavg(elev3, elev4, x_f - x_lo), y_f - y_lo));
}

void
terr_probe(egpws_terr_probe_t *probe)
{
	dem_tile_t *tile = NULL;

	VERIFY(probe->out_elev != NULL);

	/*
	 * OpenWXR can start asking us for terrain data early,
	 * so just answer that we don't know anything yet.
	 */
	if (!inited) {
		if (probe->out_elev != NULL) {
			memset(probe->out_elev, 0, sizeof (*probe->out_elev) *
			    probe->num_pts);
		}
		if (probe->out_norm != NULL) {
			memset(probe->out_norm, 0, sizeof (*probe->out_norm) *
			    probe->num_pts);
		}
		if (probe->out_water != NULL) {
			memset(probe->out_water, 0, sizeof (*probe->out_water) *
			    probe->num_pts);
		}
		return;
	}

	mutex_enter(&dem_tile_cache_lock);

	for (unsigned i = 0; i < probe->num_pts; i++) {
		geo_pos2_t pos = probe->in_pts[i];
		int tile_lat = floor(pos.lat), tile_lon = floor(pos.lon);
		unsigned x, y;
		double elev;

		/*
		 * Check if we can recycle the previous tile. Most probes
		 * will tend to cluster in the same tile, so avoid having
		 * to do multiple avl_find()s.
		 */
		if (tile == NULL ||
		    tile->lat != tile_lat || tile->lon != tile_lon) {
			dem_tile_t srch = { .lat = tile_lat, .lon = tile_lon };
			tile = avl_find(&dem_tile_cache, &srch, NULL);
		}
		if (tile == NULL || tile->empty) {
			probe->out_elev[i] = 0;
			if (probe->out_norm != NULL)
				probe->out_norm[i] = VECT3(0, 0, 1);
			/*
			 * If there is no terrain tile, then X-Plane will draw
			 * water here. So we'll assume water as well.
			 */
			if (probe->out_water != NULL)
				probe->out_water[i] = 1.0;
			continue;
		}
		ASSERT3U(tile->pix_width, >=, 3);
		ASSERT3U(tile->pix_height, >=, 3);
		ASSERT(tile->pixels != NULL);

		x = clampi(round((pos.lon - tile->lon) * tile->pix_width),
		    0, tile->pix_width - 1);
		y = clampi(round((pos.lat - tile->lat) * tile->pix_height),
		    0, tile->pix_height - 1);
		elev = ELEV_READ(tile->pixels[y * tile->pix_width + x]);

		if (probe->out_elev != NULL) {
			if (!probe->filter_lin) {
				probe->out_elev[i] = elev;
			} else {
				probe->out_elev[i] =
				    elev_filter_lin(tile, pos);
			}
		}

		if (probe->out_norm != NULL) {
			compute_terr_norm_vector(tile, x, y, elev,
			    &probe->out_norm[i]);
		}

		if (probe->out_water != NULL) {
			if (tile->water_mask != NULL) {
				probe->out_water[i] = tile->water_mask[y *
				    tile->water_mask_stride + x] / 255.0;
			} else {
				/*
				 * If we DO have a DEM tile, but no
				 * corresponding water .shp file, then we
				 * can't really tell what the terrain type
				 * is. But since more than likely it's going
				 * to be terrain, let's pretend it's all dry.
				 */
				probe->out_water[i] = 0.0;
			}
		}
	}

	mutex_exit(&dem_tile_cache_lock);
}

/*
 * Returns B_TRUE if OpenGPWS has data (of any resolution) for the requested
 * latitude and longitude, or B_FALSE if it has no data for the specified
 * point. This can be used as a quick test before querying OpenGPWS via the
 * much more expensive terr_probe interface.
 */
bool_t
terr_have_data(geo_pos2_t pos, double *tile_load_res)
{
	dem_tile_t *tile;
	dem_tile_t srch = { .lat = floor(pos.lat), .lon = floor(pos.lon) };

	mutex_enter(&dem_tile_cache_lock);
	tile = avl_find(&dem_tile_cache, &srch, NULL);
	mutex_exit(&dem_tile_cache_lock);

	if (tile != NULL && !tile->empty) {
		if (tile_load_res != NULL)
			*tile_load_res = tile->load_res;
		return (B_TRUE);
	}
	return (B_FALSE);
}

void
terr_reload_gl_progs(void)
{
	char *vtx_path, *frag_path;
	GLuint prog;

	vtx_path = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "DEM.vert", NULL);
	frag_path = mkpathname(get_xpdir(), get_plugindir(), "data",
	    "DEM.frag", NULL);
	prog = shader_prog_from_file("DEM_shader", vtx_path, frag_path,
	    DEFAULT_VTX_ATTRIB_BINDINGS, NULL);
	lacf_free(vtx_path);
	lacf_free(frag_path);

	if (prog != 0) {
		if (DEM_prog != 0)
			glDeleteProgram(DEM_prog);
		DEM_prog = prog;
	}
}
