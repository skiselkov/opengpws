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
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <ctype.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#include <GL/glew.h>

#include <XPLMGraphics.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dsf.h>
#include <acfutils/helpers.h>
#include <acfutils/list.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/shader.h>
#include <acfutils/worker.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include "dbg_log.h"
#include "egpws.h"
#include "terr.h"

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
#define	ALT_QUANT_STEP		FEET2MET(20)		/* quantization step */

typedef struct {
	int		lat;		/* latitude in degrees */
	int		lon;		/* longitude in degrees */
	double		min_elev;	/* minimum elevation (meters) */
	double		max_elev;	/* maximum elevation (meters) */
	double		load_res;	/* resolution at load time */
	unsigned	pix_width;	/* tile width in pixels */
	unsigned	pix_height;	/* tile height in pixels */
	int16_t		*pixels;	/* elevation data samples */
	bool_t		dirty;		/* pending upload to GPU */
	GLsync		in_flight;	/* data in flight to GPU */
	bool_t		empty;		/* tile contains no data */
	bool_t		seen;		/* seen in presence check */
	bool_t		remove;		/* mark for removal */

	int		cur_tex;
	GLuint		tex[2];
	GLuint		pbo;

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
static char *xpdir = NULL;

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
	    DSF_ATOM_DEMS, 0, DSF_ATOM_DEMD, 0, 0)) != NULL);
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
			te = calloc(1, sizeof (*te));
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

static void
render_dem_tile(dem_tile_t *tile, const dsf_atom_t *demi,
    const dsf_atom_t *demd)
{
	double tile_width = (EARTH_CIRC / 360) * cos(DEG2RAD(tile->lat));
	unsigned dsf_width = demi->demi_atom.width;
	unsigned dsf_height = demi->demi_atom.height;
	double dsf_res_x = tile_width / dsf_width;
	double dsf_res_y = TILE_HEIGHT / dsf_height;
	double dsf_scale_x = tile->load_res / dsf_res_x;
	double dsf_scale_y = tile->load_res / dsf_res_y;
	const uint16_t *dsf_pixels = (const uint16_t *)demd->payload;
	int16_t *pixels;

	tile->pix_width = tile_width / tile->load_res;
	tile->pix_height = TILE_HEIGHT / tile->load_res;
	pixels = malloc(tile->pix_width * tile->pix_height * sizeof (*pixels));

	dbg_log(tile, 3, "render tile %d x %d (%d x %d)",
	    tile->lat, tile->lon, tile->pix_width, tile->pix_height);

	for (unsigned y = 0; y < tile->pix_height; y++) {
		for (unsigned x = 0; x < tile->pix_width; x++) {
			int dsf_x = clampi(round(x * dsf_scale_x),
			    0, demi->demi_atom.width - 1);
			int dsf_y = clampi(round(y * dsf_scale_y),
			    0, demi->demi_atom.height - 1);
			int elev = dsf_pixels[dsf_y * dsf_width + dsf_x] *
			    demi->demi_atom.scale + demi->demi_atom.offset;

			pixels[y * tile->pix_width + x] =
			    clampi(elev, INT16_MIN, INT16_MAX);
		}
	}
	mutex_enter(&dem_tile_cache_lock);
	free(tile->pixels);
	tile->pixels = pixels;
	mutex_exit(&dem_tile_cache_lock);
}

static int
load_dem_tile(int lat, int lon, double load_res)
{
	dem_tile_t srch = { .lat = lat, .lon = lon };
	dem_tile_t *tile;
	bool_t existing;
	dsf_t *dsf = NULL;
	const dsf_atom_t *demi, *demd;

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
		tile = calloc(1, sizeof (*tile));
		tile->lat = lat;
		tile->lon = lon;
		tile->cur_tex = -1;
	}
	tile->load_res = load_res;

	dsf = load_dem_dsf(lat, lon, &demi, &demd);

	dbg_log(tile, 2, "load tile %d x %d (dsf: %p)", lat, lon, dsf);

	if (dsf != NULL) {
		ASSERT(!tile->empty);
		render_dem_tile(tile, demi, demd);
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

	return (tile->pix_width * tile->pix_height * 2);
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
			return (rngs[i].resolution);
		}
	}

	dbg_log(tile, 2, "tile res fallback %d x %d = min", tile_lat, tile_lon);

	return (MIN_RES);
}

static void
load_nrst_dem_tiles(geo_pos3_t pos)
{
	int tile_width;
	int h_tiles, v_tiles;
	const egpws_range_t *rngs = ranges;
	double rng = MIN_RNG;
	fpp_t fpp = stereo_fpp_init(GEO3_TO_GEO2(pos), 0, &wgs84, B_FALSE);
	int n_tiles = 0, bytes;

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
	free(tile->pixels);
	free(tile);
}

static void
append_cust_srch_paths(void)
{
	char *contents;
	char **lines;
	size_t n_lines;

	contents = file2str(xpdir, "Custom Scenery", "scenery_packs.ini", NULL);

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
		subpath = mkpathname(xpdir, buf, "Earth nav data", NULL);

		if (file_exists(subpath, &is_dir) && is_dir) {
			srch_path_t *path = calloc(1, sizeof (*path));

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
	char *path = mkpathname(xpdir, "Global Scenery", NULL);
	DIR *dp = opendir(path);
	struct dirent *de;

	if (dp == NULL)
		goto errout;

	while ((de = readdir(dp)) != NULL) {
		char *subpath = mkpathname(path, de->d_name, "Earth nav data",
		    NULL);
		bool_t is_dir;

		if (file_exists(subpath, &is_dir) && is_dir) {
			srch_path_t *path = calloc(1, sizeof (*path));
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
		load_nrst_dem_tiles(pos);

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
terr_init(const char *the_xpdir, const char *plugindir)
{
	char *vtx_prog_path, *frag_prog_path;

	ASSERT(!inited);
	inited = B_TRUE;

	xpdir = strdup(the_xpdir);

	mutex_init(&glob_pos_lock);

	mutex_init(&dem_tile_cache_lock);
	avl_create(&dem_tile_cache, dem_tile_compar, sizeof (dem_tile_t),
	    offsetof(dem_tile_t, cache_node));
	avl_create(&tnlc, tnlc_compar, sizeof (tnlc_ent_t),
	    offsetof(tnlc_ent_t, tnlc_node));

	list_create(&srch_paths, sizeof (srch_path_t),
	    offsetof(srch_path_t, node));

	worker_init(&load_dem_worker_wk, load_dem_worker,
	    SEC2USEC(LOAD_DEM_WORKER_INTVAL), NULL);

	vtx_prog_path = mkpathname(xpdir, plugindir, "data", "DEM_vtx.glsl",
	    NULL);
	frag_prog_path = mkpathname(xpdir, plugindir, "data", "DEM_frag.glsl",
	    NULL);
	DEM_prog = shader_prog_from_file("DEM_shader", vtx_prog_path,
	    frag_prog_path);
	lacf_free(vtx_prog_path);
	lacf_free(frag_prog_path);
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

	free(xpdir);
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
		elev = tile->pixels[y * tile->pix_width + x];
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

	for (int xi = -1; xi <= 1; xi++) {
		for (int yi = -1; yi <= 1; yi++) {
			int xx = clampi(x + xi, 0, tile->pix_width - 1);
			int yy = clampi(y + yi, 0, tile->pix_height - 1);
			double e = tile->pixels[yy * tile->pix_width + xx];

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
			XPLMBindTexture2d(tile->tex[next_tex], GL_TEXTURE_2D);
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
			XPLMBindTexture2d(tile->tex[next_tex], GL_TEXTURE_2D);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RG, tile->pix_width,
			    tile->pix_height, 0, GL_RG, GL_UNSIGNED_BYTE, NULL);

			tile->dirty = B_FALSE;
			tile->in_flight = 0;
			tile->cur_tex = next_tex;
		}

		glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	}

	mutex_exit(&dem_tile_cache_lock);
}

static void
draw_tiles(const egpws_render_t *render)
{
	fpp_t fpp = ortho_fpp_init(GEO3_TO_GEO2(render->position),
	    render->rotation, &wgs84, B_FALSE);
	const egpws_conf_t *conf = egpws_get_conf();

	XPLMSetGraphicsState(0, 1, 0, 1, 1, 1, 1);

	mutex_enter(&dem_tile_cache_lock);
	for (dem_tile_t *tile = avl_first(&dem_tile_cache);
	    tile != NULL; tile = AVL_NEXT(&dem_tile_cache, tile)) {
		vect2_t v[4];
		vect2_t t[4] = {
		    VECT2(0, 0), VECT2(0, 1), VECT2(1, 1), VECT2(1, 0)
		};
		GLfloat hgt_rngs_ft[4 * 2];
		GLfloat hgt_colors[4 * 4];

		if (tile->cur_tex == -1)
			continue;

		v[0] = geo2fpp(GEO_POS2(tile->lat, tile->lon), &fpp);
		v[1] = geo2fpp(GEO_POS2(tile->lat + 1, tile->lon), &fpp);
		v[2] = geo2fpp(GEO_POS2(tile->lat + 1, tile->lon + 1), &fpp);
		v[3] = geo2fpp(GEO_POS2(tile->lat, tile->lon + 1), &fpp);

		for (int i = 0; i < 4; i++) {
			v[i] = vect2_add(vect2_scmul(v[i], render->scale),
			    render->offset);
		}

		glActiveTexture(GL_TEXTURE0);
		XPLMBindTexture2d(tile->tex[tile->cur_tex], GL_TEXTURE_2D);

		glUseProgram(DEM_prog);

		glUniform1i(glGetUniformLocation(DEM_prog, "tex"), 0);
		glUniform1f(glGetUniformLocation(DEM_prog, "acf_elev_ft"),
		    MET2FEET(glob_pos.elev));

		for (int i = 0; i < 4; i++) {
			const egpws_terr_color_t *color = &conf->terr_colors[i];
			hgt_rngs_ft[i * 2] = MET2FEET(color->min_hgt);
			hgt_rngs_ft[i * 2 + 1] = MET2FEET(color->max_hgt);
			for (int j = 0; j < 4; j++)
				hgt_colors[i * 4 + j] = color->rgba[j];
		}
		glUniform2fv(glGetUniformLocation(DEM_prog, "hgt_rngs_ft"),
		    4 * 2, hgt_rngs_ft);
		glUniform4fv(glGetUniformLocation(DEM_prog, "hgt_colors"),
		    4 * 4, hgt_colors);

		glBegin(GL_QUADS);
		for (int i = 0; i < 4; i++) {
			glTexCoord2f(t[i].x, t[i].y);
			glVertex2f(v[i].x, v[i].y);
		}
		glEnd();
	}
	mutex_exit(&dem_tile_cache_lock);

	glUseProgram(0);
}

void
terr_render(const egpws_render_t *render)
{
	ASSERT(inited);

	update_tiles();

	if (render->do_draw)
		draw_tiles(render);
}
