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

#include <XPLMGraphics.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dsf.h>
#include <acfutils/helpers.h>
#include <acfutils/list.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include <GL/glew.h>

#include "dbg_log.h"
#include "egpws.h"
#include "terr.h"

#define	LOAD_WORKER_INTVAL	1			/* seconds */
#define	PAINT_WORKER_INTVAL	0.5			/* seconds */
#define	EARTH_CIRC		(2 * EARTH_MSL * M_PI)	/* meters */
#define	TILE_HEIGHT		(EARTH_CIRC / 360.0)	/* meters */
#define	MAX_LAT			80			/* degrees */
#define	MIN_RES			108000			/* meters */
#define	MAX_RES			108			/* meters */
#define	MIN_RNG			5000			/* meters */
#define	DFL_RNG			50000			/* meters */
#define	MAX_RNG			600000			/* meters */
#define	ALT_QUANT_STEP		FEET2MET(100)		/* quantization step */

typedef struct {
	int		lat;		/* latitude in degrees */
	int		lon;		/* longitude in degrees */
	double		min_elev;	/* minimum elevation (meters) */
	double		max_elev;	/* maximum elevation (meters) */
	double		load_res;	/* resolution at load time */
	unsigned	pix_width;	/* tile width in pixels */
	unsigned	pix_height;	/* tile height in pixels */
	int16_t		*pixels;	/* elevation data samples */
	bool_t		dirty;		/* mark for removal */
	bool_t		empty;		/* tile contains no data */

	mutex_t		tex_lock;
	GLuint		tex;

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

static bool_t load_worker_shutdown = B_TRUE;
static mutex_t load_worker_lock;
static condvar_t load_worker_cv;
static thread_t load_worker_thread;

static bool_t paint_worker_shutdown = B_TRUE;
static mutex_t paint_worker_lock;
static condvar_t paint_worker_cv;
static thread_t paint_worker_thread;

static mutex_t dem_tile_cache_lock;
static avl_tree_t dem_tile_cache;

static egpws_terr_tile_set_t terr_tile_set;

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
terr_tile_compar(const void *a, const void *b)
{
	const egpws_terr_tile_t *ta = a, *tb = b;

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

static void
load_tile(int lat, int lon, double load_res)
{
	avl_index_t where;
	dem_tile_t srch = { .lat = lat, .lon = lon };
	dem_tile_t *tile;
	bool_t existing;

	tile = avl_find(&dem_tile_cache, &srch, &where);
	existing = (tile != NULL);
	if (tile == NULL || tile->load_res != load_res) {
		const dsf_atom_t *demi, *demd;
		dsf_t *dsf = NULL;

		/* Don't reload empty tiles, it will never succeed. */
		dsf = load_dem_dsf(lat, lon, &demi, &demd);

		dbg_log(tile, 2, "load tile %d x %d (dsf: %p)", lat, lon, dsf);
		if (tile == NULL) {
			tile = calloc(1, sizeof (*tile));
			tile->lat = lat;
			tile->lon = lon;
		}
		tile->load_res = load_res;
		if (dsf != NULL) {
			render_dem_tile(tile, demi, demd);
			tile->empty = B_FALSE;
		} else {
			tile->empty = B_TRUE;
		}

		if (!existing) {
			mutex_enter(&dem_tile_cache_lock);
			avl_insert(&dem_tile_cache, tile, where);
			mutex_exit(&dem_tile_cache_lock);
		} else {
			dbg_log(tile, 2, "tile res chg %f -> %f",
			    tile->load_res, load_res);
		}

		if (dsf != NULL)
			dsf_fini(dsf);
	}

	tile->dirty = B_FALSE;
}

static double
select_tile_res(geo_pos3_t pos, int tile_lat, int tile_lon)
{
	int lat = floor(pos.lat);
	int lon = floor(pos.lon);
	vect3_t tile_ecef, my_ecef;
	double dist;
	const egpws_range_t *rngs = ranges;

	/*
	 * Tiles within 1 degree of our position are always loaded at
	 * maximum resolution to. The terrain avoidance algorithm needs
	 * this precision. The remaining tiles are for display only, so
	 * a lower resolution is safe to use.
	 */
	if (ABS(tile_lat - lat) <= 1 && ABS(tile_lon - lon) <= 1) {
		dbg_log(tile, 2, "tile res +-1 %d x %d = max", tile_lat,
		    tile_lon);
		return (MAX_RES);
	}

	tile_ecef = geo2ecef(GEO_POS3(tile_lat + 0.5, tile_lon + 0.5, 0),
	    &wgs84);
	my_ecef = geo2ecef(GEO_POS3(pos.lat, pos.lon, 0), &wgs84);
	dist = vect3_abs(vect3_sub(tile_ecef, my_ecef));
	for (int i = 0; !isnan(rngs[i].range); i++) {
		if (dist < rngs[i].range) {
			dbg_log(tile, 2, "tile res %d x %d = %.0f",
			    tile_lat, tile_lon, rngs[i].resolution);
			return (rngs[i].resolution);
		}
	}

	dbg_log(tile, 2, "tile res fallback %d x %d = max", tile_lat, tile_lon);

	return (MAX_RES);
}

static void
load_nrst_tiles(geo_pos3_t pos)
{
	int tile_width;
	int h_tiles, v_tiles;
	const egpws_range_t *rngs = ranges;
	double rng = MIN_RNG;

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

			if (load_worker_shutdown)
				return;

			res = select_tile_res(pos, tile_lat, tile_lon);
			load_tile(tile_lat, tile_lon, res);
		}
	}
}

static void
free_dem_tile(dem_tile_t *tile)
{
	dbg_log(tile, 2, "free tile %d x %d", tile->lat, tile->lon);
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

static void
load_worker(void *unused)
{
	UNUSED(unused);

	append_cust_srch_paths();
	append_glob_srch_paths();

	mutex_enter(&load_worker_lock);
	while (!load_worker_shutdown) {
		geo_pos3_t pos;

		dbg_log(tile, 4, "loop");

		mutex_enter(&glob_pos_lock);
		pos = glob_pos;
		mutex_exit(&glob_pos_lock);

		/*
		 * Mark all tiles as dirty. The load process will undirty
		 * the ones we want to keep.
		 */
		for (dem_tile_t *tile = avl_first(&dem_tile_cache);
		    tile != NULL; tile = AVL_NEXT(&dem_tile_cache, tile))
			tile->dirty = B_TRUE;

		if (!IS_NULL_GEO_POS(pos))
			load_nrst_tiles(pos);

		/* Unload dirty tiles */
		mutex_enter(&dem_tile_cache_lock);
		for (dem_tile_t *tile = avl_first(&dem_tile_cache),
		    *next = NULL; tile != NULL; tile = next) {
			next = AVL_NEXT(&dem_tile_cache, tile);
			if (tile->dirty) {
				avl_remove(&dem_tile_cache, tile);
				free_dem_tile(tile);
			}
		}
		mutex_exit(&dem_tile_cache_lock);

		cv_timedwait(&load_worker_cv, &load_worker_lock,
		    microclock() + SEC2USEC(LOAD_WORKER_INTVAL));
	}
	mutex_exit(&load_worker_lock);

	dbg_log(tile, 4, "load worker shutdown");
}

static void
render_terr_tile(geo_pos3_t pos, egpws_terr_tile_t *tile, dem_tile_t *dt)
{
	unsigned dt_width = dt->pix_width, dt_height = dt->pix_height;
	const egpws_conf_t *conf = egpws_get_conf();

	ASSERT(!dt->empty);

	if (tile->pix_width != dt_width || tile->pix_height != dt_height) {
		free(tile->pixels);
		tile->pix_width = dt_width;
		tile->pix_height = dt_height;
		tile->pixels = calloc(tile->pix_width * tile->pix_height,
		    sizeof (*tile->pixels));
	}

	/* Preset the pixels to black */
	for (unsigned j = 0; j < dt_width * dt_height; j++)
		tile->pixels[j] = BE32(0xff);

	for (int i = 0; conf->terr_colors[i].max_hgt >
	    conf->terr_colors[i].min_hgt; i++) {
		const egpws_terr_color_t *color = &conf->terr_colors[i];

		for (unsigned j = 0; j < dt_width * dt_height; j++) {
			int hgt = pos.elev - dt->pixels[j];
			if (hgt >= color->min_hgt && hgt < color->max_hgt)
				tile->pixels[j] = color->color_rgba;
		}
	}
}

static void
paint_worker(void *unused)
{
	geo_pos3_t last_pos;

	UNUSED(unused);

	mutex_enter(&paint_worker_lock);
	while (!paint_worker_shutdown) {
		geo_pos3_t pos;

		dbg_log(painter, 2, "loop");

		mutex_enter(&glob_pos_lock);
		pos = glob_pos;
		mutex_exit(&glob_pos_lock);

		mutex_enter(&dem_tile_cache_lock);
		mutex_enter(&terr_tile_set.lock);

		for (egpws_terr_tile_t *tile = avl_first(&terr_tile_set.tiles),
		    *next_tile = NULL; tile != NULL; tile = next_tile) {
			dem_tile_t srch = {.lat = tile->lat, .lon = tile->lon};
			dem_tile_t *dt;

			next_tile = AVL_NEXT(&terr_tile_set.tiles, tile);

			if (tile->remove)
				continue;
			dt = avl_find(&dem_tile_cache, &srch, NULL);
			if (dt == NULL) {
				/* Tile no longer in cache, mark for removal */
				dbg_log(painter, 1, "remove tile %d x %d",
				    tile->lat, tile->lon);
				free(tile->pixels);
				tile->pixels = NULL;
				tile->remove = B_TRUE;
				continue;
			}
			if ((tile->pix_width != dt->pix_width ||
			    tile->pix_height != dt->pix_height ||
			    pos.elev != last_pos.elev) && !dt->empty) {
				/*
				 * Since only we can modify the private part
				 * of a terrain tile, we can exit the lock to
				 * let avionics retrieve the old tile set for
				 * their rendering loops. The main thread
				 * never touches the pixel array.
				 */
				mutex_exit(&terr_tile_set.lock);
				render_terr_tile(pos, tile, dt);
				mutex_enter(&terr_tile_set.lock);
				/* Tell main thread to re-upload the texture */
				tile->dirty = B_TRUE;
			}
		}

		/* Add any new DEM tiles */
		for (dem_tile_t *dt = avl_first(&dem_tile_cache); dt != NULL;
		    dt = AVL_NEXT(&dem_tile_cache, dt)) {
			egpws_terr_tile_t srch = {
			    .lat = dt->lat, .lon = dt->lon
			};
			egpws_terr_tile_t *tile;
			avl_index_t where;

			tile = avl_find(&terr_tile_set.tiles, &srch, &where);
			if (tile != NULL)
				/* Tile already rendered, move on */
				continue;
			mutex_exit(&terr_tile_set.lock);

			dbg_log(painter, 1, "new tile %d x %d",
			    dt->lat, dt->lon);
			tile = calloc(1, sizeof (*tile));
			tile->lat = dt->lat;
			tile->lon = dt->lon;
			if (!dt->empty) {
				tile->dirty = B_TRUE;
				render_terr_tile(pos, tile, dt);
			}

			mutex_enter(&terr_tile_set.lock);
			avl_insert(&terr_tile_set.tiles, tile, where);
		}

		mutex_exit(&terr_tile_set.lock);
		mutex_exit(&dem_tile_cache_lock);

		last_pos = pos;
		cv_timedwait(&paint_worker_cv, &paint_worker_lock,
		    microclock() + SEC2USEC(PAINT_WORKER_INTVAL));
	}

	mutex_exit(&paint_worker_lock);
}

void
terr_init(const char *the_xpdir)
{
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

	mutex_init(&terr_tile_set.lock);
	avl_create(&terr_tile_set.tiles, terr_tile_compar,
	    sizeof (egpws_terr_tile_t), offsetof(egpws_terr_tile_t, node));

	load_worker_shutdown = B_FALSE;
	mutex_init(&load_worker_lock);
	cv_init(&load_worker_cv);
	VERIFY(thread_create(&load_worker_thread, load_worker, NULL));

	paint_worker_shutdown = B_FALSE;
	mutex_init(&paint_worker_lock);
	cv_init(&paint_worker_cv);
	VERIFY(thread_create(&paint_worker_thread, paint_worker, NULL));
}

void
terr_fini(void)
{
	srch_path_t *path;
	dem_tile_t *tile;
	void *cookie;
	tnlc_ent_t *te;
	egpws_terr_tile_t *terr_tile;

	if (!inited)
		return;
	inited = B_FALSE;

	/* Kill the tile loader */
	load_worker_shutdown = B_TRUE;
	mutex_enter(&load_worker_lock);
	cv_broadcast(&load_worker_cv);
	mutex_exit(&load_worker_lock);
	thread_join(&load_worker_thread);

	mutex_destroy(&load_worker_lock);
	cv_destroy(&load_worker_cv);

	/* Kill the tile painter */
	paint_worker_shutdown = B_TRUE;
	mutex_enter(&paint_worker_lock);
	cv_broadcast(&paint_worker_cv);
	mutex_exit(&paint_worker_lock);
	thread_join(&paint_worker_thread);

	mutex_destroy(&paint_worker_lock);
	cv_destroy(&paint_worker_cv);

	cookie = NULL;
	while ((terr_tile = avl_destroy_nodes(&terr_tile_set.tiles, &cookie)) !=
	    NULL) {
		if (terr_tile->tex != 0)
			glDeleteTextures(1, &terr_tile->tex);
		free(terr_tile->pixels);
		free(terr_tile);
	}
	mutex_destroy(&terr_tile_set.lock);
	avl_destroy(&terr_tile_set.tiles);

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
	while ((tile = avl_destroy_nodes(&dem_tile_cache, &cookie)) != NULL) {
		free(tile->pixels);
		free(tile);
	}
	avl_destroy(&dem_tile_cache);
	mutex_destroy(&dem_tile_cache_lock);

	cookie = NULL;
	while ((te = avl_destroy_nodes(&tnlc, &cookie)) != NULL) {
		free(te->path);
		free(te);
	}
	avl_destroy(&tnlc);

	mutex_destroy(&glob_pos_lock);

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
terr_get_elev_wide(geo_pos2_t pos)
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

		for (int xi = -1; xi <= 1; xi++) {
			for (int yi = -1; yi <= 1; yi++) {
				int xx = clampi(x + xi, 0,
				    tile->pix_width - 1);
				int yy = clampi(y + yi, 0,
				    tile->pix_height - 1);
				double e =
				    tile->pixels[yy * tile->pix_width + xx];

				if (isnan(elev))
					elev = e;
				else if (e > elev)
					elev = e;
			}
		}
		
	}
	mutex_exit(&dem_tile_cache_lock);

	dbg_log(terr, 3, "get elev (%.4fx%.4f) = %.0f\n",
	    pos.lat, pos.lon, elev);

	return (elev);
}

egpws_terr_tile_set_t *
terr_get_tile_set(void)
{
	/* On the main thread we can do all of our OpenGL calls */
	mutex_enter(&terr_tile_set.lock);
	for (egpws_terr_tile_t *tile = avl_first(&terr_tile_set.tiles),
	    *next_tile = NULL; tile != NULL; tile = next_tile) {
		next_tile = AVL_NEXT(&terr_tile_set.tiles, tile);

		if (tile->remove) {
			dbg_log(painter, 1, "free tile %d x %d",
			    tile->lat, tile->lon);
			avl_remove(&terr_tile_set.tiles, tile);
			if (tile->tex != 0)
				glDeleteTextures(1, &tile->tex);
			free(tile);
			continue;
		}
		if (tile->dirty) {
			dbg_log(painter, 2, "upload tile %d x %d  "
			    "(%dpx x %dpx)", tile->lat, tile->lon,
			    tile->pix_width, tile->pix_height);
			if (tile->tex == 0) {
				dbg_log(painter, 2, "tex alloc tile %d x %d",
				    tile->lat, tile->lon);
				glGenTextures(1, &tile->tex);
				XPLMBindTexture2d(tile->tex, GL_TEXTURE_2D);
				glTexParameteri(GL_TEXTURE_2D,
				    GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D,
				    GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			}
			ASSERT(tile->pixels != NULL);
			XPLMBindTexture2d(tile->tex, GL_TEXTURE_2D);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tile->pix_width,
			    tile->pix_height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
			    tile->pixels);
			tile->dirty = B_FALSE;
		}
	}
	mutex_exit(&terr_tile_set.lock);

	return (&terr_tile_set);
}
