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

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dsf.h>
#include <acfutils/helpers.h>
#include <acfutils/list.h>
#include <acfutils/math.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include <GL/glew.h>

#include "dbg_log.h"
#include "terr.h"

#define	LOAD_WORKER_INTVAL	1			/* seconds */
#define	EARTH_CIRC		(2 * EARTH_MSL * M_PI)	/* meters */
#define	TILE_HEIGHT		(EARTH_CIRC / 360.0)	/* meters */
#define	MAX_LAT			80			/* degrees */
#define	MIN_RES			108000			/* meters */
#define	MAX_RES			108			/* meters */
#define	MIN_RNG			5000			/* meters */
#define	DFL_RNG			50000			/* meters */
#define	MAX_RNG			600000			/* meters */

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

static geo_pos2_t pos = NULL_GEO_POS2;

static bool_t load_worker_shutdown = B_TRUE;
static mutex_t load_worker_lock;
static condvar_t load_worker_cv;
static thread_t load_worker_thread;

static mutex_t tile_cache_lock;
static avl_tree_t tile_cache;

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
tile_compar(const void *a, const void *b)
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

	snprintf(dname, sizeof (dname), "%+03.0f%04.0f", 
	    floor(lat / 10.0) * 10.0, floor(lon / 10.0) * 10.0);
	snprintf(fname, sizeof (fname), "%+03d%04d.dsf", lat, lon);

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
render_tile(dem_tile_t *tile, const dsf_atom_t *demi,
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
	mutex_enter(&tile_cache_lock);
	free(tile->pixels);
	tile->pixels = pixels;
	mutex_exit(&tile_cache_lock);
}

static void
load_tile(int lat, int lon, double load_res)
{
	avl_index_t where;
	dem_tile_t srch = { .lat = lat, .lon = lon };
	dem_tile_t *tile;
	bool_t existing;

	tile = avl_find(&tile_cache, &srch, &where);
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
			render_tile(tile, demi, demd);
			tile->empty = B_FALSE;
		} else {
			tile->empty = B_TRUE;
		}

		if (!existing) {
			mutex_enter(&tile_cache_lock);
			avl_insert(&tile_cache, tile, where);
			mutex_exit(&tile_cache_lock);
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
select_tile_res(int tile_lat, int tile_lon)
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
load_nrst_tiles(void)
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

			res = select_tile_res(tile_lat, tile_lon);
			load_tile(tile_lat, tile_lon, res);
		}
	}
}

static void
free_tile(dem_tile_t *tile)
{
	dbg_log(tile, 2, "free tile %d x %d", tile->lat, tile->lon);
	free(tile->pixels);
	free(tile);
}

static void
load_worker(void *unused)
{
	UNUSED(unused);

	mutex_enter(&load_worker_lock);
	while (!load_worker_shutdown) {
		dbg_log(tile, 4, "loop");

		/*
		 * Mark all tiles as dirty. The load process will undirty
		 * the ones we want to keep.
		 */
		for (dem_tile_t *tile = avl_first(&tile_cache); tile != NULL;
		    tile = AVL_NEXT(&tile_cache, tile)) {
			tile->dirty = B_TRUE;
		}

		if (!IS_NULL_GEO_POS(pos))
			load_nrst_tiles();

		/* Unload dirty tiles */
		mutex_enter(&tile_cache_lock);
		for (dem_tile_t *tile = avl_first(&tile_cache), *next = NULL;
		    tile != NULL; tile = next) {
			next = AVL_NEXT(&tile_cache, tile);
			if (tile->dirty) {
				avl_remove(&tile_cache, tile);
				free_tile(tile);
			}
		}
		mutex_exit(&tile_cache_lock);

		cv_timedwait(&load_worker_cv, &load_worker_lock,
		    microclock() + SEC2USEC(LOAD_WORKER_INTVAL));
	}
	mutex_exit(&load_worker_lock);

	dbg_log(tile, 4, "load worker shutdown");
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

void
terr_init(const char *the_xpdir)
{
	ASSERT(!inited);
	inited = B_TRUE;

	xpdir = strdup(the_xpdir);

	mutex_init(&tile_cache_lock);
	avl_create(&tile_cache, tile_compar, sizeof (dem_tile_t),
	    offsetof(dem_tile_t, cache_node));
	avl_create(&tnlc, tnlc_compar, sizeof (tnlc_ent_t),
	    offsetof(tnlc_ent_t, tnlc_node));

	list_create(&srch_paths, sizeof (srch_path_t),
	    offsetof(srch_path_t, node));
	append_cust_srch_paths();
	append_glob_srch_paths();

	load_worker_shutdown = B_FALSE;
	mutex_init(&load_worker_lock);
	cv_init(&load_worker_cv);
	VERIFY(thread_create(&load_worker_thread, load_worker, NULL));
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

	load_worker_shutdown = B_TRUE;
	mutex_enter(&load_worker_lock);
	cv_broadcast(&load_worker_cv);
	mutex_exit(&load_worker_lock);
	thread_join(&load_worker_thread);

	mutex_destroy(&load_worker_lock);
	cv_destroy(&load_worker_cv);

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
	while ((tile = avl_destroy_nodes(&tile_cache, &cookie)) != NULL) {
		free(tile->pixels);
		free(tile);
	}
	avl_destroy(&tile_cache);
	mutex_destroy(&tile_cache_lock);

	cookie = NULL;
	while ((te = avl_destroy_nodes(&tnlc, &cookie)) != NULL) {
		free(te->path);
		free(te);
	}
	avl_destroy(&tnlc);

	free(xpdir);
}

void
terr_set_pos(geo_pos2_t new_pos)
{
	if (ABS(new_pos.lat) >= MAX_LAT)
		pos = NULL_GEO_POS2;
	else
		pos = new_pos;

	dbg_log(terr, 2, "set pos %.4f x %.4f", pos.lat, pos.lon);
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

	mutex_enter(&tile_cache_lock);
	tile = avl_find(&tile_cache, &srch, NULL);
	if (tile != NULL && !tile->empty) {
		int x = clampi((pos.lon - tile->lon) * tile->pix_width,
		    0, tile->pix_width - 1);
		int y = clampi((pos.lat - tile->lat) * tile->pix_height,
		    0, tile->pix_height - 1);
		elev = tile->pixels[y * tile->pix_width + x];
	}
	mutex_exit(&tile_cache_lock);

	dbg_log(terr, 3, "get elev (%.4fx%.4f) = %.0f\n", pos.lat, pos.lon,
	    elev);

	return (elev);
}
