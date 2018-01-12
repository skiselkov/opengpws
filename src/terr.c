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

#include "terr.h"

#define	LOAD_WORKER_INTVAL	0.3			/* seconds */
#define	EARTH_CIRC		(2 * EARTH_MSL * M_PI)	/* meters */
#define	TILE_HEIGHT		(EARTH_CIRC / 360.0)	/* meters */
#define	MAX_LAT			80			/* degrees */
#define	MIN_RES			100000			/* meters */
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
	bool_t		dirty;

	mutex_t		tex_lock;
	GLuint		tex;

	avl_node_t	cache_node;
} dem_tile_t;

typedef struct {
	char		*path;
	list_node_t	node;
} srch_path_t;

static bool_t inited = B_FALSE;
static list_t srch_paths;

static char *dsf_paths[180][360];
static char *xpdir = NULL;

static geo_pos2_t pos = NULL_GEO_POS2;
static double range = DFL_RNG;		/* meters */
static double resolution = MAX_RES;	/* meters per pixel */

static bool_t load_worker_shutdown = B_TRUE;
static mutex_t load_worker_lock;
static condvar_t load_worker_cv;
static thread_t load_worker_thread;

static mutex_t tile_cache_lock;
static avl_tree_t tile_cache;

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

static dsf_t *
load_dem_dsf(int lat, int lon, const dsf_atom_t **demi_p,
    const dsf_atom_t **demd_p)
{
	char dname[16];
	char fname[16];

	snprintf(dname, sizeof (dname), "%+03.0f%04.0f", 
	    floor(lat / 10.0) * 10.0, floor(lon / 10.0) * 10.0);
	snprintf(fname, sizeof (fname), "%+03d%04d.dsf", lat, lon);

	for (srch_path_t *sp = list_head(&srch_paths); sp != NULL;
	    sp = list_next(&srch_paths, sp)) {
		char *path = mkpathname(xpdir, sp->path, dname, fname, NULL);
		dsf_t *dsf = dsf_init(path);

		if (dsf != NULL && (*demi_p = dsf_lookup(dsf, DSF_ATOM_DEMS, 0,
		    DSF_ATOM_DEMI, 0, 0)) != NULL && (*demd_p = dsf_lookup(dsf,
		    DSF_ATOM_DEMS, 0, DSF_ATOM_DEMD, 0, 0)) != NULL) {
			free(path);
			return (dsf);
		}
		free(path);
	}

	return (NULL);
}

static void
render_tile(dem_tile_t *tile, const dsf_atom_t *demi,
    const dsf_atom_t *demd)
{
	double tile_width = EARTH_CIRC * cos(DEG2RAD(tile->lat));
	unsigned dsf_width = demi->demi_atom.width;
	unsigned dsf_height = demi->demi_atom.height;
	double dsf_res_x = tile_width / dsf_width;
	double dsf_res_y = TILE_HEIGHT / dsf_height;
	double dsf_scale_x = tile->load_res / dsf_res_x;
	double dsf_scale_y = tile->load_res / dsf_res_y;
	const uint16_t *dsf_pixels = (const uint16_t *)demd->payload;
	int16_t *pixels;

	tile->pix_width = tile_width * tile->load_res;
	tile->pix_height = TILE_HEIGHT * tile->load_res;
	pixels = malloc(tile->pix_width * tile->pix_height * sizeof (*pixels));

	for (unsigned y = 0; y < tile->pix_height; y++) {
		for (unsigned x = 0; x < tile->pix_width; x++) {
			int dsf_x = clampi(round(x * dsf_scale_x),
			    0, demi->demi_atom.width - 1);
			int dsf_y = clampi(round(y * dsf_scale_y),
			    0, demi->demi_atom.height - 1);
			int elev = dsf_pixels[dsf_y * dsf_width + dsf_x] *
			    demi->demi_atom.scale + demi->demi_atom.offset;

			pixels[y * tile->pix_width + tile->pix_height] =
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

	tile = avl_find(&tile_cache, &srch, &where);
	if (tile == NULL || tile->load_res != load_res) {
		const dsf_atom_t *demi, *demd;
		dsf_t *dsf = load_dem_dsf(lat, lon, &demi, &demd);

		if (dsf == NULL)
			return;
		if (tile == NULL) {
			tile = calloc(1, sizeof (*tile));
			tile->lat = lat;
			tile->lon = lon;
		}
		tile->load_res = load_res;
		tile->dirty = B_FALSE;
		render_tile(tile, demi, demd);

		dsf_fini(dsf);
	}
}

static void
load_nrst_tiles(void)
{
	int tile_width;
	int h_tiles, v_tiles;

	ASSERT3F(ABS(pos.lat), <=, MAX_LAT);
	tile_width = EARTH_CIRC * cos(DEG2RAD(pos.lat));
	h_tiles = MAX(range / tile_width, 1);
	v_tiles = MAX(range / TILE_HEIGHT, 1);

	for (int v = -v_tiles; v <= v_tiles; v++) {
		for (int h = -h_tiles; h <= h_tiles; h++) {
			double res;

			if (load_worker_shutdown)
				return;
			if (v >= -1 && v <= 1 && h >= -1 && h <= 1)
				res = MAX_RES;
			else
				res = resolution;
			load_tile(floor(pos.lat + v), floor(pos.lon + h), res);
		}
	}
}

static void
free_tile(dem_tile_t *tile)
{
	free(tile->pixels);
	free(tile);
}

static void
load_worker(void *unused)
{
	UNUSED(unused);

	mutex_enter(&load_worker_lock);
	while (!load_worker_shutdown) {
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

	free(xpdir);
}

void
terr_set_pos(geo_pos2_t new_pos)
{
	if (ABS(new_pos.lat) >= MAX_LAT)
		pos = NULL_GEO_POS2;
	else
		pos = new_pos;
}

void
terr_set_range(double rng, double res)
{
	range = clamp(rng, MIN_RNG, MAX_RNG);
	resolution = clamp(res, MAX_RES, MIN_RES);
}

double
terr_get_elev(geo_pos2_t pos)
{
	double elev = NAN;
	dem_tile_t srch = { .lat = floor(pos.lat), .lon = floor(pos.lon) };
	dem_tile_t *tile;

	mutex_enter(&tile_cache_lock);
	tile = avl_find(&tile_cache, &srch, NULL);
	if (tile != NULL) {
		int x = clampi((pos.lon - tile->lon) * tile->pix_width,
		    0, tile->pix_width - 1);
		int y = clampi((pos.lat - tile->lat) * tile->pix_height,
		    0, tile->pix_height - 1);
		elev = tile->pixels[y * tile->pix_width + x];
	}
	mutex_exit(&tile_cache_lock);

	return (elev);
}
