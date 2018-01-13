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

#include <string.h>

#include <acfutils/airportdb.h>
#include <acfutils/assert.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "dbg_log.h"
#include "egpws.h"
#include "snd_sys.h"
#include "terr.h"
#include "xplane.h"
#include <opengpws/xplane_api.h>

#define	RUN_INTVAL		SEC2USEC(0.5)
#define	INF_VS			1e10
#define	RA_LIMIT		FEET2MET(2500)
#define	TAWSB_ARPT_500_DIST_LIM	NM2MET(5)

static bool_t		inited = B_FALSE;
static bool_t		shutdown = B_FALSE;
static mutex_t		lock;
static condvar_t	cv;
static thread_t		worker;
static airportdb_t	db;		/* read-only once inited */
static bool_t		init_error = B_FALSE;

static egpws_conf_t	conf;		/* read-only once inited */

static struct {
	mutex_t			lock;
	egpws_arpt_ref_t	dest;		/* protected by lock */
	egpws_pos_t		pos;		/* protected by lock */
	bool_t			flaps_ovrd;	/* atomic */
} glob_data;

static struct {
	struct {
		struct {
			double last_caut_vs;
		} mode1;
	} mk8;
	struct {
		struct {
			bool_t lo_caut;
			bool_t hi_caut;
		} edr;
		bool_t ann_500ft;
	} tawsb;
} state;

static void
egpws_boot(void)
{
	const char *plugindir = get_plugindir();
	char *cachedir = mkpathname(plugindir, "airport.cache", NULL);

	airportdb_create(&db, get_xpdir(), cachedir);
	init_error = !recreate_cache(&db);
	free(cachedir);
}

static void
egpws_shutdown(void)
{
	airportdb_destroy(&db);
}

static void
mk8_mode1(const egpws_pos_t *pos)
{
	static const vect2_t caut_rng_tp[] = {
		VECT2(0,		FPM2MPS(-1031)),
		VECT2(FEET2MET(2450),	FPM2MPS(-5007)),
		VECT2(FEET2MET(2500),	-INF_VS),
		NULL_VECT2
	};
	static const vect2_t warn_rng_tp[] = {
		VECT2(0,		FPM2MPS(-1500)),
		VECT2(FEET2MET(346),	FPM2MPS(-1765)),
		VECT2(FEET2MET(1958),	FPM2MPS(-10000)),
		VECT2(FEET2MET(2000),	-INF_VS),
		NULL_VECT2
	};
	static const vect2_t caut_rng_jet[] = {
		VECT2(0,		FPM2MPS(-1031)),
		VECT2(FEET2MET(2450),	FPM2MPS(-4700)),
		VECT2(FEET2MET(2500),	-INF_VS),
		NULL_VECT2
	};
	static const vect2_t warn_rng_jet[] = {
		VECT2(0,		FPM2MPS(-1500)),
		VECT2(FEET2MET(346),	FPM2MPS(-1765)),
		VECT2(FEET2MET(2450),	FPM2MPS(-7000)),
		VECT2(FEET2MET(2500),	-INF_VS),
		NULL_VECT2
	};
	double caut_vs, warn_vs;

	if (isnan(pos->vs) || isnan(pos->ra) || isnan(pos->pos.elev) ||
	    pos->ra > RA_LIMIT || pos->on_gnd)
		return;

	if (conf.jet) {
		caut_vs = fx_lin_multi(pos->ra, caut_rng_jet, B_FALSE);
		warn_vs = fx_lin_multi(pos->ra, warn_rng_jet, B_FALSE);
	} else {
		caut_vs = fx_lin_multi(pos->ra, caut_rng_tp, B_FALSE);
		warn_vs = fx_lin_multi(pos->ra, warn_rng_tp, B_FALSE);
	}

	if (pos->vs > warn_vs) {
		sched_sound(SND_PUP);
	} else if (pos->vs >= caut_vs) {
		double next_caut_vs;
		double caut_vs_20pct = caut_vs * 0.2;

		if (state.mk8.mode1.last_caut_vs < caut_vs) {
			next_caut_vs = caut_vs;
		} else {
			next_caut_vs = ceil(pos->vs / caut_vs_20pct) *
			    caut_vs_20pct;
		}
		if (pos->vs >= next_caut_vs &&
		    next_caut_vs > state.mk8.mode1.last_caut_vs) {
			state.mk8.mode1.last_caut_vs = next_caut_vs;
			sched_sound(SND_SINKRATE);
		}
	} else {
		state.mk8.mode1.last_caut_vs = 0;
	}
}

/*
 * Excessive Descent Rate
 *
 * Caution about excessive vertical speed when close to terrain (based on
 * terrain DB).
 */
static void
tawsb_edr(const egpws_pos_t *pos)
{
	static const vect2_t caut_rng[] = {
		VECT2(0,		FPM2MPS(-1000)),
		VECT2(FEET2MET(1500),	FPM2MPS(-2350)),
		VECT2(FEET2MET(3100),	FPM2MPS(-4850)),
		VECT2(FEET2MET(5500),	FPM2MPS(-12000)),
		NULL_VECT2
	};
	static const vect2_t warn_rng[] = {
		VECT2(0,		FPM2MPS(-1300)),
		VECT2(FEET2MET(1150),	FPM2MPS(-2350)),
		VECT2(FEET2MET(2450),	FPM2MPS(-4850)),
		VECT2(FEET2MET(4400),	FPM2MPS(-12000)),
		NULL_VECT2
	};
	static const double max_hgt = FEET2MET(5500);
	double terr_elev, hgt, caut_vs, warn_vs;

	if (IS_NULL_GEO_POS(pos->pos) || isnan(pos->vs) || pos->on_gnd)
		goto errout;
	terr_elev = terr_get_elev(GEO_POS2(pos->pos.lat, pos->pos.lon));
	if (isnan(terr_elev))
		terr_elev = 0;
	hgt = MAX(pos->pos.elev - terr_elev, 0);
	if (hgt > max_hgt)
		goto errout;

	caut_vs = fx_lin_multi(hgt, caut_rng, B_FALSE);
	warn_vs = fx_lin_multi(hgt, warn_rng, B_FALSE);
	dbg_log(egpws, 2, "edr| caut: %.1f  warn: %.1f  vs: %.1f", caut_vs,
	    warn_vs, pos->vs);

	if (pos->vs <= warn_vs) {
		dbg_log(egpws, 2, "edr| pull up");
		state.tawsb.edr.lo_caut = B_TRUE;
		state.tawsb.edr.hi_caut = B_TRUE;
		sched_sound(SND_PUP);
	} else if (pos->vs <= caut_vs) {
		if (!state.tawsb.edr.lo_caut) {
			dbg_log(egpws, 2, "edr| lo_caut");
			state.tawsb.edr.lo_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
		} else if (!state.tawsb.edr.hi_caut &&
		    pos->vs < (caut_vs + warn_vs) / 2) {
			dbg_log(egpws, 2, "edr| hi_caut");
			state.tawsb.edr.hi_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
		}
	} else {
		dbg_log(egpws, 3, "edr| clear");
		state.tawsb.edr.lo_caut = B_FALSE;
		state.tawsb.edr.hi_caut = B_FALSE;
	}

errout:
	state.tawsb.edr.hi_caut = B_FALSE;
	state.tawsb.edr.lo_caut = B_FALSE;
	return;
}

static double
tawsb_dest_dist(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest)
{
	vect3_t my_ecef, dest_ecef;
	vect2_t my_pos_2d;
	airport_t *arpt;
	double dist;

	if (dest->icao[0] == '\0' || IS_NULL_GEO_POS(pos->pos)) {
		/* No destination set, return infinity */
		dbg_log(egpws, 2, "dest_dist| inf");
		return (1e10);
	}
	ASSERT(!IS_NULL_GEO_POS(dest->pos));

	my_ecef = geo2ecef(pos->pos, &wgs84);
	dest_ecef = geo2ecef(dest->pos, &wgs84);
	dist = vect3_abs(vect3_sub(dest_ecef, my_ecef));

	/* Check if we are closer to a runway threshold. */
	arpt = airport_lookup(&db, dest->icao, GEO3_TO_GEO2(pos->pos));
	if (arpt == NULL)
		return (dist);

	my_pos_2d = geo2fpp(GEO3_TO_GEO2(pos->pos), &arpt->fpp);
	for (runway_t *rwy = avl_first(&arpt->rwys); rwy != NULL;
	    rwy = AVL_NEXT(&arpt->rwys, rwy)) {
		/* When we're over a runway, we'll declare zero distance. */
		if (point_in_poly(my_pos_2d, rwy->rwy_bbox))
			return (0);

		for (int i = 0; i < 2; i++) {
			vect3_t p = geo2ecef(rwy->ends[i].thr, &wgs84);
			double d = vect3_abs(vect3_sub(p, my_ecef));
			dist = MIN(d, dist);
		}
	}

	dbg_log(egpws, 2, "dest_dist| %.0f", dist);

	return (dist);
}

//static void
//tawsb_rtc_iti(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest)

/*
 * Premature Descent Alerting
 *
 * Based on destination field elevation.
 */
static void
tawsb_pda(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest,
    double dest_dist)
{
	static const vect2_t curve[] = {
		VECT2(NM2MET(0.5),	FEET2MET(0)),
		VECT2(NM2MET(0.75),	FEET2MET(80)),
		VECT2(NM2MET(1),	FEET2MET(120)),
		VECT2(NM2MET(2),	FEET2MET(200)),
		VECT2(NM2MET(3),	FEET2MET(265)),
		VECT2(NM2MET(4),	FEET2MET(315)),
		VECT2(NM2MET(5),	FEET2MET(350)),
		VECT2(NM2MET(7),	FEET2MET(375)),
		VECT2(NM2MET(9),	FEET2MET(410)),
		VECT2(NM2MET(11),	FEET2MET(445)),
		VECT2(NM2MET(13),	FEET2MET(495)),
		VECT2(NM2MET(14),	FEET2MET(535)),
		VECT2(NM2MET(14.75),	FEET2MET(600)),
		VECT2(NM2MET(15),	FEET2MET(700)),
		NULL_VECT2
	};
	double min_elev, min_hgt;

	min_hgt = fx_lin_multi(dest_dist, curve, B_FALSE);
	if (isnan(min_hgt) || pos->on_gnd)
		return;

	min_elev = min_hgt + dest->pos.elev;
	dbg_log(egpws, 1, "pda| elv: %.0f  min: %.0f", pos->pos.elev, min_elev);
	if (pos->pos.elev < min_elev)
		sched_sound(SND_TOO_LOW_TERR);
}

static void
tawsb_500(const egpws_pos_t *pos)
{
	list_t *arpts = find_nearest_airports(&db, GEO3_TO_GEO2(pos->pos));
	double nrst_arpt_dist = 1e10;
	double nrst_rwy_dist = 1e10;
	geo_pos3_t nrst_rwy_pos = NULL_GEO_POS3;
	vect3_t my_ecef = geo2ecef(pos->pos, &wgs84);
	double gnd_elev, ra;

	for (airport_t *arpt = list_head(arpts); arpt != NULL;
	    arpt = list_next(arpts, arpt)) {
		double d = vect3_abs(vect3_sub(arpt->ecef, my_ecef));

		if (d > TAWSB_ARPT_500_DIST_LIM || d > nrst_arpt_dist)
			continue;
		nrst_arpt_dist = d;

		for (runway_t *rwy = avl_first(&arpt->rwys);
		    rwy != NULL; rwy = AVL_NEXT(&arpt->rwys, rwy)) {
			for (int i = 0; i < 2; i++) {
				double rd = vect3_abs(vect3_sub(geo2ecef(
				    rwy->ends[i].thr, &wgs84), my_ecef));
				if (rd < nrst_rwy_dist) {
					nrst_rwy_dist = rd;
					nrst_rwy_pos = rwy->ends[i].thr;
				}
			}
		}
	}

	dbg_log(egpws, 2, "500| nrst arpt: %.0f  rwy: %.4fx%.4fx%.0f",
	    nrst_arpt_dist, nrst_rwy_pos.lat, nrst_rwy_pos.lon,
	    nrst_rwy_pos.elev);

	if (!IS_NULL_GEO_POS(nrst_rwy_pos)) {
		gnd_elev = nrst_rwy_pos.elev;
	} else {
		gnd_elev = terr_get_elev(GEO3_TO_GEO2(pos->pos));
		if (isnan(gnd_elev))
			gnd_elev = 0;
	}
	ra = pos->pos.elev - gnd_elev;
	if (ra < FEET2MET(510)) {
		if (!state.tawsb.ann_500ft) {
			dbg_log(egpws, 1, "500| CALL elv:%.0f  gnd:%.0f  "
			    "ra:%.0f", pos->pos.elev, gnd_elev, ra);
			if (!pos->on_gnd)
				sched_sound(SND_RA_500_BOEING);
			state.tawsb.ann_500ft = B_TRUE;
		}
	} else if (ra > FEET2MET(600) && state.tawsb.ann_500ft) {
		dbg_log(egpws, 1, "500| CLR elv:%.0f  gnd:%.0f  ra:%.0f",
		    pos->pos.elev, gnd_elev, ra);
		state.tawsb.ann_500ft = B_FALSE;
	}

	free_nearest_airport_list(arpts);
}

static void
tawsb(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest)
{
	double dest_dist = tawsb_dest_dist(pos, dest);
	tawsb_edr(pos);
	tawsb_pda(pos, dest, dest_dist);
	tawsb_500(pos);
}

static void
main_loop(void)
{
	uint64_t now = microclock();

	ASSERT(inited);

	egpws_boot();

	mutex_enter(&lock);
	while (!shutdown) {
		egpws_pos_t pos;
		egpws_arpt_ref_t dest;

		if (init_error)
			goto out;
		mutex_exit(&lock);

		mutex_enter(&glob_data.lock);
		pos = glob_data.pos;
		memcpy(&dest, &glob_data.dest, sizeof (dest));
		mutex_exit(&glob_data.lock);

		if (conf.type == EGPWS_MK_VIII)
			mk8_mode1(&pos);
		else
			tawsb(&pos, &dest);
		unload_distant_airport_tiles(&db, GEO3_TO_GEO2(pos.pos));

		mutex_enter(&lock);
out:
		cv_timedwait(&cv, &lock, now + RUN_INTVAL);
		now = microclock();
	}
	mutex_exit(&lock);

	egpws_shutdown();
}

void
egpws_init(egpws_conf_t acf_conf)
{
	ASSERT(!inited);

	inited = B_TRUE;
	shutdown = B_FALSE;
	mutex_init(&lock);
	cv_init(&cv);

	conf = acf_conf;

	memset(&glob_data, 0, sizeof (glob_data));
	mutex_init(&glob_data.lock);
	glob_data.pos.pos = NULL_GEO_POS3;

	memset(&state, 0, sizeof (state));

	dbg_log(egpws, 3, "EGPWS type: %s",
	    conf.type == EGPWS_MK_VIII ? "MK VIII" : "TAWS-B");

	VERIFY(thread_create(&worker, (int(*)(void *))main_loop, NULL));

	dbg_log(egpws, 1, "init");
}

void
egpws_fini(void)
{
	dbg_log(egpws, 1, "fini");

	if (!inited)
		return;

	mutex_enter(&lock);
	shutdown = B_TRUE;
	cv_broadcast(&cv);
	mutex_exit(&lock);

	thread_join(&worker);

	mutex_destroy(&glob_data.lock);
	mutex_destroy(&lock);
	cv_destroy(&cv);

	inited = B_FALSE;
}

void
egpws_set_position(egpws_pos_t pos)
{
	ASSERT(inited);
	mutex_enter(&glob_data.lock);
	glob_data.pos = pos;
	dbg_log(cfg, 1, "set pos %.4f x %.4f x %.0f",
	    pos.pos.lat, pos.pos.lon, pos.pos.elev);
	mutex_exit(&glob_data.lock);
}

void
egpws_set_flaps_ovrd(bool_t flag)
{
	ASSERT(inited);
	if (glob_data.flaps_ovrd != flag)
		dbg_log(cfg, 1, "set flaps ovrd %d", flag);
	glob_data.flaps_ovrd = flag;
}

void
egpws_set_dest(const egpws_arpt_ref_t *ref)
{
	mutex_enter(&glob_data.lock);
	if (strcmp(ref->icao, glob_data.dest.icao) != 0) {
		dbg_log(cfg, 1, "set dest \"%s\"", ref->icao);
		if (ref->icao[0] != '\0')
			VERIFY(!IS_NULL_GEO_POS(ref->pos));
		memcpy(&glob_data.dest, ref, sizeof (glob_data.dest));
	}
	mutex_exit(&glob_data.lock);
}
