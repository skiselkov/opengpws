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

#define	RUN_INTVAL		0.5		/* seconds */
#define	INF_VS			1e10		/* m/s */
#define	RA_LIMIT		FEET2MET(2500)
#define	TAWSB_ARPT_500_DIST_LIM	NM2MET(5)
#define	MAX_NCR_HDG_CHG		110		/* degrees */
#define	MAX_NCR_HGT		FEET2MET(700)
#define	MAX_NCR_DIST		NM2MET(5)
#define	NCR_SUPRESS_CLB_RATE	FPM2MPS(300)
#define	D_TRK_UPD_RATE		2		/* seconds */
#define	ARPT_LOAD_LIMIT		NM2MET(23)

static bool_t		inited = B_FALSE;
static bool_t		main_shutdown = B_FALSE;
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
	double			d_trk;		/* rate of change of trk */
	double			last_pos_update; /* time (in seconds) */
} glob_data;

static struct {
	mutex_t			adv_lock;
	egpws_advisory_t	adv;
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
		struct {
			bool_t		active;
			geo_pos3_t	liftoff_pt;	/* lift-off point */
			double		liftoff_hdg;
			double		max_hgt;
			double		prev_hgt;
			double		max_alt;
			double		prev_alt;
		} ncr;
		struct {
			bool_t		ahead_played;
			double		last_caut_time;
		} rtc;
	} tawsb;
} state;

static void
egpws_boot(void)
{
	const char *plugindir = get_plugindir();
	char *cachedir = mkpathname(plugindir, "airport.cache", NULL);

	airportdb_create(&db, get_xpdir(), cachedir);
	set_airport_load_limit(&db, ARPT_LOAD_LIMIT);
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
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP);
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
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
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
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP);
	} else if (pos->vs <= caut_vs) {
		if (!state.tawsb.edr.lo_caut) {
			dbg_log(egpws, 2, "edr| lo_caut");
			state.tawsb.edr.lo_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
		} else if (!state.tawsb.edr.hi_caut &&
		    pos->vs < (caut_vs + warn_vs) / 2) {
			dbg_log(egpws, 2, "edr| hi_caut");
			state.tawsb.edr.hi_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
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

static void
tawsb_db_arpt_dist(const egpws_pos_t *pos, airport_t *arpt,
    double *arpt_dist_p, double *arpt_hgt_p)
{
	vect3_t my_ecef = geo2ecef(GEO3_FT2M(pos->pos), &wgs84);
	vect3_t dest_ecef = geo2ecef(GEO3_FT2M(arpt->refpt), &wgs84);
	double dist = vect3_abs(vect3_sub(dest_ecef, my_ecef));
	double elev = FEET2MET(arpt->refpt.elev);
	vect2_t my_pos_2d = geo2fpp(GEO3_TO_GEO2(pos->pos), &arpt->fpp);

	for (runway_t *rwy = avl_first(&arpt->rwys); rwy != NULL;
	    rwy = AVL_NEXT(&arpt->rwys, rwy)) {
		/* When we're over a runway, we'll declare zero distance. */
		if (point_in_poly(my_pos_2d, rwy->rwy_bbox)) {
			*arpt_dist_p = 0;
			/* Assume the runway elev is the mean of its ends */
			*arpt_hgt_p = pos->pos.elev -
			    FEET2MET((rwy->ends[0].thr.elev +
			    rwy->ends[1].thr.elev) / 2);
			return;
		}

		for (int i = 0; i < 2; i++) {
			vect3_t p = geo2ecef(GEO3_FT2M(rwy->ends[i].thr),
			    &wgs84);
			double d = vect3_abs(vect3_sub(p, my_ecef));
			if (d < dist) {
				dist = d;
				elev = FEET2MET(rwy->ends[i].thr.elev);
			}
		}
	}

	*arpt_dist_p = dist;
	*arpt_hgt_p = pos->pos.elev - elev;
}

static bool_t
tawsb_nearest_arpt_or_rwy(const egpws_pos_t *pos, double *arpt_dist_p,
    double *arpt_hgt_p)
{
	list_t *arpts = find_nearest_airports(&db, GEO3_TO_GEO2(pos->pos));
	double dist = NAN, hgt = NAN;

	for (airport_t *arpt = list_head(arpts); arpt != NULL;
	    arpt = list_next(arpts, arpt)) {
		double arpt_dist, arpt_hgt;
		tawsb_db_arpt_dist(pos, arpt, &arpt_dist, &arpt_hgt);
		if (isnan(dist) || arpt_dist < dist) {
			dist = arpt_dist;
			hgt = arpt_hgt;
		}
	}

	free_nearest_airport_list(arpts);

	if (!isnan(dist) && !isnan(hgt)) {
		*arpt_dist_p = dist;
		*arpt_hgt_p = hgt;
		return (B_TRUE);
	} else {
		return (B_FALSE);
	}
}

static void
tawsb_dest_dist(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest,
    double *dest_dist_p, double *dest_hgt_p)
{
	airport_t *arpt;

	if (dest->icao[0] == '\0' || IS_NULL_GEO_POS(pos->pos)) {
		/* No destination set, return infinity */
		dbg_log(egpws, 2, "dest_dist| inf");
		*dest_dist_p = 1e10;
		*dest_hgt_p = 1e10;
		return;
	}
	ASSERT(!IS_NULL_GEO_POS(dest->pos));

	/* Check if we are closer to a runway threshold. */
	arpt = airport_lookup(&db, dest->icao, GEO3_TO_GEO2(pos->pos));
	if (arpt == NULL) {
		/* Use the externally-provided info */
		*dest_dist_p = vect3_abs(vect3_sub(geo2ecef(pos->pos, &wgs84),
		    geo2ecef(dest->pos, &wgs84)));
		*dest_hgt_p = pos->pos.elev - dest->pos.elev;
		return;
	}

	tawsb_db_arpt_dist(pos, arpt, dest_dist_p, dest_hgt_p);

	dbg_log(egpws, 2, "dest| dist:%.0f  hgt:%.0f", *dest_dist_p,
	    *dest_hgt_p);
}

static void
tawsb_rtc_iti(const egpws_pos_t *pos, double d_trk)
{
#define	RTC_SIM_STEP		RUN_INTVAL
#define	RTC_NUM_STEPS_MAX	(45 / RTC_SIM_STEP)	/* simulate 35s */
#define	RTC_NUM_STEPS_MIN	(30 / RTC_SIM_STEP)	/* simulate 20s */
#define	RTC_WARN_STEP_FRACT	0.8
#define	RTC_MAX_D_TRK_RATE	3			/* deg/s */
#define	RTC_DES_THRESH		FPM2MPS(-300)
#define	RTC_INH_DIST_THRESH	NM2MET(0.5)
#define	RTC_INH_HGT_THRESH	FEET2MET(200)
#define	RTC_CAUT_INTVAL		5			/* seconds */
#define	RTC_RISING_TERR_THRESH	FEET2MET(150)

	static const vect2_t lvl_curve[] = {
		VECT2(0,		0),
		VECT2(NM2MET(0.5),	0),
		VECT2(NM2MET(2),	FEET2MET(175)),
		VECT2(NM2MET(23),	FEET2MET(700)),
		VECT2(1e11,		FEET2MET(700)),
		NULL_VECT2
	};
	static const vect2_t des_curve[] = {
		VECT2(0,		0),
		VECT2(NM2MET(0.5),	0),
		VECT2(NM2MET(2),	FEET2MET(120)),
		VECT2(NM2MET(17.5),	FEET2MET(500)),
		VECT2(1e11,		FEET2MET(500)),
		NULL_VECT2
	};
	static const vect2_t coll_curve[] = {
		VECT2(NM2MET(0),	FEET2MET(50)),
		VECT2(NM2MET(0.5),	FEET2MET(100)),
		VECT2(1e11,		FEET2MET(100)),
		NULL_VECT2
	};
	double rqd_clr, coll_clr, trk, arpt_dist, arpt_hgt;
	fpp_t fpp;
	vect2_t p;
	double cur_hgt = NAN;
	int num_sim_steps = wavg(RTC_NUM_STEPS_MAX, RTC_NUM_STEPS_MIN,
	    iter_fract(ABS(d_trk), 0, RTC_MAX_D_TRK_RATE, B_TRUE));
	double hgt_samples[num_sim_steps];
	bool_t caut_found = B_FALSE, warn_found = B_FALSE;
	double min_hgt = 1e10;

	if (pos->on_gnd)
		return;

	if (!tawsb_nearest_arpt_or_rwy(pos, &arpt_dist, &arpt_hgt)) {
		arpt_dist = 1e10;
		arpt_hgt = 1e10;
	}

	/* Inihibit within 0.5 NM and less than 200 ft above destination */
	if (arpt_dist < RTC_INH_DIST_THRESH && arpt_hgt < RTC_INH_HGT_THRESH)
		return;

	if (pos->vs > RTC_DES_THRESH)
		rqd_clr = fx_lin_multi(arpt_dist, lvl_curve, B_FALSE);
	else
		rqd_clr = fx_lin_multi(arpt_dist, des_curve, B_FALSE);
	coll_clr = fx_lin_multi(arpt_dist, coll_curve, B_FALSE);
	ASSERT(!isnan(rqd_clr));
	ASSERT(!isnan(coll_clr));

	trk = pos->trk;
	p = ZERO_VECT2;
	fpp = ortho_fpp_init(GEO3_TO_GEO2(pos->pos), 0, &wgs84, B_TRUE);

	for (int i = 0; i < num_sim_steps; i++) {
		geo_pos2_t gp = fpp2geo(p, &fpp);
		double terr_elev = terr_get_elev_wide(gp);
		vect2_t vel;

		ASSERT(!IS_NULL_GEO_POS(gp));

		if (isnan(terr_elev))
			terr_elev = 0;
		hgt_samples[i] = pos->pos.elev - terr_elev;
		if (i == 0)
			cur_hgt = hgt_samples[i];

		/* Integrate our position variables */
		trk += d_trk * RTC_SIM_STEP;
		vel = vect2_scmul(hdg2dir(trk), pos->gs * RTC_SIM_STEP);
		p = vect2_add(p, vel);
	}
	ASSERT(!isnan(cur_hgt));

	for (int i = 0; i < num_sim_steps; i++) {
		if (i < num_sim_steps * RTC_WARN_STEP_FRACT &&
		    hgt_samples[i] < coll_clr) {
			warn_found = B_TRUE;
			break;
		}
		if (hgt_samples[i] < rqd_clr)
			caut_found = B_TRUE;
		min_hgt = MIN(min_hgt, hgt_samples[i]);
	}

	if (warn_found) {
		dbg_log(rtc, 1, "rtc| min_hgt:%.0f  rqd:%.0f  coll:%.0f = "
		    "PULL UP", min_hgt, rqd_clr, coll_clr);
		if (!state.tawsb.rtc.ahead_played) {
			sched_sound(SND_TERR_AHEAD_PUP);
			state.tawsb.rtc.ahead_played = B_TRUE;
		}
		sched_sound(SND_PUP);
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP);
	} else if (caut_found) {
		snd_id_t snd;
		double now = USEC2SEC(microclock());
		dbg_log(rtc, 1, "rtc| min_hgt:%.0f  rqd:%.0f  coll:%.0f = "
		    "TERR AHEAD", min_hgt, rqd_clr, coll_clr);

		if (cur_hgt - min_hgt < RTC_RISING_TERR_THRESH)
			snd = SND_TOO_LOW_TERR;
		else
			snd = SND_TERR_AHEAD;
		state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);

		if (now - state.tawsb.rtc.last_caut_time > RTC_CAUT_INTVAL) {
			sched_sound(snd);
			state.tawsb.rtc.last_caut_time = now;
		}
		state.tawsb.rtc.ahead_played = B_FALSE;
	} else {
		dbg_log(rtc, 1, "rtc| min_hgt:%.0f  rqd: %.0f  coll:%.0f = OK",
		    min_hgt, rqd_clr, coll_clr);
		state.tawsb.rtc.ahead_played = B_FALSE;
	}
}

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
	if (pos->pos.elev < min_elev) {
		sched_sound(SND_TOO_LOW_TERR);
		state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
	}
}

/*
 * 500 ft callout
 *
 * Aircraft has descended to 500 ft or lower height a certain platform
 * altitude. The altitude of that platform is either:
 *
 * 1) If <= 5 NM from an airport, the threshold elevation of the
 *	nearest runway threshold, OR
 * 2) If > 5 NM from an airport, based on terrain DB elevation.
 */
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
				vect3_t thr_p = geo2ecef(
				    GEO3_FT2M(rwy->ends[i].thr), &wgs84);
				double rd = vect3_abs(vect3_sub(thr_p,
				    my_ecef));
				if (rd < nrst_rwy_dist) {
					nrst_rwy_dist = rd;
					nrst_rwy_pos =
					    GEO3_FT2M(rwy->ends[i].thr);
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

/*
 * Negative Climb Rate (after takeoff)
 */
static void
tawsb_ncr(const egpws_pos_t *pos)
{
	static const vect2_t alt_curve[] = {
		VECT2(FEET2MET(50),	FEET2MET(-10)),
		VECT2(FEET2MET(700),	FEET2MET(-70)),
		NULL_VECT2
	};
	static const vect2_t vs_curve[] = {
		VECT2(FEET2MET(50),	FPM2MPS(-100)),
		VECT2(FEET2MET(700),	FPM2MPS(-500)),
		NULL_VECT2
	};
	vect3_t my_ecef, dep_ecef;
	double hgt, d_hdg, dist, vs_hgt, vs_alt, alt_lim, vs_lim, d_hgt, d_alt;
	double terr_elev;

	/* Reactivate NCR on the ground */
	if (pos->on_gnd) {
		dbg_log(egpws, 1, "ncr| on_gnd");
		state.tawsb.ncr.active = B_TRUE;
		state.tawsb.ncr.liftoff_pt = pos->pos;
		state.tawsb.ncr.liftoff_hdg = pos->trk;
		state.tawsb.ncr.max_hgt = 0;
		state.tawsb.ncr.prev_hgt = 0;
		state.tawsb.ncr.max_alt = pos->pos.elev;
		state.tawsb.ncr.prev_alt = pos->pos.elev;
		return;
	}
	if (!state.tawsb.ncr.active)
		return;

	/* Validate the NCR termination conditions */
	my_ecef = geo2ecef(pos->pos, &wgs84);
	dep_ecef = geo2ecef(state.tawsb.ncr.liftoff_pt, &wgs84);

	dist = vect3_abs(vect3_sub(my_ecef, dep_ecef));
	terr_elev = terr_get_elev(GEO3_TO_GEO2(pos->pos));
	if (isnan(terr_elev))
		terr_elev = 0;
	hgt = pos->pos.elev - terr_elev;
	d_hdg = ABS(rel_hdg(pos->trk, state.tawsb.ncr.liftoff_hdg));

	if (hgt > MAX_NCR_HGT || d_hdg >= MAX_NCR_HDG_CHG ||
	    dist > MAX_NCR_DIST) {
		dbg_log(egpws, 1, "ncr| term  hgt:%.0f  d_hdg:%.0f  dist:%.0f",
		    hgt, d_hdg, dist);
		state.tawsb.ncr.active = B_FALSE;
		return;
	}

	vs_hgt = (hgt - state.tawsb.ncr.prev_hgt) / RUN_INTVAL;
	vs_alt = (pos->pos.elev - state.tawsb.ncr.prev_alt) / RUN_INTVAL;

	d_hgt = hgt - state.tawsb.ncr.max_hgt;
	d_alt = pos->pos.elev - state.tawsb.ncr.max_alt;

	alt_lim = fx_lin_multi(hgt, alt_curve, B_FALSE);
	vs_lim = fx_lin_multi(hgt, vs_curve, B_FALSE);

	dbg_log(egpws, 1, "ncr| vs_hgt:%.1f  vs_alt:%.1f  "
	    "d_hgt:%.0f  d_alt:%.0f  alt_lim:%.0f  vs_lim:%.0f",
	    vs_hgt, vs_alt, d_hgt, d_alt, alt_lim, vs_lim);

	if (vs_hgt < vs_lim || d_hgt < alt_lim) {
		/*
		 * If we're descending, advise against descent,
		 * otherwise advise of rising terrain.
		 */
		if (vs_hgt < NCR_SUPRESS_CLB_RATE) {
			if (vs_alt < 0)
				sched_sound(SND_DONT_SINK);
			else
				sched_sound(SND_TOO_LOW_TERR);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
		}
	} else if (vs_alt < vs_lim || d_alt < alt_lim) {
		if (vs_hgt < NCR_SUPRESS_CLB_RATE) {
			sched_sound(SND_DONT_SINK);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
		}
	}

	state.tawsb.ncr.prev_hgt = hgt;
	state.tawsb.ncr.max_hgt = MAX(state.tawsb.ncr.max_hgt, hgt);
	state.tawsb.ncr.prev_alt = pos->pos.elev;
	state.tawsb.ncr.max_alt = MAX(state.tawsb.ncr.max_alt, pos->pos.elev);
}

static void
tawsb(const egpws_pos_t *pos, const egpws_arpt_ref_t *dest, double d_trk)
{
	double dest_dist, dest_hgt;

	/* TAWS-B needs position */
	if (IS_NULL_GEO_POS(pos->pos))
		return;

	tawsb_dest_dist(pos, dest, &dest_dist, &dest_hgt);
	tawsb_edr(pos);
	tawsb_pda(pos, dest, dest_dist);
	tawsb_500(pos);
	tawsb_ncr(pos);
	tawsb_rtc_iti(pos, d_trk);
}

static void
main_loop(void)
{
	uint64_t now = microclock();

	ASSERT(inited);

	egpws_boot();

	mutex_enter(&lock);
	while (!main_shutdown) {
		egpws_pos_t pos;
		double d_trk;
		egpws_arpt_ref_t dest;

		if (init_error)
			goto out;
		mutex_exit(&lock);

		mutex_enter(&glob_data.lock);
		pos = glob_data.pos;
		d_trk = glob_data.d_trk;
		memcpy(&dest, &glob_data.dest, sizeof (dest));
		mutex_exit(&glob_data.lock);

		mutex_enter(&state.adv_lock);
		state.adv = EGPWS_ADVISORY_NONE;
		if (conf.type == EGPWS_MK_VIII)
			mk8_mode1(&pos);
		else
			tawsb(&pos, &dest, d_trk);
		mutex_exit(&state.adv_lock);
		unload_distant_airport_tiles(&db, GEO3_TO_GEO2(pos.pos));

		mutex_enter(&lock);
out:
		cv_timedwait(&cv, &lock, now + SEC2USEC(RUN_INTVAL));
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
	main_shutdown = B_FALSE;
	mutex_init(&lock);
	cv_init(&cv);

	conf = acf_conf;

	memset(&glob_data, 0, sizeof (glob_data));
	mutex_init(&glob_data.lock);
	mutex_init(&state.adv_lock);
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
	main_shutdown = B_TRUE;
	cv_broadcast(&cv);
	mutex_exit(&lock);

	thread_join(&worker);

	mutex_destroy(&state.adv_lock);
	mutex_destroy(&glob_data.lock);
	mutex_destroy(&lock);
	cv_destroy(&cv);

	inited = B_FALSE;
}

void
egpws_set_position(egpws_pos_t pos, double now)
{
	double d_t;

	ASSERT(inited);

	mutex_enter(&glob_data.lock);

	d_t = now - glob_data.last_pos_update;
	ASSERT3F(d_t, >, 0);
	if (!isnan(glob_data.pos.trk) && !isnan(pos.trk)) {
		double rel_trk = rel_hdg(glob_data.pos.trk, pos.trk) / d_t;
		FILTER_IN(glob_data.d_trk, rel_trk, d_t, D_TRK_UPD_RATE);
	} else if (!isnan(pos.trk)) {
		glob_data.d_trk = pos.trk;
	} else {
		glob_data.d_trk = 0;
	}
	glob_data.pos = pos;
	glob_data.last_pos_update = now;

	mutex_exit(&glob_data.lock);

	dbg_log(cfg, 1, "set pos %.4f x %.4f x %.0f   (d_trk: %.1f)",
	    pos.pos.lat, pos.pos.lon, pos.pos.elev, glob_data.d_trk);
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

const egpws_conf_t *
egpws_get_conf(void)
{
	return (&conf);
}

egpws_advisory_t
egpws_get_advisory(void)
{
	egpws_advisory_t adv;

	mutex_enter(&state.adv_lock);
	adv = state.adv;
	mutex_exit(&state.adv_lock);

	return (adv);
}
