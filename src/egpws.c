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
 * Copyright 2020 Saso Kiselkov. All rights reserved.
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
#define	OBST_REFRESH_INTVAL	10		/* seconds */
#define	OBST_DIST_RANGE		20000		/* meters */
#define	OBST_HGT_RANGE		1000		/* meters */

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
	egpws_impact_t		imp;		/* impact points */
} glob_data;

static struct {
	odb_t			*odb;
	mutex_t			adv_lock;
	egpws_advisory_t	adv;
	list_t			obstacles;
	time_t			obst_refresh_t;
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
			double		vs_alt;
			double		vs_hgt;
		} ncr;
		struct {
			bool_t		ahead_played;
			double		last_caut_time;
		} rtc;
		struct {
			/* protected by adv_lock */
			egpws_obst_impact_t	imp;
		} roc;
	} tawsb;
} state = { NULL };

typedef struct {
	geo_pos3_t	pos;
	vect3_t		pos_ecef;
	double		agl;
	unsigned	quant;
	list_node_t	node;
} obst_t;

typedef struct {
	geo_pos3_t	pos;
	vect3_t		ecef;
} pos_info_t;

static void
egpws_boot(void)
{
	char *cachedir = mkpathname(get_xpdir(), get_plugindir(),
	    "airport.cache", NULL);

	airportdb_create(&db, get_xpdir(), cachedir);
	db.ifr_only = conf.ifr_only;
	set_airport_load_limit(&db, ARPT_LOAD_LIMIT);
	init_error = !recreate_cache(&db);
	lacf_free(cachedir);
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
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP_TERRAIN);
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
			state.adv = MAX(state.adv, EGPWS_ADVISORY_SINKRATE_DR);
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
		dbg_log(egpws, 2, "edr| PULL UP");
		state.tawsb.edr.lo_caut = B_TRUE;
		state.tawsb.edr.hi_caut = B_TRUE;
		sched_sound(SND_PUP);
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP_DR);
	} else if (pos->vs <= caut_vs) {
		if (!state.tawsb.edr.lo_caut) {
			dbg_log(egpws, 2, "edr| lo_caut");
			state.tawsb.edr.lo_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_SINKRATE_DR);
		} else if (!state.tawsb.edr.hi_caut &&
		    pos->vs < (caut_vs + warn_vs) / 2) {
			dbg_log(egpws, 2, "edr| hi_caut");
			state.tawsb.edr.hi_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
			state.adv = MAX(state.adv, EGPWS_ADVISORY_SINKRATE_DR);
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
	vect3_t my_ecef = geo2ecef_mtr(pos->pos, &wgs84);
	vect3_t dest_ecef = geo2ecef_ft(arpt->refpt, &wgs84);
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
			vect3_t p = geo2ecef_ft(rwy->ends[i].thr, &wgs84);
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
    double *arpt_rel_hgt_p)
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

	if (!IS_NULL_GEO_POS(state.tawsb.ncr.liftoff_pt)) {
		vect3_t pos_v = geo2ecef_mtr(pos->pos, &wgs84);
		vect3_t liftoff_v = geo2ecef_mtr(state.tawsb.ncr.liftoff_pt,
		    &wgs84);
		double d = vect3_abs(vect3_sub(pos_v, liftoff_v));

		if (isnan(dist) || d < dist) {
			dist = d;
			hgt = pos->pos.elev - state.tawsb.ncr.liftoff_pt.elev;
		}
	}

	if (!isnan(dist) && !isnan(hgt)) {
		*arpt_dist_p = dist;
		*arpt_rel_hgt_p = hgt;
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
		*dest_dist_p = vect3_abs(vect3_sub(geo2ecef_mtr(pos->pos,
		    &wgs84), geo2ecef_mtr(dest->pos, &wgs84)));
		*dest_hgt_p = pos->pos.elev - dest->pos.elev;
		return;
	}

	tawsb_db_arpt_dist(pos, arpt, dest_dist_p, dest_hgt_p);

	dbg_log(egpws, 2, "dest| dist:%.0f  hgt:%.0f", *dest_dist_p,
	    *dest_hgt_p);
}

static void
clear_impact(void)
{
	mutex_enter(&glob_data.lock);
	memset(&glob_data.imp, 0, sizeof (glob_data.imp));
	mutex_exit(&glob_data.lock);
}

static int
xfer_imp_pts(geo_pos3_t imp_pts[9], geo_pos3_t terr_pos[9],  double acf_elev,
    double clr)
{
	int num_pts = 0;
	for (int i = 0; i < 9; i++) {
		if (acf_elev - terr_pos[i].elev < clr)
			imp_pts[num_pts++] = terr_pos[i];
	}
	return (num_pts);
}

static void
tawsb_rtc_iti(const egpws_pos_t *pos, double d_trk)
{
#define	RTC_SIM_STEP		1			/* second */
#define	RTC_NUM_STEPS_MAX	(45 / RTC_SIM_STEP)	/* simulate 35s */
#define	RTC_NUM_STEPS_MIN	(30 / RTC_SIM_STEP)	/* simulate 20s */
#define	RTC_WARN_STEP_FRACT	0.7
#define	RTC_MAX_D_TRK_RATE	3			/* deg/s */
#define	RTC_DES_THRESH		FPM2MPS(-300)
#define	RTC_INH_DIST_THRESH	NM2MET(0.75)
#define	RTC_INH_HGT_THRESH	FEET2MET(350)
#define	RTC_CAUT_INTVAL		5			/* seconds */
#define	RTC_RISING_TERR_THRESH	FEET2MET(150)
#define	MIN_IMPACT_PTS		5
#define	MAX_OBSTACLES		1024
#define	OBST_LAT_CLR_MIN	150			/* meters */
#define	OBST_LAT_CLR_MAX	250			/* meters */
#define	OBST_LAT_BLOOM_BASE_AGL	150			/* meters */
#define	OBST_LAT_BLOOM_MAX_MULT	2

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
	/*
	 * Correction curve based on airport proximity. The closer we are
	 * to our destination the tighter the simulation time limit is.
	 */
	static const vect2_t dist2time_curve[] = {
		VECT2(NM2MET(0),	0.5),
		VECT2(NM2MET(0.5),	0.5),
		VECT2(NM2MET(3),	1.0),
		VECT2(NM2MET(10),	1.0),
		NULL_VECT2
	};
	double rqd_clr, coll_clr, trk, arpt_dist, arpt_rel_hgt;
	fpp_t fpp;
	vect2_t p;
	double cur_hgt = NAN;
	int num_sim_steps = round(wavg(RTC_NUM_STEPS_MAX, RTC_NUM_STEPS_MIN,
	    iter_fract(ABS(d_trk), 0, RTC_MAX_D_TRK_RATE, B_TRUE)));
	/*
	 * We store two sets of height samples, one set used for cautions and
	 * one for warnings. The caution samples are computed assuming the
	 * aircraft will always remain level, or at most is descending at
	 * -500 fpm. The warning samples are computed assuming the aircraft
	 * will remain level or climb at up to 1000 fpm.
	 */
	double hgt_samples_caut[num_sim_steps], hgt_samples_warn[num_sim_steps];
	geo_pos3_t terr_pos[num_sim_steps * 9];
	geo_pos3_t imp_pts[EGPWS_MAX_NUM_IMP_PTS];
	geo_pos3_t caut_obst[MAX_OBSTACLES];
	geo_pos3_t warn_obst[MAX_OBSTACLES];
	int num_imp_pts = 0;
	int num_caut_obst = 0;
	int num_warn_obst = 0;
	bool_t caut_found = B_FALSE, warn_found = B_FALSE;
	bool_t caut_obst_found = B_FALSE, warn_obst_found = B_FALSE;
	double min_hgt = 1e10;
	double vs_caut, vs_warn;
	egpws_obst_impact_t *obst_imp = &state.tawsb.roc.imp;

	/* Validate input data */
	if (pos->on_gnd || IS_NULL_GEO_POS(pos->pos) || isnan(pos->vs) ||
	    isnan(pos->gs) || isnan(pos->trk)) {
		clear_impact();
		return;
	}

	if (!tawsb_nearest_arpt_or_rwy(pos, &arpt_dist, &arpt_rel_hgt)) {
		arpt_dist = 1e10;
		arpt_rel_hgt = 1e10;
	}

	/*
	 * Inihibit within 0.75 NM and less than 350 ft above destination.
	 * Spec says 0.5NM and 200 ft, but I find it's a bit too tight.
	 */
	if (arpt_dist < RTC_INH_DIST_THRESH &&
	    arpt_rel_hgt < RTC_INH_HGT_THRESH) {
		clear_impact();
		return;
	}

	num_sim_steps = round(num_sim_steps *
	    fx_lin_multi(arpt_dist, dist2time_curve, B_TRUE));

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
	vs_caut = clamp(pos->vs, -FPM2MPS(500), 0);
	vs_warn = clamp(pos->vs, 0, FPM2MPS(1000));

	for (int i = 0; i < num_sim_steps; i++) {
		double elev_caut = (pos->pos.elev + vs_caut * i * RTC_SIM_STEP);
		double elev_warn = (pos->pos.elev + vs_warn * i * RTC_SIM_STEP);
		geo_pos2_t gp = fpp2geo(p, &fpp);
		double terr_elev = terr_get_elev_wide(gp, &terr_pos[i * 9]);
		vect2_t vel;
		vect3_t ecef = geo2ecef_mtr(GEO2_TO_GEO3(gp, pos->pos.elev),
		    &wgs84);

		ASSERT(!IS_NULL_GEO_POS(gp));

		if (isnan(terr_elev))
			terr_elev = 0;
		hgt_samples_caut[i] = elev_caut - terr_elev;
		hgt_samples_warn[i] = elev_warn - terr_elev;
		if (i == 0)
			cur_hgt = hgt_samples_caut[i];

		for (obst_t *obst = list_head(&state.obstacles);
		    obst != NULL; obst = list_next(&state.obstacles, obst)) {
			double obst_dist =
			    vect3_abs(vect3_sub(ecef, obst->pos_ecef));
			double rqd_lat_clr = wavg(
			    OBST_LAT_CLR_MIN, OBST_LAT_CLR_MAX,
			    iter_fract(obst->quant, 1, 3, B_TRUE));
			double d_elev_caut = elev_caut -
			    (obst->pos.elev + obst->agl);
			double d_elev_warn = elev_warn -
			    (obst->pos.elev + obst->agl);

			rqd_lat_clr *= clamp(
			    obst->agl / OBST_LAT_BLOOM_BASE_AGL,
			    1, OBST_LAT_BLOOM_MAX_MULT);

			if (obst_dist < rqd_lat_clr && d_elev_warn < coll_clr &&
			    num_warn_obst < MAX_OBSTACLES &&
			    i < num_sim_steps * RTC_WARN_STEP_FRACT) {
				warn_obst[num_warn_obst] = obst->pos;
				num_warn_obst++;
			} else if (obst_dist < rqd_lat_clr &&
			    d_elev_caut < rqd_clr &&
			    num_caut_obst < MAX_OBSTACLES) {
				caut_obst[num_caut_obst] = obst->pos;
				num_caut_obst++;
			}
		}

		/* Integrate our position variables */
		trk += d_trk * RTC_SIM_STEP;
		vel = vect2_scmul(hdg2dir(trk), pos->gs * RTC_SIM_STEP);
		p = vect2_add(p, vel);
	}
	ASSERT(!isnan(cur_hgt));

	for (int i = 0; i < num_sim_steps; i++) {
		if (i < num_sim_steps * RTC_WARN_STEP_FRACT &&
		    hgt_samples_warn[i] < coll_clr) {
			num_imp_pts = xfer_imp_pts(imp_pts, &terr_pos[i * 9],
			    pos->pos.elev, coll_clr);
			/*
			 * Sometimes the first hit generates very few impact
			 * points. In that case, check the next step, to see
			 * if we can grab some more.
			 */
			if (num_imp_pts < MIN_IMPACT_PTS &&
			    i + 1 < num_sim_steps) {
				num_imp_pts += xfer_imp_pts(
				    &imp_pts[num_imp_pts], &terr_pos[i * 9],
				    pos->pos.elev, coll_clr);
			}
			warn_found = B_TRUE;
			break;
		}
		if (hgt_samples_caut[i] < rqd_clr) {
			if (!caut_found) {
				num_imp_pts = xfer_imp_pts(imp_pts,
				    &terr_pos[i * 9], pos->pos.elev, rqd_clr);
				if (num_imp_pts < MIN_IMPACT_PTS &&
				    i + 1 < num_sim_steps) {
					num_imp_pts += xfer_imp_pts(
					    &imp_pts[num_imp_pts],
					    &terr_pos[i * 9], pos->pos.elev,
					    coll_clr);
				}
			}
			caut_found = B_TRUE;
		}
		min_hgt = MIN(min_hgt, hgt_samples_caut[i]);
	}

	obst_imp->num_points = 0;
	for (int i = 0; i < num_warn_obst &&
	    obst_imp->num_points < EGPWS_MAX_NUM_OBST_IMP_PTS; i++) {
		obst_imp->points[obst_imp->num_points] = warn_obst[i];
		obst_imp->is_warn[obst_imp->num_points] = B_TRUE;
		obst_imp->num_points++;
	}
	for (int i = 0; i < num_caut_obst &&
	    obst_imp->num_points < EGPWS_MAX_NUM_OBST_IMP_PTS; i++) {
		obst_imp->points[obst_imp->num_points] = caut_obst[i];
		obst_imp->is_warn[obst_imp->num_points] = B_FALSE;
		obst_imp->num_points++;
	}
	warn_obst_found = (num_warn_obst > 0);
	caut_obst_found = (num_caut_obst > 0);

	/*
	 * Priority of messages:
	 * 1) Terrain warnings (PULL UP)
	 * 2) Obstacle warnings (PULL UP)
	 * 3) Terrain cautions (TERRAIN AHEAD)
	 * 4) Obstacle cautions (OBSTACLE AHEAD)
	 */
	if (warn_found) {
		dbg_log(rtc, 1, "rtc| min_hgt:%.0f  rqd:%.0f  coll:%.0f = "
		    "PULL UP", min_hgt, rqd_clr, coll_clr);
		if (!state.tawsb.rtc.ahead_played) {
			sched_sound(SND_TERR_AHEAD_PUP);
			state.tawsb.rtc.ahead_played = B_TRUE;
		}
		sched_sound(SND_PUP);
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP_TERRAIN);
	} else if (warn_obst_found) {
		dbg_log(rtc, 1, "rtc| num_warn_obst: %d = PULL UP",
		    num_warn_obst);
		if (!state.tawsb.rtc.ahead_played) {
			sched_sound(SND_OBST_AHEAD_PUP);
			state.tawsb.rtc.ahead_played = B_TRUE;
		}
		sched_sound(SND_PUP);
		state.adv = MAX(state.adv, EGPWS_ADVISORY_PULL_UP_OBSTACLE);
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
	} else if (caut_obst_found) {
		double now = USEC2SEC(microclock());

		dbg_log(rtc, 1, "rtc| num_caut_obst: %d = PULL UP",
		    num_caut_obst);
		if (now - state.tawsb.rtc.last_caut_time > RTC_CAUT_INTVAL) {
			sched_sound(SND_OBST_AHEAD);
			state.tawsb.rtc.last_caut_time = now;
		}
		state.adv = MAX(state.adv, EGPWS_ADVISORY_OBSTACLE);
	} else {
		dbg_log(rtc, 1, "rtc| min_hgt:%.0f  rqd: %.0f  coll:%.0f = OK",
		    min_hgt, rqd_clr, coll_clr);
		state.tawsb.rtc.ahead_played = B_FALSE;
	}

	mutex_enter(&glob_data.lock);
	glob_data.imp.num_points = num_imp_pts;
	memcpy(&glob_data.imp.points, imp_pts, num_imp_pts * sizeof (*imp_pts));
	mutex_exit(&glob_data.lock);
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
		dbg_log(egpws, 2, "pda|  TOO LOW TERR");
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
	vect3_t my_ecef = geo2ecef_mtr(pos->pos, &wgs84);
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
				vect3_t thr_p =
				    geo2ecef_ft(rwy->ends[i].thr, &wgs84);
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
			if (!pos->on_gnd) {
				if (conf.ra_500 == RA_500_GARMIN)
					sched_sound(SND_RA_500_GARMIN);
				else
					sched_sound(SND_RA_500_BOEING);
			}
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
	double hgt, d_hdg, dist, alt_lim, vs_lim, d_hgt, d_alt;
	double terr_elev, new_vs_hgt, new_vs_alt, vs_hgt, vs_alt, afe;

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
		state.tawsb.ncr.vs_alt = 0;
		state.tawsb.ncr.vs_hgt = 0;
		return;
	}
	if (!state.tawsb.ncr.active) {
		state.tawsb.ncr.vs_alt = 0;
		state.tawsb.ncr.vs_hgt = 0;
		return;
	}

	/* Validate the NCR termination conditions */
	my_ecef = geo2ecef_mtr(pos->pos, &wgs84);
	dep_ecef = geo2ecef_mtr(state.tawsb.ncr.liftoff_pt, &wgs84);

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

	/*
	 * We need to filter these values in gradually to avoid spurious
	 * cautions when we get a sudden short jump up in terrain below us.
	 */
	new_vs_hgt = (hgt - state.tawsb.ncr.prev_hgt) / RUN_INTVAL;
	new_vs_alt = (pos->pos.elev - state.tawsb.ncr.prev_alt) / RUN_INTVAL;
	FILTER_IN(state.tawsb.ncr.vs_hgt, new_vs_hgt, RUN_INTVAL, 2);
	FILTER_IN(state.tawsb.ncr.vs_alt, new_vs_alt, RUN_INTVAL, 2);
	vs_hgt = state.tawsb.ncr.vs_hgt;
	vs_alt = state.tawsb.ncr.vs_alt;

	afe = pos->pos.elev - state.tawsb.ncr.liftoff_pt.elev;
	d_hgt = hgt - MIN(afe, state.tawsb.ncr.max_hgt);
	d_alt = pos->pos.elev - state.tawsb.ncr.max_alt;

	alt_lim = fx_lin_multi(hgt, alt_curve, B_FALSE);
	vs_lim = fx_lin_multi(hgt, vs_curve, B_FALSE);

	dbg_log(egpws, 1, "ncr| vs_hgt:%.1f  vs_alt:%.1f  "
	    "d_hgt:%.0f  d_alt:%.0f  afe:%.0f  alt_lim:%.0f  vs_lim:%.0f",
	    vs_hgt, vs_alt, d_hgt, d_alt, afe, alt_lim, vs_lim);

	if (vs_hgt < vs_lim || d_hgt < alt_lim) {
		/*
		 * If we're descending, advise against descent,
		 * otherwise advise of rising terrain.
		 */
		if (vs_hgt < NCR_SUPRESS_CLB_RATE) {
			if (vs_alt < 0) {
				sched_sound(SND_DONT_SINK);
				dbg_log(egpws, 2, "ncr| DONT SINK");
			} else {
				sched_sound(SND_TOO_LOW_TERR);
				dbg_log(egpws, 2, "ncr| TOO LOW TERR");
			}
			state.adv = MAX(state.adv, EGPWS_ADVISORY_TERRAIN);
		}
	} else if (vs_alt < vs_lim || d_alt < alt_lim) {
		if (vs_hgt < NCR_SUPRESS_CLB_RATE) {
			sched_sound(SND_DONT_SINK);
			dbg_log(egpws, 2, "ncr| TOO LOW TERR");
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
	if (IS_NULL_GEO_POS(pos->pos)) {
		clear_impact();
		return;
	}

	tawsb_dest_dist(pos, dest, &dest_dist, &dest_hgt);
	tawsb_edr(pos);
	tawsb_pda(pos, dest, dest_dist);
	tawsb_500(pos);
	tawsb_ncr(pos);
	tawsb_rtc_iti(pos, d_trk);
}

static void
add_obst_cb(obst_type_t type, geo_pos3_t pos, float agl, obst_light_t light,
    unsigned quant, void *userinfo)
{
	obst_t *obst;
	vect3_t pos_ecef = geo2ecef_mtr(pos, &wgs84);
	const pos_info_t *pos_info = userinfo;

	UNUSED(type);
	UNUSED(light);

	if (vect3_abs(vect3_sub(pos_ecef, pos_info->ecef)) > OBST_DIST_RANGE ||
	    pos_info->pos.elev - (pos.elev + agl) > OBST_HGT_RANGE)
		return;

	obst = calloc(1, sizeof (*obst));
	obst->pos = pos;
	obst->pos_ecef = pos_ecef;
	obst->agl = agl;
	obst->quant = quant;

	list_insert_tail(&state.obstacles, obst);
}

static void
gather_obstacles(egpws_pos_t *pos)
{
	time_t now = time(NULL);
	obst_t *obst;
	int lat = floor(pos->pos.lat), lon = floor(pos->pos.lon);
	pos_info_t pos_info = {
	    .pos = pos->pos, .ecef = geo2ecef_mtr(pos->pos, &wgs84)
	};

	ASSERT_MUTEX_HELD(&state.adv_lock);

	if (state.odb == NULL)
		return;

	if (now - state.obst_refresh_t < OBST_REFRESH_INTVAL)
		return;
	state.obst_refresh_t = now;

	while ((obst = list_remove_head(&state.obstacles)) != NULL)
		free(obst);

	for (int lat_i = -1; lat_i <= 1; lat_i++) {
		for (int lon_i = -1; lon_i <= 1; lon_i++) {
			odb_get_obstacles(state.odb, lat + lat_i, lon + lon_i,
			    add_obst_cb, &pos_info);
		}
	}
}

static void
main_loop(void *unused)
{
	uint64_t now = microclock();

	ASSERT(inited);
	UNUSED(unused);

	thread_set_name("OpenGPWS");

	egpws_boot();

	mutex_enter(&lock);
	while (!main_shutdown) {
		egpws_pos_t pos;
		geo_pos2_t pos_2d;
		double d_trk;
		egpws_arpt_ref_t dest;

		if (init_error)
			goto out;
		mutex_exit(&lock);

		mutex_enter(&glob_data.lock);
		pos = glob_data.pos;
		pos_2d = GEO3_TO_GEO2(pos.pos);
		d_trk = glob_data.d_trk;
		memcpy(&dest, &glob_data.dest, sizeof (dest));
		mutex_exit(&glob_data.lock);

		if (!IS_NULL_GEO_POS(pos_2d))
			load_nearest_airport_tiles(&db, pos_2d);

		mutex_enter(&state.adv_lock);
		state.adv = EGPWS_ADVISORY_NONE;
		gather_obstacles(&pos);
		switch (conf.type) {
		case EGPWS_MK_VIII:
			mk8_mode1(&pos);
			break;
		case EGPWS_TAWS_B:
			tawsb(&pos, &dest, d_trk);
			break;
		case EGPWS_DB_ONLY:
			/* no annunciations in this mode */
			break;
		}
		mutex_exit(&state.adv_lock);

		if (!IS_NULL_GEO_POS(pos_2d))
			unload_distant_airport_tiles(&db, pos_2d);

		mutex_enter(&lock);
out:
		cv_timedwait(&cv, &lock, now + SEC2USEC(RUN_INTVAL));
		now = microclock();
	}
	mutex_exit(&lock);

	egpws_shutdown();
}

void
egpws_init(const egpws_conf_t *acf_conf)
{
	ASSERT(!inited);
	inited = B_TRUE;

	memset(&state, 0, sizeof (state));
	state.tawsb.ncr.liftoff_pt = NULL_GEO_POS3;

	main_shutdown = B_FALSE;
	mutex_init(&lock);
	cv_init(&cv);

	memcpy(&conf, acf_conf, sizeof (conf));

	memset(&glob_data, 0, sizeof (glob_data));
	mutex_init(&glob_data.lock);
	mutex_init(&state.adv_lock);
	glob_data.pos.pos = NULL_GEO_POS3;

	list_create(&state.obstacles, sizeof (obst_t), offsetof(obst_t, node));

	dbg_log(egpws, 3, "EGPWS type: %s",
	    conf.type == EGPWS_MK_VIII ? "MK VIII" : "TAWS-B");

	VERIFY(thread_create(&worker, main_loop, NULL));

	dbg_log(egpws, 1, "init");
}

void
egpws_fini(void)
{
	obst_t *obst;

	dbg_log(egpws, 1, "fini");

	if (!inited)
		return;

	mutex_enter(&lock);
	main_shutdown = B_TRUE;
	cv_broadcast(&cv);
	mutex_exit(&lock);

	thread_join(&worker);

	while ((obst = list_remove_head(&state.obstacles)) != NULL)
		free(obst);

	mutex_destroy(&state.adv_lock);
	mutex_destroy(&glob_data.lock);
	mutex_destroy(&lock);
	cv_destroy(&cv);

	memset(&state, 0, sizeof (state));

	inited = B_FALSE;
}

void
egpws_set_position(egpws_pos_t pos, double now)
{
	double d_t;

	ASSERT(inited);

	mutex_enter(&glob_data.lock);

	d_t = now - glob_data.last_pos_update;
	if (d_t < 0) {
		/*
		 * If the real time clock jumps backwards, eat the new
		 * time and wait for another position update.
		 */
		logMsg("Time skew detected, real time clock jumped back "
		    "%f seconds", ABS(d_t));
		glob_data.last_pos_update = now;
		mutex_exit(&glob_data.lock);
		return;
	}
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
egpws_set_odb(odb_t *odb)
{
	ASSERT(inited);
	if (state.odb != odb) {
		mutex_enter(&state.adv_lock);
		state.odb = odb;
		mutex_exit(&state.adv_lock);
	}
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

	if (!inited)
		return (EGPWS_ADVISORY_NONE);

	mutex_enter(&state.adv_lock);
	adv = state.adv;
	mutex_exit(&state.adv_lock);

	return (adv);
}

void
egpws_get_impact_points(egpws_impact_t *imp)
{
	if (!inited) {
		memset(imp, 0, sizeof (*imp));
		return;
	}

	mutex_enter(&glob_data.lock);
	memcpy(imp, &glob_data.imp, sizeof (*imp));
	mutex_exit(&glob_data.lock);
}

void
egpws_get_obst_impact_pts(egpws_obst_impact_t *imp)
{
	if (!inited) {
		memset(imp, 0, sizeof (*imp));
		return;
	}

	mutex_enter(&state.adv_lock);
	memcpy(imp, &state.tawsb.roc.imp, sizeof (*imp));
	mutex_exit(&state.adv_lock);
}

bool_t
egpws_is_inited(void)
{
	return (inited);
}
