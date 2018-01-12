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

#include "egpws.h"
#include "snd_sys.h"
#include "terr.h"
#include "xplane.h"

#define	RUN_INTVAL	SEC2USEC(0.5)
#define	INF_VS		1e10
#define	RA_LIMIT	FEET2MET(2500)

static bool_t inited = B_FALSE;
static bool_t shutdown = B_FALSE;
static mutex_t lock;
static condvar_t cv;
static thread_t worker;
static airportdb_t db;
static bool_t init_error = B_FALSE;
static egpws_syst_type_t syst_type = EGPWS_MK_VIII;

static mutex_t data_lock;
static egpws_pos_t cur_pos;		/* protected by data_lock */
static egpws_conf_t conf;		/* read-only once inited */
static bool_t flaps_ovrd = B_FALSE;	/* atomic */

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
mk8_mode1(egpws_pos_t pos)
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

	if (isnan(pos.vs) || isnan(pos.ra) || isnan(pos.pos.elev) ||
	    pos.ra > RA_LIMIT)
		return;

	if (conf.jet) {
		caut_vs = fx_lin_multi(pos.ra, caut_rng_jet, B_FALSE);
		warn_vs = fx_lin_multi(pos.ra, warn_rng_jet, B_FALSE);
	} else {
		caut_vs = fx_lin_multi(pos.ra, caut_rng_tp, B_FALSE);
		warn_vs = fx_lin_multi(pos.ra, warn_rng_tp, B_FALSE);
	}

	if (pos.vs > warn_vs) {
		sched_sound(SND_PUP);
	} else if (pos.vs >= caut_vs) {
		double next_caut_vs;
		double caut_vs_20pct = caut_vs * 0.2;

		if (state.mk8.mode1.last_caut_vs < caut_vs) {
			next_caut_vs = caut_vs;
		} else {
			next_caut_vs = ceil(pos.vs / caut_vs_20pct) *
			    caut_vs_20pct;
		}
		if (pos.vs >= next_caut_vs &&
		    next_caut_vs > state.mk8.mode1.last_caut_vs) {
			state.mk8.mode1.last_caut_vs = next_caut_vs;
			sched_sound(SND_SINKRATE);
		}
	} else {
		unsched_sound(SND_SINKRATE);
		state.mk8.mode1.last_caut_vs = 0;
	}
}

static void
tawsb_edr(egpws_pos_t pos)
{
	static const vect2_t caut_rng[] = {
		VECT2(0,		FPM2MPS(-1300)),
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

	if (IS_NULL_GEO_POS(pos.pos) || isnan(pos.vs))
		goto errout;
	terr_elev = terr_get_elev(GEO_POS2(pos.pos.lat, pos.pos.lon));
	if (isnan(terr_elev))
		goto errout;
	hgt = MAX(pos.pos.elev - terr_elev, 0);
	if (hgt > max_hgt)
		goto errout;

	caut_vs = fx_lin_multi(hgt, caut_rng, B_FALSE);
	warn_vs = fx_lin_multi(hgt, warn_rng, B_FALSE);

	if (pos.vs >= warn_vs) {
		state.tawsb.edr.lo_caut = B_TRUE;
		state.tawsb.edr.hi_caut = B_TRUE;
		sched_sound(SND_PUP);
	} else if (pos.vs >= caut_vs) {
		if (!state.tawsb.edr.lo_caut) {
			state.tawsb.edr.lo_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
		} else if (!state.tawsb.edr.hi_caut &&
		    pos.vs > (caut_vs + warn_vs) / 2) {
			state.tawsb.edr.hi_caut = B_TRUE;
			sched_sound(SND_SINKRATE);
		}
	} else {
		state.tawsb.edr.lo_caut = B_FALSE;
		state.tawsb.edr.hi_caut = B_FALSE;
		unsched_sound(SND_PUP);
		unsched_sound(SND_SINKRATE);
	}

errout:
	state.tawsb.edr.hi_caut = B_FALSE;
	state.tawsb.edr.lo_caut = B_FALSE;
	return;
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

		if (init_error)
			goto out;
		mutex_exit(&lock);

		mutex_enter(&data_lock);
		pos = cur_pos;
		mutex_exit(&data_lock);

		if (syst_type == EGPWS_MK_VIII) {
			mk8_mode1(pos);
		} else {
			tawsb_edr(pos);
		}

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
	flaps_ovrd = B_FALSE;

	memset(&state, 0, sizeof (state));
	memset(&cur_pos, 0, sizeof (cur_pos));
	cur_pos.pos = NULL_GEO_POS3;
	mutex_init(&data_lock);

	VERIFY(thread_create(&worker, (int(*)(void *))main_loop, NULL));
}

void
egpws_fini(void)
{
	if (!inited)
		return;

	mutex_enter(&lock);
	shutdown = B_TRUE;
	cv_broadcast(&cv);
	mutex_exit(&lock);

	thread_join(&worker);

	mutex_destroy(&data_lock);
	mutex_destroy(&lock);
	cv_destroy(&cv);

	inited = B_FALSE;
}

void
egpws_set_syst_type(egpws_syst_type_t type)
{
	syst_type = type;
}

void
egpws_set_position(egpws_pos_t pos)
{
	ASSERT(inited);
	mutex_enter(&data_lock);
	cur_pos = pos;
	mutex_exit(&data_lock);
}

void
egpws_set_flaps_ovrd(bool_t flag)
{
	ASSERT(inited);
	flaps_ovrd = flag;
}
