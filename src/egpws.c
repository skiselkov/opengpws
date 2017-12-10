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
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "egpws.h"
#include "xplane.h"

#define	RUN_INTVAL	SEC2USEC(1)

static bool_t inited = B_FALSE;
static bool_t shutdown = B_FALSE;
static mutex_t lock;
static condvar_t cv;
static thread_t worker;
static airportdb_t db;
static bool_t init_error = B_FALSE;

static mutex_t data_lock;
static egpws_pos_t cur_pos;
static egpws_acf_desc_t acf;
static bool_t flaps_ovrd = B_FALSE;

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
mode1(egpws_pos_t pos)
{
	UNUSED(pos);
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

		mode1(pos);

		mutex_enter(&lock);
out:
		cv_timedwait(&cv, &lock, now + RUN_INTVAL);
		now = microclock();
	}
	mutex_exit(&lock);

	egpws_shutdown();
}

void
egpws_init(egpws_acf_desc_t acf_desc)
{
	ASSERT(!inited);

	inited = B_TRUE;
	shutdown = B_FALSE;
	mutex_init(&lock);
	cv_init(&cv);
	acf = acf_desc;
	flaps_ovrd = B_FALSE;

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
