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

#include <acfutils/assert.h>
#include <acfutils/wav.h>
#include <acfutils/thread.h>

#include "snd_sys.h"
#include "xplane.h"

typedef struct {
	const char	*filename;
	wav_t		*wav;
	bool_t		play;
} snd_t;

static bool_t inited = B_FALSE;
static alc_t *alc = NULL;
static mutex_t lock;
static snd_t sounds[NUM_SOUNDS] = {
	{ .filename = "dont_sink.opus" },
	{ .filename = "glideslope.opus" },
	{ .filename = "go_around_windshear_ahead.opus" },
	{ .filename = "monitor_radar_display.opus" },
	{ .filename = "obstacle_ahead.opus" },
	{ .filename = "obstacle_ahead_pull_up.opus" },
	{ .filename = "pull_up.opus" },
	{ .filename = "sinkrate.opus" },
	{ .filename = "terrain2.opus" },
	{ .filename = "terrain_ahead_pull_up.opus" },
	{ .filename = "terrain.opus" },
	{ .filename = "too_low_flaps.opus" },
	{ .filename = "too_low_gear.opus" },
	{ .filename = "too_low_terrain.opus" },
	{ .filename = "windshear_ahead.opus" },
	{ .filename = "windshear.opus" }
};

bool_t
snd_sys_init(void)
{
	ASSERT(!inited);

	for (snd_id_t snd = 0; snd < NUM_SOUNDS; snd++) {
		sounds[snd].wav = NULL;
		sounds[snd].play = B_FALSE;
	}

	inited = B_TRUE;

	mutex_init(&lock);
	alc = openal_init(NULL, B_FALSE);

	for (snd_id_t snd = 0; snd < NUM_SOUNDS; snd++) {
		char *filename = mkpathname(get_plugindir(), "data",
		    sounds[snd].filename, NULL);

		sounds[snd].wav = wav_load(filename, sounds[snd].filename, alc);
		free(filename);

		if (sounds[snd].wav == NULL) {
			snd_sys_fini();
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

void
snd_sys_fini(void)
{
	if (!inited)
		return;

	for (snd_id_t snd = 0; snd < NUM_SOUNDS; snd++) {
		if (sounds[snd].wav != NULL) {
			wav_free(sounds[snd].wav);
			sounds[snd].wav = NULL;
		}
	}

	openal_fini(alc);
	mutex_destroy(&lock);

	inited = B_FALSE;
}

void
snd_sys_floop_cb(void)
{
	ASSERT(inited);

	mutex_enter(&lock);
	for (snd_id_t snd = 0; snd < NUM_SOUNDS; snd++) {
		if (sounds[snd].play)
			wav_play(sounds[snd].wav);
	}
	mutex_exit(&lock);
}

void
play_sound(snd_id_t snd)
{
	ASSERT(inited);

	mutex_enter(&lock);
	sounds[snd].play = B_TRUE;
	mutex_exit(&lock);
}
