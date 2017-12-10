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
	{ .filename = "pull_up.opus" },			/* SND_PUP */
	{ .filename = "terr2.opus" },			/* SND_TERR2 */
	{ .filename = "terr_ahead_pull_up.opus" },	/* SND_TERR_AHEAD_PUP */
	{ .filename = "obst_ahead_pull_up.opus" },	/* SND_OBST_AHEAD_PUP */
	{ .filename = "ws.opus" },			/* SND_WS */
	{ .filename = "go_around_ws_ahead.opus" },	/* SND_GOAR_WS_AHEAD, */
	{ .filename = "ws_ahead.opus" },		/* SND_WS_AHEAD */
	{ .filename = "terr.opus" },			/* SND_TERR */
	{ .filename = "terr.opus" },			/* SND_MINS */
	{ .filename = "terr_ahead.opus" },		/* SND_TERR_AHEAD */
	{ .filename = "obst_ahead.opus" },		/* SND_OBST_AHEAD */
	{ .filename = "too_low_terr.opus" },		/* SND_TOOLOW_TERR */
	{ .filename = "terr.opus" },			/* SND_RA_2500 */
	{ .filename = "terr.opus" },			/* SND_RA_1000 */
	{ .filename = "terr.opus" },			/* SND_RA_500 */
	{ .filename = "terr.opus" },			/* SND_RA_400 */
	{ .filename = "terr.opus" },			/* SND_RA_300 */
	{ .filename = "terr.opus" },			/* SND_RA_200 */
	{ .filename = "terr.opus" },			/* SND_RA_100 */
	{ .filename = "terr.opus" },			/* SND_RA_50 */
	{ .filename = "terr.opus" },			/* SND_RA_40 */
	{ .filename = "terr.opus" },			/* SND_RA_30 */
	{ .filename = "terr.opus" },			/* SND_RA_20 */
	{ .filename = "terr.opus" },			/* SND_RA_10 */
	{ .filename = "terr.opus" },			/* SND_RA_5 */
	{ .filename = "too_low_gear.opus" },		/* SND_TOOLOW_GEAR */
	{ .filename = "too_low_flaps.opus" },		/* SND_TOOLOW_FLAPS */
	{ .filename = "sinkrate.opus" },		/* SND_SINKRATE */
	{ .filename = "dont_sink.opus" },		/* SND_DONT_SINK */
	{ .filename = "gs.opus" },			/* SND_GLIDESLOPE */
	{ .filename = "terr.opus" }			/* SND_BANK_ANGLE2 */
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
	bool_t higher_playing = B_FALSE;

	ASSERT(inited);

	mutex_enter(&lock);
	for (snd_id_t snd_id = 0; snd_id < NUM_SOUNDS; snd_id++) {
		snd_t *snd = &sounds[snd_id];

		/*
		 * If playback was requested and a higher priority is not yet
		 * playing, play our message.
		 */
		if (snd->play && !higher_playing) {
			if (!wav_is_playing(snd->wav))
				wav_play(snd->wav);
			snd->play = B_FALSE;
			higher_playing = B_TRUE;
		} else if (higher_playing && wav_is_playing(snd->wav)) {
			/* Stop lower priority messages */
			wav_stop(snd->wav);
			snd->play = B_FALSE;
		}
	}
	mutex_exit(&lock);
}

void
sched_sound(snd_id_t snd)
{
	ASSERT(inited);
	ASSERT3U(snd, <, NUM_SOUNDS);

	mutex_enter(&lock);
	sounds[snd].play = B_TRUE;
	mutex_exit(&lock);
}

void
unsched_sound(snd_id_t snd)
{
	ASSERT(inited);
	ASSERT3U(snd, <, NUM_SOUNDS);

	mutex_enter(&lock);
	sounds[snd].play = B_FALSE;
	mutex_exit(&lock);
}
