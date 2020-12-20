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
#include <acfutils/dr.h>
#include <acfutils/wav.h>
#include <acfutils/thread.h>

#include "snd_sys.h"
#include "xplane.h"

typedef struct {
	const char	*filename;
	double		base_gain;
	double		gain;
	wav_t		*wav;
	bool_t		play;
} snd_t;

static bool_t inited = B_FALSE;
static alc_t *alc = NULL;
static mutex_t lock;
static bool_t sound_supp = B_FALSE;
static bool_t sound_inh = B_FALSE;
static snd_t sounds[NUM_SOUNDS] = {
	/* SND_TERR_AHEAD_PUP */
	{ .filename = "terr_ahead_pull_up.opus", .base_gain = 1.0 },
	/* SND_OBST_AHEAD_PUP */
	{ .filename = "obst_ahead_pull_up.opus", .base_gain = 1.0 },
	/* SND_PUP */
	{ .filename = "pull_up.opus", .base_gain = 1.0 },
	/* SND_TERR2 */
	{ .filename = "terr2.opus", .base_gain = 1.0 },
	/* SND_WS */
	{ .filename = "ws.opus", .base_gain = 1.0 },
	/* SND_GOAR_WS_AHEAD, */
	{ .filename = "go_around_ws_ahead.opus", .base_gain = 1.0 },
	/* SND_WS_AHEAD */
	{ .filename = "ws_ahead.opus", .base_gain = 1.0 },
	/* SND_TERR */
	{ .filename = "terr.opus", .base_gain = 1.0 },
	/* SND_MINS */
	{ .filename = "terr.opus", .base_gain = 1.0 },
	/* SND_TERR_AHEAD */
	{ .filename = "terr_ahead.opus", .base_gain = 1.0 },
	/* SND_OBST_AHEAD */
	{ .filename = "obst_ahead.opus", .base_gain = 1.0 },
	/* SND_TOO_LOW_TERR */
	{ .filename = "too_low_terr.opus", .base_gain = 1.0 },
	/* SND_RA_10_BOEING */
	{ .filename = "ra/boeing/10.opus", .base_gain = 0.35 },
	/* SND_RA_20_BOEING */
	{ .filename = "ra/boeing/20.opus", .base_gain = 0.35 },
	/* SND_RA_30_BOEING */
	{ .filename = "ra/boeing/30.opus", .base_gain = 0.35 },
	/* SND_RA_40_BOEING */
	{ .filename = "ra/boeing/40.opus", .base_gain = 0.35 },
	/* SND_RA_50_BOEING */
	{ .filename = "ra/boeing/50.opus", .base_gain = 0.35 },
	/* SND_RA_100_BOEING */
	{ .filename = "ra/boeing/100.opus", .base_gain = 0.35 },
	/* SND_RA_200_BOEING */
	{ .filename = "ra/boeing/200.opus", .base_gain = 0.35 },
	/* SND_RA_300_BOEING */
	{ .filename = "ra/boeing/300.opus", .base_gain = 0.35 },
	/* SND_RA_400_BOEING */
	{ .filename = "ra/boeing/400.opus", .base_gain = 0.35 },
	/* SND_RA_500_BOEING */
	{ .filename = "ra/boeing/500.opus", .base_gain = 0.35 },
	/* SND_RA_500_GARMIN */
	{ .filename = "ra/garmin/500.opus", .base_gain = 1 },
	/* SND_RA_1000_BOEING */
	{ .filename = "ra/boeing/1000.opus", .base_gain = 0.35 },
	/* SND_RA_2500_BOEING */
	{ .filename = "ra/boeing/2500.opus", .base_gain = 0.35 },
	/* SND_TOO_LOW_GEAR */
	{ .filename = "too_low_gear.opus", .base_gain = 1.0 },
	/* SND_TOO_LOW_FLAPS */
	{ .filename = "too_low_flaps.opus", .base_gain = 1.0 },
	/* SND_SINKRATE */
	{ .filename = "sinkrate.opus", .base_gain = 1.0 },
	/* SND_DONT_SINK */
	{ .filename = "dont_sink.opus", .base_gain = 1.0 },
	/* SND_GLIDESLOPE */
	{ .filename = "gs.opus", .base_gain = 1.0 },
	/* SND_BANK_ANGLE2 */
	{ .filename = "terr.opus", .base_gain = 1.0 }
};

struct {
	dr_t	view_is_ext;
	dr_t	sound_on;
	dr_t	paused;
	dr_t	int_volume;
	dr_t	warn_volume;
} drs;

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
	if (alc == NULL)
		goto errout;
	sound_inh = B_FALSE;

	for (snd_id_t snd = 0; snd < NUM_SOUNDS; snd++) {
		char *filename = mkpathname(get_plugindir(), "data",
		    sounds[snd].filename, NULL);

		sounds[snd].wav = wav_load(filename, sounds[snd].filename, alc);
		free(filename);

		if (sounds[snd].wav == NULL)
			goto errout;
		sounds[snd].gain = -1;
	}

	fdr_find(&drs.view_is_ext, "sim/graphics/view/view_is_external");
	fdr_find(&drs.sound_on, "sim/operation/sound/sound_on");
	fdr_find(&drs.int_volume, "sim/operation/sound/interior_volume_ratio");
	fdr_find(&drs.warn_volume, "sim/operation/sound/warning_volume_ratio");
	fdr_find(&drs.paused, "sim/time/paused");

	return (B_TRUE);
errout:
	snd_sys_fini();
	return (B_FALSE);
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
	if (alc != NULL) {
		openal_fini(alc);
		alc = NULL;
	}
	mutex_destroy(&lock);

	inited = B_FALSE;
}

void
snd_sys_floop_cb(void)
{
	snd_id_t highest_playing = NUM_SOUNDS;
	double gain;

	ASSERT(inited);

	if (dr_geti(&drs.view_is_ext) != 0 || dr_geti(&drs.sound_on) == 0 ||
	    dr_geti(&drs.paused) != 0 || sound_inh)
		gain = 0;
	else
		gain = dr_getf(&drs.int_volume) * dr_getf(&drs.warn_volume);

	mutex_enter(&lock);
	for (snd_id_t snd_id = 0; snd_id < NUM_SOUNDS; snd_id++) {
		snd_t *snd = &sounds[snd_id];

		/* Audio suppressed, stop the WAV and reschedule for later */
		if (sound_supp) {
			if (wav_is_playing(snd->wav)) {
				wav_stop(snd->wav);
				snd->play = B_TRUE;
			}
			continue;
		}

		if (wav_is_playing(snd->wav) && snd_id < highest_playing)
			highest_playing = snd_id;

		if (snd->gain != gain * snd->base_gain) {
			wav_set_gain(snd->wav, gain * snd->base_gain);
			snd->gain = gain * snd->base_gain;
		}

		/*
		 * If playback was requested and a higher priority is not yet
		 * playing, play our message.
		 */
		if (snd->play && highest_playing > snd_id) {
			if (!wav_is_playing(snd->wav))
				wav_play(snd->wav);
			snd->play = B_FALSE;
			highest_playing = snd_id;
		} else if (highest_playing < snd_id &&
		    wav_is_playing(snd->wav)) {
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

void
snd_sys_set_inh(bool_t flag)
{
	sound_inh = flag;
}

void
snd_sys_set_supp(bool_t flag)
{
	sound_supp = flag;
}
