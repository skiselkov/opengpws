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

#ifndef	_SND_SYS_H_
#define	_SND_SYS_H_

#include <acfutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SND_DONT_SINK,		/* Don't sink */
	SND_GLIDESLOPE,		/* Glideslop */
	SND_GOAR_WS_AHEAD,	/* Go around, winshear ahead */
	SND_MON_RAD_DISP,	/* Monitor radar display */
	SND_OBST_AHEAD,		/* Obstacle ahead */
	SND_OBST_AHEAD_PUP,	/* Obstacle ahead, pull up */
	SND_PUP,		/* Pull up */
	SND_SINKRATE,		/* Sinkrate */
	SND_TERR2,		/* Terrain, terrain */
	SND_TERR_AHEAD_PUP,	/* Terrain ahead, pull up */
	SND_TERR,		/* Terrain */
	SND_TOOLOW_FLAPS,	/* Too low, flaps */
	SND_TOOLOW_GEAR,	/* Too low, gear */
	SND_TOOLOW_TERR,	/* Too low, terrain */
	SND_WS_AHEAD,		/* Windshear ahead */
	SND_WS,			/* Windshear */
	NUM_SOUNDS
} snd_id_t;

bool_t snd_sys_init(void);
void snd_sys_fini(void);
void snd_sys_floop_cb(void);
void play_sound(snd_id_t snd);

#ifdef __cplusplus
}
#endif

#endif	/* _SND_SYS_H_ */
