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
	SND_SINKRATE2,
	SND_PULL_UP,
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
