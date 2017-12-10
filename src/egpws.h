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

#ifndef	_EGPWS_H_
#define	_EGPWS_H_

#include <acfutils/geom.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	geo_pos3_t	pos;
	double		trk;
	double		gs;
	double		vs;
} egpws_pos_t;

void egpws_init(void);
void egpws_fini(void);
void egpws_set_position(egpws_pos_t pos);

#ifdef __cplusplus
}
#endif

#endif	/* _EGPWS_H_ */
