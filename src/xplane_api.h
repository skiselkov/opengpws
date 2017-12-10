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

#ifndef	_OPENGPWS_XPLANE_API_H_
#define	_OPENGPWS_XPLANE_API_H_

#include <acfutils/geom.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	bool_t		jet;	/* aircraft is a turbojet */
} egpws_acf_desc_t;

typedef struct {
	geo_pos3_t	pos;	/* lat x lon (degrees) x elev (meters) */
	double		trk;	/* true track, degrees */
	double		gs;	/* groundspeed, m/s */
	double		vs;	/* vertical speed, m/s */
	double		ra;	/* radio altitude, meters */
	bool_t		on_gnd;	/* on-ground gear switch */
} egpws_pos_t;

typedef enum {
	EGPWS_SET_STATE =	0x100000,	/* egpws_tcf_tesc_t * param */
	EGPWS_SET_FLAPS_OVRD,			/* bool_t param */
	EGPWS_SET_POS_OK,			/* bool_t param */
	EGPWS_SET_RA_OK,			/* bool_t param */
	EGPWS_SET_ON_GND_OK,			/* bool_t param */
	EGPWS_SET_DEST				/* char * param, ICAO ID */
} egpws_msg_t;

#ifdef __cplusplus
}
#endif

#endif	/* _OPENGPWS_XPLANE_API_H_ */
