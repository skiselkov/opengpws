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

typedef enum {
	APPR_MIN_NONE,		/* no callout of approaching minimums */
	APPR_MIN_BOEING,	/* "Approaching minimums" for MDA & DH */
	APPR_MIN_BOEING_WITH_DH,/* ^^^ + "Approaching decision height" for DH */
	APPR_MIN_AIRBUS		/* "Hundred above" */
} appr_min_t;

/*
 * Optional RA callouts. The '500' callout is mandatory. See ra_500_type_t
 * for configuration of the 500 foot callout.
 */
typedef enum {
	RA_CALLOUT_2500 =	1 << 0,
	RA_CALLOUT_1000 =	1 << 1,
	RA_CALLOUT_400 =	1 << 2,
	RA_CALLOUT_300 =	1 << 3,
	RA_CALLOUT_200 =	1 << 4,
	RA_CALLOUT_100 =	1 << 5,
	RA_CALLOUT_50 =		1 << 6,
	RA_CALLOUT_40 =		1 << 7,
	RA_CALLOUT_30 =		1 << 8,
	RA_CALLOUT_20 =		1 << 9,
	RA_CALLOUT_10 =		1 << 10,
	RA_CALLOUT_5 =		1 << 11
} ra_call_mask_t;

typedef enum {
	RA_500_HARD,	/* Call out "FIVE HUNDRED" RA once during appr */
	RA_500_SMART,	/* Call out "FIVE HUNDRED" on non-precision appr */
	RA_500_AFE,	/* Call out "FIVE HUNDRED" of closest rwy elev */
	RA_500_ABV_AFE	/* Call out "FIVE HUNDRED ABOVE" of closest rwy elev */
} ra_500_type_t;

typedef struct {
	bool_t		jet;	/* aircraft is a turbojet */
	appr_min_t	appr_min; /* how to call out approaching minimums */
	ra_call_mask_t	ra_calls; /* how/if to annunciate RA altitudes */
	ra_500_type_t	ra_500;	/* how to annunciate "FIVE HUNDRED" */
} egpws_conf_t;

typedef struct {
	geo_pos3_t	pos;	/* lat x lon (degrees) x elev (meters) */
	double		trk;	/* true track, degrees */
	double		gs;	/* groundspeed, m/s */
	double		asi;	/* indicated airspeed, m/s */
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
