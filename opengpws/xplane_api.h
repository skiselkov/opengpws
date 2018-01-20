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

#include <GL/glew.h>

#include <acfutils/geom.h>
#include <acfutils/avl.h>
#include <acfutils/thread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	OPENGPWS_PLUGIN_SIG	"skiselkov.opengpws"

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

typedef enum {
	EGPWS_MK_VIII,
	EGPWS_TAWS_B
} egpws_syst_type_t;

typedef struct {
	int		min_hgt;
	int		max_hgt;
	uint32_t	color_rgba;
} egpws_terr_color_t;

typedef struct {
	bool_t		jet;		/* aircraft is a turbojet */
	appr_min_t	appr_min;	/* how to call out approaching mins */
	ra_call_mask_t	ra_calls;	/* how/if to annunciate RA altitudes */
	ra_500_type_t	ra_500;		/* how to annunciate "FIVE HUNDRED" */
	egpws_syst_type_t	type;		/* system type */
	const egpws_terr_color_t *terr_colors;	/* color patterns */
} egpws_conf_t;

typedef struct {
	geo_pos3_t	pos;	/* lat x lon (degrees) x true elev (meters) */
	double		trk;	/* true track, degrees */
	double		gs;	/* groundspeed, m/s */
	double		asi;	/* indicated airspeed, m/s */
	double		vs;	/* vertical speed, m/s */
	double		ra;	/* radio altitude, meters */
	bool_t		on_gnd;	/* on-ground gear switch */
	double		loc_dev;/* localizer deviation, dots, pos right */
	double		gs_dev;	/* glideslope deviation, dots, pos up */
} egpws_pos_t;

typedef struct {
	double	range;		/* range in meters */
	double	resolution;	/* resolution in meters per pixel */
} egpws_range_t;

typedef struct {
	char		icao[8];	/* airport ICAO code */
	geo_pos3_t	pos;		/* airport geographic position */
} egpws_arpt_ref_t;

typedef struct {
	int			lat;
	int			lon;
	GLuint			tex;

	/* OpenGPWS-internal */
	bool_t			dirty;
	bool_t			remove;

	/* modified only from painter thread */
	unsigned		pix_width;
	unsigned		pix_height;
	uint32_t		*pixels;

	avl_node_t		node;
} egpws_terr_tile_t;

typedef struct {
	avl_tree_t		tiles;
	mutex_t			lock;
} egpws_terr_tile_set_t;

typedef enum {
	EGPWS_ADVISORY_NONE,
	EGPWS_ADVISORY_TERRAIN,
	EGPWS_ADVISORY_PULL_UP
} egpws_advisory_t;

enum {
	EGPWS_SET_STATE =	0x100000,	/* bool_t param */
	EGPWS_SET_FLAPS_OVRD,			/* bool_t param */
	EGPWS_SET_POS_OK,			/* bool_t param */
	EGPWS_SET_RA_OK,			/* bool_t param */
	EGPWS_SET_ON_GND_OK,			/* bool_t param */
	EGPWS_SET_DEST,				/* arpt_ref_t * param */
	EGPWS_SET_NAV1_ON,			/* bool_t param */
	EGPWS_SET_NAV2_ON,			/* bool_t param */
	EGPWS_SET_RANGES,			/* egpws_range_t ptr param */
	EGPWS_GET_TERR_TILE_SET,		/* egpws_terr_tile_set_t ** */
	EGPWS_GET_ADVISORY,			/* egpws_advisory_t * param */
	EGPWS_SET_SOUND_INH			/* bool_t param */
};

#ifdef __cplusplus
}
#endif

#endif	/* _OPENGPWS_XPLANE_API_H_ */
