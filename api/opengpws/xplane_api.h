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

#include <acfutils/avl.h>
#include <acfutils/glew.h>
#include <acfutils/geom.h>
#include <acfutils/odb.h>
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
	RA_500_ABV_AFE,	/* Call out "FIVE HUNDRED ABOVE" of closest rwy elev */
	RA_500_GARMIN	/* Call out "FIVE HUNDRED" using the Garmin audio */
} ra_500_type_t;

typedef enum {
	EGPWS_MK_VIII,
	EGPWS_TAWS_B,
	EGPWS_DB_ONLY
} egpws_syst_type_t;

typedef struct {
	int		min_hgt;
	int		max_hgt;
	float		rgba[4];
} egpws_terr_color_t;

typedef struct {
	bool_t		jet;		/* aircraft is a turbojet */
	bool_t		ifr_only;	/* only check IFR airports */
	appr_min_t	appr_min;	/* how to call out approaching mins */
	ra_call_mask_t	ra_calls;	/* how/if to annunciate RA altitudes */
	ra_500_type_t	ra_500;		/* how to annunciate "FIVE HUNDRED" */
	egpws_syst_type_t	type;			/* system type */
	const egpws_terr_color_t terr_colors[4];	/* color patterns */
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
	bool_t		do_draw;
	geo_pos3_t	position;
	double		rotation;
	double		scale;
	vect2_t		offset;
	vect2_t		disp_sz;	/* pixel-size of target */
} egpws_render_t;

typedef enum {
	EGPWS_ADVISORY_NONE,
	EGPWS_ADVISORY_SINKRATE_DR,	/* Caused by descent rate */
	EGPWS_ADVISORY_PULL_UP_DR,	/* Caused by descent rate */
	EGPWS_ADVISORY_OBSTACLE,	/* Caused by proximity to obstacles */
	EGPWS_ADVISORY_TERRAIN,		/* Caused by proximity to terrain */
	EGPWS_ADVISORY_PULL_UP_OBSTACLE,/* Caused by proximity to terrain */
	EGPWS_ADVISORY_PULL_UP_TERRAIN	/* Caused by proximity to terrain */
} egpws_advisory_t;

enum { EGPWS_MAX_NUM_IMP_PTS = 32 };
typedef struct {
	int		num_points;
	geo_pos3_t	points[EGPWS_MAX_NUM_IMP_PTS];
} egpws_impact_t;

enum { EGPWS_MAX_NUM_OBST_IMP_PTS = 128 };
typedef struct {
	int		num_points;
	geo_pos3_t	points[EGPWS_MAX_NUM_OBST_IMP_PTS];
	bool_t		is_warn[EGPWS_MAX_NUM_OBST_IMP_PTS];
} egpws_obst_impact_t;

typedef struct {
	unsigned		num_pts;
	geo_pos2_t		*in_pts;
	double			*out_elev;
	vect3_t			*out_norm;
	double			*out_water;
	bool_t			filter_lin;
} egpws_terr_probe_t;

typedef struct {
	void (*set_state)(const egpws_conf_t *conf);
	void (*set_flaps_ovrd)(bool_t flag);
	void (*set_pos_ok)(bool_t flag);
	void (*set_ra_ok)(bool_t flag);
	void (*set_on_gnd_ok)(bool_t flag);
	void (*set_dest)(const egpws_arpt_ref_t *arpt);
	void (*set_nav_on)(bool_t nav1_on, bool_t nav2_on);
	void (*set_ranges)(const egpws_range_t *ranges);
	void (*terr_render)(const egpws_render_t *render);
	egpws_advisory_t (*get_advisory)(void);
	void (*set_sound_inh)(bool_t flag);
	void (*set_sound_supp)(bool_t flag);
	void (*get_impact_pts)(egpws_impact_t *imp);
	void (*get_obst_impact_pts)(egpws_obst_impact_t *imp);
	void (*terr_probe)(egpws_terr_probe_t *probe);
	bool_t (*terr_have_data)(geo_pos2_t pos, double *tile_load_res);
	/* Debugging support */
	void (*reload_gl_progs)(void);
	bool_t (*is_inited)(void);
	void (*set_odb)(odb_t *odb);
} egpws_intf_t;

enum {
	EGPWS_GET_INTF = 0x100000		/* egpws_intf_t * param */
};

#ifdef __cplusplus
}
#endif

#endif	/* _OPENGPWS_XPLANE_API_H_ */
