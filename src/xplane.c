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

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/perf.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "egpws.h"
#include "snd_sys.h"
#include "xplane_api.h"

#define	PLUGIN_NAME		"OpenGPWS by Saso Kiselkov"
#define	PLUGIN_SIG		"skiselkov.opengpws"
#define	PLUGIN_DESCRIPTION	"An open-source Mk.VIII EGPWS simulation"

#define	SENSOR_INTVAL		0.2	/* seconds */

char			xpdir[512];
char			plugindir[512];

int			xp_ver, xplm_ver;
XPLMHostApplicationID	host_id;
static bool_t		booted = B_FALSE;
static bool_t		pos_ok = B_TRUE;	/* position acquisition OK? */
static bool_t		ra_ok = B_TRUE;		/* radio altimeter OK? */
static bool_t		on_gnd_ok = B_TRUE;	/* ground squat switch OK? */

static dr_t		lat, lon, elev;
static dr_t		trk;
static dr_t		gs;
static dr_t		vs_ft;
static dr_t		ra_ft;
static dr_t		on_gnd;

static float
sensor_cb(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (booted) {
		egpws_pos_t pos;

		if (pos_ok) {
			pos.pos = GEO_POS3(dr_getf(&lat), dr_getf(&lon),
			    dr_getf(&elev));
			pos.trk = dr_getf(&trk);
			pos.gs = dr_getf(&gs);
			pos.vs = FEET2MET(dr_getf(&vs_ft));
		} else {
			pos.pos = NULL_GEO_POS3;
			pos.trk = NAN;
			pos.gs = NAN;
			pos.vs = NAN;
		}
		if (on_gnd_ok) {
			int on_ground[3];
			VERIFY3S(dr_getvi(&on_gnd, on_ground, 0, 3), ==, 3);
			pos.on_gnd = B_FALSE;
			for (int i = 0; i < 3; i++)
				pos.on_gnd |= on_ground[i];
		} else {
			pos.on_gnd = B_FALSE;
		}
		if (ra_ok)
			pos.ra = FEET2MET(dr_getf(&ra_ft));
		else
			pos.ra = NAN;
		egpws_set_position(pos);
	}

	return (SENSOR_INTVAL);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;

	log_init(XPLMDebugString, "OpenGPWS");
	crc64_init();
	crc64_srand(microclock());
	logMsg("This is OpenGPWS (" PLUGIN_VERSION ") libacfutils-%s",
	    libacfutils_version);

	/* Always use Unix-native paths on the Mac! */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

	XPLMGetSystemPath(xpdir);
	XPLMGetPluginInfo(XPLMGetMyID(), NULL, plugindir, NULL, NULL);

#if	IBM
	fix_pathsep(xpdir);
	fix_pathsep(plugindir);
#endif	/* IBM */

	/* cut off the trailing path component (our filename) */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL)
		*p = '\0';
	/* cut off an optional '32' or '64' trailing component */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL) {
		if (strcmp(p + 1, "64") == 0 || strcmp(p + 1, "32") == 0)
			*p = '\0';
	}

	/*
	 * Now we strip a leading xpdir from plugindir, so that now plugindir
	 * will be relative to X-Plane's root directory.
	 */
	if (strstr(plugindir, xpdir) == plugindir) {
		int xpdir_len = strlen(xpdir);
		int plugindir_len = strlen(plugindir);
		memmove(plugindir, &plugindir[xpdir_len],
		    plugindir_len - xpdir_len + 1);
	}

	strcpy(name, PLUGIN_NAME);
	strcpy(sig, PLUGIN_SIG);
	strcpy(desc, PLUGIN_DESCRIPTION);

	XPLMGetVersions(&xp_ver, &xplm_ver, &host_id);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
}

PLUGIN_API int
XPluginEnable(void)
{
	if (!snd_sys_init())
		return (0);

	fdr_find(&lat, "sim/flightmodel/position/latitude");
	fdr_find(&lon, "sim/flightmodel/position/longitude");
	fdr_find(&elev, "sim/flightmodel/position/elevation");
	fdr_find(&trk, "sim/flightmodel/position/hpath");
	fdr_find(&gs, "sim/flightmodel/position/groundspeed");
	fdr_find(&vs_ft, "sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
	fdr_find(&ra_ft,
	    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");
	fdr_find(&on_gnd, "sim/flightmodel2/gear/on_ground");

	XPLMRegisterFlightLoopCallback(sensor_cb, SENSOR_INTVAL, NULL);

	return (1);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(sensor_cb, NULL);

	egpws_fini();
	snd_sys_fini();
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);

	switch (msg) {
	case EGPWS_SET_STATE:
		if (param != NULL && !booted) {
			egpws_init(*(egpws_acf_desc_t *)param);
			booted = B_TRUE;
			pos_ok = B_TRUE;
			ra_ok = B_TRUE;
			on_gnd_ok = B_TRUE;
		} else if (param == NULL && booted) {
			egpws_fini();
			booted = B_FALSE;
		}
		break;
	case EGPWS_SET_FLAPS_OVRD:
		if (booted)
			egpws_set_flaps_ovrd((uintptr_t)param);
		break;
	case EGPWS_SET_POS_OK:
		pos_ok = (uintptr_t)param;
		break;
	case EGPWS_SET_RA_OK:
		ra_ok = (uintptr_t)param;
		break;
	case EGPWS_SET_ON_GND_OK:
		on_gnd_ok = (uintptr_t)param;
		break;
	case EGPWS_SET_DEST:
		if (booted)
			egpws_set_dest(param);
		break;
	}
}

const char *
get_xpdir(void)
{
	return (xpdir);
}

const char *get_plugindir(void)
{
	return (plugindir);
}
