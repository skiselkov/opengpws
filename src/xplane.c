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
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */

#include <errno.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMDisplay.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/glew.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/perf.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "dbg_log.h"
#include "egpws.h"
#include "snd_sys.h"
#include "terr.h"
#include <opengpws/xplane_api.h>

#define	PLUGIN_NAME		"OpenGPWS by Saso Kiselkov"
#define	PLUGIN_DESCRIPTION \
	"An open-source Mk.VIII EGPWS & TAWS-B simulation"

#define	SENSOR_INTVAL		0.2	/* seconds */

static void set_state(const egpws_conf_t *conf);
static void set_pos_ok(bool_t flag);
static void set_ra_ok(bool_t flag);
static void set_on_gnd_ok(bool_t flag);
static void set_nav_on(bool_t nav1_on, bool_t nav2_on);

char			xpdir[512];
char			plugindir[512];

int			xp_ver, xplm_ver;
XPLMHostApplicationID	host_id;
static bool_t		booted = B_FALSE;
static bool_t		pos_ok = B_TRUE;	/* position acquisition OK? */
static bool_t		ra_ok = B_TRUE;		/* radio altimeter OK? */
static bool_t		on_gnd_ok = B_TRUE;	/* ground squat switch OK? */

static dr_t	lat, lon, elev;
static dr_t	trk;
static dr_t	asi_kts;
static dr_t	asi_fail;
static dr_t	gs;
static dr_t	vs_ft;
static dr_t	vs_fail;
static dr_t	ra_ft;
static dr_t	on_gnd;
static dr_t	loc_fail, gs_fail;

static struct {
	dr_t	freq;
	dr_t	type;
	dr_t	id;
	dr_t	hdef;
	dr_t	vdef;
	dr_t	gs_flag;
	bool_t	on;
} nav1, nav2;

static egpws_intf_t egpws_intf = {
	.set_state = set_state,
	.set_flaps_ovrd = egpws_set_flaps_ovrd,
	.set_pos_ok = set_pos_ok,
	.set_ra_ok = set_ra_ok,
	.set_on_gnd_ok = set_on_gnd_ok,
	.set_dest = egpws_set_dest,
	.set_nav_on = set_nav_on,
	.set_ranges = terr_set_ranges,
	.terr_render = terr_render,
	.terr_render_get_fpp = terr_render_get_fpp,
	.get_advisory = egpws_get_advisory,
	.set_sound_inh = snd_sys_set_inh,
	.set_sound_supp = snd_sys_set_supp,
	.get_impact_pts = egpws_get_impact_points,
	.get_obst_impact_pts = egpws_get_obst_impact_pts,
	.terr_probe = terr_probe,
	.terr_have_data = terr_have_data,
	.reload_gl_progs = terr_reload_gl_progs,
	.is_inited = egpws_is_inited,
	.set_odb = egpws_set_odb
};

#ifndef	NOAUDIO
static float
sound_cb(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	snd_sys_floop_cb();

	return (-1);
}
#endif	/* !defined(NOAUDIO) */

static float
sensor_cb(float elapsed, float elapsed2, int counter, void *refcon)
{
	egpws_pos_t pos;
	geo_pos3_t terr_pos;

	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	if (!booted)
		return (SENSOR_INTVAL);

	if (pos_ok) {
		pos.pos = GEO_POS3(dr_getf(&lat), dr_getf(&lon),
		    dr_getf(&elev));
		pos.trk = dr_getf(&trk);
		pos.gs = dr_getf(&gs);
		terr_pos = pos.pos;
	} else {
		pos.pos = NULL_GEO_POS3;
		pos.trk = NAN;
		pos.gs = NAN;
		pos.vs = NAN;
		terr_pos = NULL_GEO_POS3;
	}
	if (dr_geti(&asi_fail) != 6)
		pos.asi = KT2MPS(dr_getf(&asi_kts));
	else
		pos.asi = NAN;
	if (dr_geti(&vs_fail) != 6)
		pos.vs = FPM2MPS(dr_getf(&vs_ft));
	else
		pos.vs = NAN;
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

	/* Using real-time here to allow functioning in replay mode */
	egpws_set_position(pos, USEC2SEC(microclock()));
	terr_set_pos(terr_pos);

	return (SENSOR_INTVAL);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *p;
	char *confpath;
	conf_t *conf = NULL;
	GLenum err;

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
	/*
	 * Cut off an optional '32' or '64' trailing component. Please note
	 * that XPLM 3.0 now supports OS-specific suffixes, so clamp those
	 * away as well.
	 */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL) {
		if (strcmp(p + 1, "64") == 0 || strcmp(p + 1, "32") == 0 ||
		    strcmp(p + 1, "win_x64") == 0 ||
		    strcmp(p + 1, "mac_x64") == 0 ||
		    strcmp(p + 1, "lin_x64") == 0)
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
	strcpy(sig, OPENGPWS_PLUGIN_SIG);
	strcpy(desc, PLUGIN_DESCRIPTION);

	XPLMGetVersions(&xp_ver, &xplm_ver, &host_id);

	err = glewInit();
	if (err != GLEW_OK) {
		/* Problem: glewInit failed, something is seriously wrong. */
		logMsg("FATAL ERROR: cannot initialize libGLEW: %s",
		    glewGetErrorString(err));
		return (0);
	}
	if (!GLEW_VERSION_2_1) {
		logMsg("FATAL ERROR: your system doesn't support OpenGL 2.1");
		return (0);
	}

	confpath = mkpathname(xpdir, plugindir, "OpenGPWS.cfg", NULL);
	if (file_exists(confpath, NULL)) {
		int errline;

		conf = conf_read_file(confpath, &errline);
		if (conf == NULL) {
			if (errline < 0) {
				logMsg("Error reading configuration %s: cannot "
				    "open configuration file.", confpath);
			} else {
				logMsg("Error reading configuration %s: syntax "
				    "error on line %d.", confpath, errline);
			}
		}
	}
	if (conf == NULL)
		conf = conf_create_empty();
	dbg_log_init(conf);
	conf_free(conf);
	free(confpath);

	logMsg("OpenGPWS start succeded");

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
	logMsg("OpenGPWS stopped");
}

PLUGIN_API int
XPluginEnable(void)
{
	logMsg("OpenGPWS enabling");

	if (!snd_sys_init())
		return (0);
	fdr_find(&lat, "sim/flightmodel/position/latitude");
	fdr_find(&lon, "sim/flightmodel/position/longitude");
	fdr_find(&elev, "sim/flightmodel/position/elevation");
	fdr_find(&trk, "sim/flightmodel/position/hpath");
	fdr_find(&asi_kts, "sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
	fdr_find(&asi_fail, "sim/operation/failures/rel_ss_asi");
	fdr_find(&gs, "sim/flightmodel/position/groundspeed");
	fdr_find(&vs_ft, "sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
	fdr_find(&vs_fail, "sim/operation/failures/rel_ss_vvi");
	fdr_find(&ra_ft,
	    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");
	fdr_find(&on_gnd, "sim/flightmodel2/gear/on_ground");

	fdr_find(&loc_fail, "sim/operation/failures/rel_loc");
	fdr_find(&gs_fail, "sim/operation/failures/rel_gls");

	fdr_find(&nav1.freq, "sim/cockpit/radios/nav1_freq_hz");
	fdr_find(&nav1.type, "sim/cockpit2/radios/indicators/nav1_type");
	fdr_find(&nav1.id, "sim/cockpit2/radios/indicators/nav1_nav_id");
	fdr_find(&nav1.hdef, "sim/cockpit/radios/nav1_hdef_dot");
	fdr_find(&nav1.vdef, "sim/cockpit/radios/nav1_vdef_dot");
	fdr_find(&nav1.gs_flag,
	    "sim/cockpit2/radios/indicators/hsi_flag_glideslope_pilot");

	fdr_find(&nav2.freq, "sim/cockpit/radios/nav2_freq_hz");
	fdr_find(&nav2.type, "sim/cockpit2/radios/indicators/nav2_type");
	fdr_find(&nav2.id, "sim/cockpit2/radios/indicators/nav2_nav_id");
	fdr_find(&nav2.hdef, "sim/cockpit/radios/nav2_hdef_dot");
	fdr_find(&nav2.vdef, "sim/cockpit/radios/nav2_vdef_dot");
	fdr_find(&nav1.gs_flag,
	    "sim/cockpit2/radios/indicators/hsi_flag_glideslope_copilot");

	/* We initialize this early, as OpenWXR can call us for terrain */
	terr_init();

	XPLMRegisterFlightLoopCallback(sensor_cb, SENSOR_INTVAL, NULL);
#ifndef	NOAUDIO
	XPLMRegisterFlightLoopCallback(sound_cb, -1, NULL);
#endif

	logMsg("OpenGPWS successful");

	return (1);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(sensor_cb, NULL);
#ifndef	NOAUDIO
	XPLMUnregisterFlightLoopCallback(sound_cb, NULL);
#endif
	logMsg("OpenGPWS disabling");
	egpws_fini();
	terr_fini();
	snd_sys_fini();
	logMsg("OpenGPWS disable complete");
}

static void
set_state(const egpws_conf_t *conf)
{
	if (conf != NULL && !booted) {
		VERIFY(conf->terr_colors != NULL);
		egpws_init(conf);
		booted = B_TRUE;
		pos_ok = B_TRUE;
		ra_ok = B_TRUE;
		on_gnd_ok = B_TRUE;
	} else if (conf == NULL && booted) {
		egpws_fini();
		booted = B_FALSE;
	}
}

static void
set_pos_ok(bool_t flag)
{
	pos_ok = flag;
}

static void
set_ra_ok(bool_t flag)
{
	ra_ok = flag;
}

static void
set_on_gnd_ok(bool_t flag)
{
	on_gnd_ok = flag;
}

static void
set_nav_on(bool_t nav1_on, bool_t nav2_on)
{
	nav1.on = nav1_on;
	nav2.on = nav2_on;
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);

	switch (msg) {
	case EGPWS_GET_INTF: {
		egpws_intf_t **intf_p = param;
		*intf_p = &egpws_intf;
		break;
	}
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

#if	IBM
BOOL WINAPI
DllMain(HINSTANCE hinst, DWORD reason, LPVOID resvd)
{
	UNUSED(hinst);
	UNUSED(resvd);
	lacf_glew_dllmain_hook(reason);
	return (TRUE);
}
#endif	/* IBM */
