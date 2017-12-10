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

#include <GL/glew.h>

#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/core.h>
#include <acfutils/crc64.h>
#include <acfutils/dr.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "egpws.h"
#include "snd_sys.h"

#define	PLUGIN_NAME		"OpenGPWS by Saso Kiselkov"
#define	PLUGIN_SIG		"skiselkov.opengpws"
#define	PLUGIN_DESCRIPTION	"An open-source Mk.VIII EGPWS simulation"

char			xpdir[512];
char			plugindir[512];

int			xp_ver, xplm_ver;
XPLMHostApplicationID	host_id;

static float
floop_cb(float elapsed, float elapsed2, int counter, void *refcon)
{
	UNUSED(elapsed);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	return (-1);
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

	XPLMRegisterFlightLoopCallback(floop_cb, -1, NULL);

	return (1);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);

	snd_sys_fini();
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(param);
	UNUSED(msg);
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
