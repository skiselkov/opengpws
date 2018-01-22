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

#ifndef	_OPENGPWS_TERR_H_
#define	_OPENGPWS_TERR_H_

#include <acfutils/geom.h>
#include <opengpws/xplane_api.h>

#ifdef __cplusplus
extern "C" {
#endif

void terr_init(const char *xpdir, const char *plugindir);
void terr_fini(void);

void terr_set_pos(geo_pos3_t pos);
void terr_set_ranges(const egpws_range_t *new_rngs);
double terr_get_elev(geo_pos2_t pos);
double terr_get_elev_wide(geo_pos2_t pos);

void terr_render(const egpws_render_t *render);

#ifdef __cplusplus
}
#endif

#endif	/* _OPENGPWS_TERR_H_ */
