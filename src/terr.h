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

void terr_init(void);
void terr_fini(void);

void terr_set_pos(geo_pos3_t pos);
void terr_set_ranges(const egpws_range_t *new_rngs);
double terr_get_elev(geo_pos2_t pos);
double terr_get_elev_wide(geo_pos2_t pos, geo_pos3_t out_pos[9]);
void terr_probe(egpws_terr_probe_t *probe);
bool_t terr_have_data(geo_pos2_t pos, double *tile_load_res);

void terr_render(const egpws_render_t *render);
fpp_t terr_render_get_fpp(const egpws_render_t *render);

void terr_reload_gl_progs(void);

#ifdef __cplusplus
}
#endif

#endif	/* _OPENGPWS_TERR_H_ */
