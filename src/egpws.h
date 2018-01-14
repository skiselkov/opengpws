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

#include <opengpws/xplane_api.h>

#ifdef __cplusplus
extern "C" {
#endif

void egpws_init(egpws_conf_t acf_conf);
void egpws_fini(void);
void egpws_set_position(egpws_pos_t pos, double now);
void egpws_set_dest(const egpws_arpt_ref_t *ref);
void egpws_set_flaps_ovrd(bool_t flag);

#ifdef __cplusplus
}
#endif

#endif	/* _EGPWS_H_ */
