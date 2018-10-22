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
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#version 120

uniform sampler2D	tex;
uniform float		acf_elev_ft;
uniform vec2		hgt_rngs_ft[4];
uniform vec4		hgt_colors[4];

varying vec2		tex_coord;

float
m2ft(float m)
{
	return (m * 3.2808398950131);
}

void
main()
{
	/*
	 * The elevation is stored in the red & green channels, offset by
	 * 10000m to avoid underflowing on negative elevations.
	 */
	vec4 pixel = texture2D(tex, tex_coord);
	float terr_elev_m = ((pixel.r * 255.0) + (pixel.g * 255 * 256)) - 10000;
	float terr_elev_ft = m2ft(terr_elev_m);
	float hgt_ft = acf_elev_ft - terr_elev_ft;

	for (int i = 0; i < 4; i++) {
		if (hgt_ft >= hgt_rngs_ft[i].x && hgt_ft < hgt_rngs_ft[i].y) {
			gl_FragColor = hgt_colors[i];
			return;
		}
	}
	gl_FragColor = vec4(0);
}
