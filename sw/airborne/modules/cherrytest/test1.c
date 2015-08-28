/*
 * Copyright (C) 2015
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "cherrytest/test1.h"
#include "state.h"


float d_oi;
float d_vo;
float r_vo;
float rpz;
float theta;

double DD_oi[2][1];
double A[2][1];
float C;
float azimuthangle;


void avoidance_alg(void)
{

	printf("hey %f\n", stateCalcPositionEcef_i);
	printf("hey %f\n", stateCalcPositionNed_i);
	printf("hey %f\n", stateCalcPositionEnu_i);

	d_oi = 2;
	d_vo = (pow(d_oi, 2) - pow(rpz, 2)) / d_oi;
	r_vo = rpz * (sqrt  (pow(d_oi,2) - pow(rpz,2)   )   )/d_oi;
	theta = atan(r_vo/d_vo);
	
	DD_oi[1][1] = cos(azimuthangle) * d_vo;
	DD_oi[2][1] = sin(azimuthangle) * d_vo;


	double B(double A[], double DD_oi[], int n)	
	{
	double result = 0.0;
	for (int i = 0; i < n; i++)
		result += A[i]*DD_oi[i];
	return result;
	}


}


