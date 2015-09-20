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
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"

float d_oi;
float d_vo;
float r_vo;
float rpz;
float theta;
double DD_oi[2][1];
double A[2][1];
float C;
float azimuthangle;
float positionx;

void cherrytest(unsigned char *buffer)
{
	printf("new AC id is %i\n",DL_REMOTE_GPS_ac_id(dl_buffer));

  	struct EcefCoor_i new_pos;
  	struct EnuCoor_i enu;
  	new_pos.x = DL_REMOTE_GPS_ecef_x(dl_buffer);
  	new_pos.y = DL_REMOTE_GPS_ecef_y(dl_buffer);
  	new_pos.z = DL_REMOTE_GPS_ecef_z(dl_buffer);

  	enu_of_ecef_point_i(&enu, &state.ned_origin_i, &new_pos);
  	INT32_VECT3_SCALE_2(enu, enu, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);

	printf("hello %f\n",enu.x);
	printf("hello %f\n",enu.y);
	printf("hello %f\n",enu.z);
} 

int funfun()
{
	printf("Position x %f\n", stateGetPositionEnu_f()->x);
	printf("Position y %f\n", stateGetPositionEnu_f()->y);
	printf("Position z %f\n", stateGetPositionEnu_f()->z);
	printf("Speed x %f\n", stateGetSpeedEnu_f()->x);
	printf("Speed y %f\n", stateGetSpeedEnu_f()->y);
	printf("Speed z %f\n", stateGetSpeedEnu_f()->z);
	printf("aircraft id %i\n", AC_ID);

	positionx = stateGetPositionEnu_f()->x;

	d_vo = (pow(d_oi, 2) - pow(rpz, 2)) / d_oi;
	r_vo = rpz * (sqrt  (pow(d_oi,2) - pow(rpz,2)   )   )/d_oi;
	theta = atan(r_vo/d_vo);
	
	DD_oi[1][1] = cos(azimuthangle) * d_vo;
	DD_oi[2][1] = sin(azimuthangle) * d_vo;

	DD_oi[1][1]*A[1][1] + DD_oi[2][1]*A[2][1];
}
