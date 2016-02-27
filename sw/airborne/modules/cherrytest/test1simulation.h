/*
 * Copyright (C) 2014
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

/** \file test1simulation.h
 *  \brief testing testing testing!
 */

#include "std.h"
#include <stdio.h>

/*#ifndef TEST_H__
#define TEST_H__

struct aTest {
  float own_direction_deg;
  float angle_azimuth;
};
extern struct aTest Test;

#endif*/
#ifndef _LISTH_
#define _LISTH_

struct uav {
  float pos_x;
  float pos_y;
  float speed;
  float direction;
};

extern struct uav ownship;
extern struct uav intruder;

#endif

int function_init();
void getOwnship();
void getIntruder();
void getRelative();

extern int avoid_detection1(void);
float calcGlobalAngle1(float ownshipx, float ownshipy, float intruderx, float intrudery);
float calcAzimuthAngle1(float ownshipx, float ownshipy, float intruderx, float intrudery,float angleownship);
void calcAvoidanceDist(float lala, float rpz, float ownshipangle_rad, float* d_avo1, float* x_inc, float* y_inc);
extern int avoid_navigation1(uint8_t wpb);
extern int safe_setting1(void);


extern int valueofdetection1;
extern int valueofnavigation1;
extern float hellobaby;
extern float new_waypoint_x;
extern float new_waypoint_y;
extern float angle_azimuth;
extern float own_direction_deg;
extern float testvalue;
extern int simulation;
extern int ac_id2;
