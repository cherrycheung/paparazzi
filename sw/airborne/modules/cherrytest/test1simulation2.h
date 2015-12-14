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

/** \file test1simulation2.h
 *  \brief testing testing testing!
 */

#include "std.h"
#include "math/pprz_geodetic_int.h"

#include "subsystems/navigation/waypoints.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/navigation/traffic_info.h"
#include "messages.h"
#include "dl_protocol.h"

#include <stdio.h>
#include <time.h>
#include "state.h"
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"
#include "math.h"
#include "subsystems/navigation/traffic_info.h"
#include "subsystems/gps.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "navigation.h"

extern int avoid_detection2(void);
extern int avoid_navigation2(void);
float calcGlobalAngle2(float ownshipx, float ownshipy, float intruderx, float intrudery);
float calcAzimuthAngle2(float ownshipx, float ownshipy, float intruderx, float intrudery,float angleownship);

extern int valueofdetection2;
extern int valueofnavigation2;
extern int safe_warning2();
