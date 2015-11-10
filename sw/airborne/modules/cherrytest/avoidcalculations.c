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

#include <stdio.h>
#include <time.h>
#include "state.h"
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"
#include "math.h"

float calcGlobalAngle(float ownshipx, float ownshipy, float intruderx, float intrudery){
  if(intrudery > ownshipy){
    if(intruderx == ownshipx){
      global_angle = 0;
    }
      else if(intruderx > ownshipx){
	global_angle = (atan(((intruderx - ownshipx))/((intrudery - ownshipy))))/M_PI * 180;
      }
      else if(intruderx < ownshipx){
	global_angle =  atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180;
      }
    }
    else if(intrudery < ownshipy){
      if(intruderx == ownshipx){
	global_angle = 180;
      }
      else if(intruderx > ownshipx){
	global_angle = 180-(-1)*(atan((intruderx - ownshipx)/((intrudery - ownshipy))))/M_PI * 180;
      }
      else if(intruderx < ownshipx){
	global_angle =  -1 * (180 - (atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180));
      }
    }
    else if(intrudery == ownshipy){
      if(intruderx > ownshipx){
	global_angle = 90;
      }
      else if (intruderx < ownshipx){
	global_angle = -90;
      }
      else if (intruderx == ownshipx){
	global_angle = 0;
      }
    }
    return(global_angle);
}

int calctest(){
  printf("calctest works");
  return 0;
}