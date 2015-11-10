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
#include "cherrytest/test1simulation.h"
#include "state.h"
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"
#include "math.h"
#include "subsystems/navigation/traffic_info.h"
#include "subsystems/gps.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "navigation.h"
#include "avoidcalculations.h"

int avoid_detection1()
{
  
  
  /* To construct the package of incoming data */
  int ac_id2 = 207;
  struct ac_info_ * intr = get_ac_info(ac_id2);
  struct ac_info_ intruder;
  
  intruder.ac_id = intr->ac_id;
  intruder.course = intr->course;
  intruder.east = intr->east - 594534.8125;
  intruder.north = intr->north - 5760891.5000;      
  intruder.alt = intr->alt;
  intruder.itow = intr->itow;    
  intruder.gspeed = intr->gspeed;
  intruder.climb = intr->climb;
  
  float own_pos_x = stateGetPositionEnu_f()->x;
  float own_pos_y = stateGetPositionEnu_f()->y;
  float own_speed_x = stateGetSpeedEnu_f()->x;
  float own_speed_y = stateGetSpeedEnu_f()->y;
  
  float int_pos_x = intruder.east;
  float int_pos_y = intruder.north;
  float int_speed_x = cos(intruder.course)*intruder.gspeed;
  float int_speed_y = sin(intruder.course)*intruder.gspeed;

  float rpz = 0.50; 
  float d_avo = 1.00; 
  float d_oi = sqrt((own_pos_x - int_pos_x)*(own_pos_x - int_pos_x) + (own_pos_y - int_pos_y)*(own_pos_y - int_pos_y));
  float angle_ownship_rad = stateGetNedToBodyEulers_f()->psi;
  float angle_ownship_deg = (angle_ownship_rad/M_PI)*180;
  float angle_global = calcGlobalAngle1(own_pos_x, own_pos_y, int_pos_x, int_pos_y,angle_ownship_deg);  
  float angle_azimuth = calcAzimuthAngle1(own_pos_x, own_pos_y, int_pos_x, int_pos_y,angle_ownship_deg);  
  /* PRINTING */ 
  printf("ardrone 1, d_oi: %f  ", d_oi);
  /*printf("201: Pos x & y %f, %f\n", own_pos_x, own_pos_y); 
  printf("%d: Pos x & y %f, %f\n",intruder.ac_id,int_pos_x,int_pos_y);
  
  
  
   
  printf("ownspeed x and y: %f & %f  ", own_speed_x, own_speed_y); 
  printf("intruder speed x and y: %f & %f  ", int_speed_x, int_speed_y);*/ 
  printf("ownship angle: %f %f ",angle_ownship_deg,angle_global);
 
  
  if (d_oi > rpz){
    if (angle_ownship_deg > (angle_global - 45) &&  angle_ownship_deg < (angle_global + 45)){
      float d_vo = (d_oi*d_oi - rpz*rpz)/d_oi;
      float r_vo = rpz * (sqrt(d_oi*d_oi - rpz*rpz))/d_oi;
      float alpha_vo = atan(r_vo/d_vo);
      float theta_vo = 0.00;
    
      /* theta_vo = 0 in this case because we have a 2D case */ 
      float DD_oi[2];
      DD_oi[0] = d_vo * cos(angle_azimuth) * cos(theta_vo);
      DD_oi[1] = d_vo * sin(angle_azimuth) * cos(theta_vo);

    
      float BB = (own_speed_x - int_speed_x) * DD_oi[0] + (own_speed_y - int_speed_y) * DD_oi[1];
      float CC = sqrt((own_speed_x - int_speed_x)*(own_speed_x - int_speed_x) + (own_speed_y - int_speed_y)*(own_speed_y - int_speed_y)) *d_vo;
      float avoid_angle = acos(BB/(CC));
    
      if (avoid_angle < alpha_vo && BB > 0){
	if (d_oi < d_avo){
	  printf("colliding\n");
	}
	else{
	  printf("not colliding yet\n");
	}
      }
      else{
      printf("speed angle inside but safe\n");
	
      }
      
    }
    else{
      printf("speed angle too large so safe\n");
    }
  }
  else{ 
    printf("inside protected zone\n");
  }
  return 0;
}

int avoid_navigation1(){
  int intint = avoid_detection1;
  if(intint = 1){
    printf("colliding works\n");
  }
  return 0;
}

int headingset(int yawangle){
  nav_set_heading_deg(yawangle);
  return 0;
}

float calcGlobalAngle1(float ownshipx, float ownshipy, float intruderx, float intrudery,float angleownship){
  float global_angle1;
  float azimuth_angle;
  if(intrudery > ownshipy){
    if(intruderx == ownshipx){
      global_angle1 = 0;
      azimuth_angle = angleownship;

    }
      else if(intruderx > ownshipx){
	global_angle1 = (atan(((intruderx - ownshipx))/((intrudery - ownshipy))))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if(intruderx < ownshipx){
	global_angle1 =  atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
    }
    else if(intrudery < ownshipy){
      if(intruderx == ownshipx){
	global_angle1 = 180;
	azimuth_angle = (global_angle1 - abs(angleownship));
      }
      else if(intruderx > ownshipx){
	global_angle1 = 180-(-1)*(atan((intruderx - ownshipx)/((intrudery - ownshipy))))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if(intruderx < ownshipx){
	global_angle1 =  -1 * (180 - (atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180));
	azimuth_angle = abs(global_angle1 - angleownship);
      }
    }
    else if(intrudery == ownshipy){
      if(intruderx > ownshipx){
	global_angle1 = 90;
        azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if (intruderx < ownshipx){
	global_angle1 = -90;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if (intruderx == ownshipx){
	global_angle1 = 0;
        azimuth_angle = 0;
      }
    }
    return(global_angle1);
}

float calcAzimuthAngle1(float ownshipx, float ownshipy, float intruderx, float intrudery,float angleownship){
  float global_angle1;
  float azimuth_angle;
  if(intrudery > ownshipy){
    if(intruderx == ownshipx){
      global_angle1 = 0;
      azimuth_angle = angleownship;

    }
      else if(intruderx > ownshipx){
	global_angle1 = (atan(((intruderx - ownshipx))/((intrudery - ownshipy))))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if(intruderx < ownshipx){
	global_angle1 =  atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
    }
    else if(intrudery < ownshipy){
      if(intruderx == ownshipx){
	global_angle1 = 180;
	azimuth_angle = (global_angle1 - abs(angleownship));
      }
      else if(intruderx > ownshipx){
	global_angle1 = 180-(-1)*(atan((intruderx - ownshipx)/((intrudery - ownshipy))))/M_PI * 180;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if(intruderx < ownshipx){
	global_angle1 =  -1 * (180 - (atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180));
	azimuth_angle = abs(global_angle1 - angleownship);
      }
    }
    else if(intrudery == ownshipy){
      if(intruderx > ownshipx){
	global_angle1 = 90;
        azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if (intruderx < ownshipx){
	global_angle1 = -90;
	azimuth_angle = abs(global_angle1 - angleownship);
      }
      else if (intruderx == ownshipx){
	global_angle1 = 0;
        azimuth_angle = 0;
      }
    }
    return(azimuth_angle);
}

