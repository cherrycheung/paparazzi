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
#include <std.h>
#include <stdlib.h>
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
#include "generated/flight_plan.h"  //needed to use WP_HOME
#include "subsystems/ins.h"
#include "math/pprz_geodetic_float.h"

int valueofdetection1 = 0;
int valueofnavigation1 = 0;
float azimuth = 0;
float own_heading = 0;
float d_avo = 0;

int avoid_detection1()
{
  // OWN coordinates
  float own_pos_x = stateGetPositionEnu_f()->x;
  float own_pos_y = stateGetPositionEnu_f()->y;
  float own_speed_x = stateGetSpeedEnu_f()->x;
  float own_speed_y = stateGetSpeedEnu_f()->y;
  float own_course_rad = *stateGetHorizontalSpeedDir_f();
  float own_course_deg = (own_course_rad/M_PI)*180;
  float own_heading_rad = stateGetNedToBodyEulers_f()->psi;
  float own_heading_deg = (own_heading_rad/M_PI)*180;

  // INTRUDER coordinates
  int ac_id2 = 207;
  struct ac_info_ * intr = get_ac_info(ac_id2);
  struct ac_info_ intruder;
  intruder.ac_id = intr->ac_id;
  intruder.course = intr->course;
  intruder.east = intr->east - 594535.5000;
  intruder.north = intr->north - 5760891.5000;
  intruder.gspeed = intr->gspeed;
  float int_pos_x = intruder.east;
  float int_pos_y = intruder.north;
  float int_heading_rad;
  if (intruder.course > M_PI){
    int_heading_rad = intruder.course - 2*M_PI;
  }
  else{
    int_heading_rad = intruder.course;
  }
  float int_heading_deg = (int_heading_rad/M_PI)*180;
  float int_speed_x = cos((intruder.course)*-1 + 0.5*M_PI)*intruder.gspeed;
  float int_speed_y = sin((intruder.course)*-1 + 0.5*M_PI)*intruder.gspeed;

  float angle_global = calcGlobalAngle1(own_pos_x, own_pos_y, int_pos_x, int_pos_y);
  float angle_azimuth = calcAzimuthAngle1(own_pos_x, own_pos_y, int_pos_x, int_pos_y,own_heading_deg);

  /* Avoidance data */
  float rpz = 0.5;
  d_avo = 1.85;
  float d_oi = sqrt(powf((own_pos_x - int_pos_x),2) + powf((own_pos_y - int_pos_y),2));

  // YAZDI'S EQUATIONS
  float d_vo = (d_oi*d_oi - rpz*rpz)/d_oi;
  float r_vo = rpz*((sqrt(d_oi*d_oi - rpz*rpz))/d_oi);
  float alpha_vo = atan(r_vo/d_vo);
  /* float theta_vo = 0.00; */
  float DD_vo[2];
  DD_vo[0] = d_vo * cos(angle_azimuth); /** cos(theta_vo);*/
  DD_vo[1] = d_vo * sin(angle_azimuth); /** cos(theta_vo);*/
  float AA = (own_speed_x-int_speed_x)*DD_vo[0]+(own_speed_y-int_speed_y)*DD_vo[1];
  float AAA = sqrt(powf((own_speed_x-int_speed_x),2)+powf((own_speed_y-int_speed_y),2))*d_vo;
  float BB = AA/AAA;

  (sqrt((own_speed_x - int_speed_x)*(own_speed_x - int_speed_x) + (own_speed_y - int_speed_y)*(own_speed_y - int_speed_y)) *d_vo);
  float avoid_angle = acos(BB);

  printf("drone1: %f  %f  %f  %f  \n",avoid_angle,alpha_vo,AA,AAA);

  if (d_oi > rpz){
    if (own_heading_deg > (angle_global - 70) &&  own_heading_deg < (angle_global + 70)){
      if (d_oi < d_avo){
          if (avoid_angle < alpha_vo && BB > 0){
            valueofdetection1 = 1;
            printf("  avoid mode \n");
            azimuth = angle_azimuth;
            own_heading = own_heading_deg;
            return(1);
          }
          else if(avoid_angle < alpha_vo && BB < 0){
            //printf("  maintain mode 1");
          }
          else if(avoid_angle > alpha_vo && BB > 0){
            //printf("  maintain mode 2");
          }
          else if(avoid_angle > alpha_vo && BB < 0){
            //printf("  maintain mode 3");
          }
      }
    }
    //printf(" \n");
  }

  return(0);
  //printf(" \n");
}

int avoid_navigation1(uint8_t wpb,float angle_avoid){
  float angle_avoidance;
  angle_avoidance = own_heading + angle_avoid;
  float angle_avoidance_rad = angle_avoidance/180*M_PI;

  float x_inc = d_avo*1.2 * sin(angle_avoidance_rad);
  float y_inc = d_avo*1.2 * cos(angle_avoidance_rad);

  NavCherry(wpb,x_inc,y_inc);
  nav_set_heading_towards(x_inc + stateGetPositionEnu_f()->x, y_inc + stateGetPositionEnu_f()->y);
  NavGotoWaypoint(wpb);
  return 0;
}

int safe_setting1(){
  valueofdetection1 = 0;
  printf("Value of detection has been resetted\n");
return 0;
}

float calcGlobalAngle1(float ownshipx, float ownshipy, float intruderx, float intrudery){
  float global_angle1;
  if(intrudery > ownshipy){
    if(intruderx == ownshipx){
      global_angle1 = 0;
    }
      else if(intruderx > ownshipx){
	global_angle1 = (atan(((intruderx - ownshipx))/((intrudery - ownshipy))))/M_PI * 180;
      }
      else if(intruderx < ownshipx){
	global_angle1 =  atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180;
      }
    }
    else if(intrudery < ownshipy){
      if(intruderx == ownshipx){
	global_angle1 = 180;
      }
      else if(intruderx > ownshipx){
	global_angle1 = 180-(-1)*(atan((intruderx - ownshipx)/((intrudery - ownshipy))))/M_PI * 180;
      }
      else if(intruderx < ownshipx){
	global_angle1 =  -1 * (180 - (atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180));
      }
    }
    else if(intrudery == ownshipy){
      if(intruderx > ownshipx){
	global_angle1 = 90;
      }
      else if (intruderx < ownshipx){
	global_angle1 = -90;
      }
      else if (intruderx == ownshipx){
	global_angle1 = 0;
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
      else if(intruderx > ownshipx){ /* situation 2 */
	       global_angle1 = (atan(((intruderx - ownshipx))/((intrudery - ownshipy))))/M_PI * 180;
	        if(angleownship<0 && angleownship>(global_angle1-180)){
	           azimuth_angle = -1* ( abs(angleownship)+global_angle1) ;
	        }
	        else if(angleownship>0 && angleownship<global_angle1){
	           azimuth_angle = -1* (global_angle1 - angleownship);
	        }
	        else if(angleownship<(global_angle1-180) && angleownship>-180){
	           azimuth_angle = 180-global_angle1+abs(angleownship);
	        }
	        else{
	           azimuth_angle = abs(angleownship)-global_angle1;
	        }
      }
      else if(intruderx < ownshipx){ /* situation 3 */
	       global_angle1 =  atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180;
	        if(angleownship>0 && angleownship<(global_angle1+180)){
	           azimuth_angle = abs(global_angle1)+angleownship;
	        }
	        else if(angleownship<0 && angleownship>global_angle1){
	           azimuth_angle = abs(global_angle1) - abs(angleownship);
	        }
	        else if(angleownship>(global_angle1+180) && angleownship<180){
	           azimuth_angle = -1* (360 - angleownship-abs(global_angle1));
	        }
	        else{
	           azimuth_angle = -1*( abs(angleownship)-abs(global_angle1));
	        }
      }
    }
    else if(intrudery < ownshipy){
      if(intruderx == ownshipx){ /* situation 4 */
	       global_angle1 = 180;
	        if(angleownship > 0){
	           azimuth_angle = -1 *  (global_angle1 - abs(angleownship));
	        }
	        else{
	           azimuth_angle = (global_angle1 - abs(angleownship));
	          }
          }
        else if(intruderx > ownshipx){  /* situation 5 */
	         global_angle1 = 180-(-1)*(atan((intruderx - ownshipx)/((intrudery - ownshipy))))/M_PI * 180;
	          if(angleownship>0 && angleownship<global_angle1){
	            azimuth_angle = -1*(global_angle1 - angleownship);
	          }
	          else if(angleownship<180 && angleownship>global_angle1){
	            azimuth_angle = -1*(angleownship - global_angle1);
	            }
	          else if(angleownship<0 && angleownship>(global_angle1-180)){
	            azimuth_angle = global_angle1+abs(angleownship);
	          }
	          else{
	             azimuth_angle = 360-abs(global_angle1)-abs(angleownship);
	          }
        }
        else if(intruderx < ownshipx){ /* situation 6 */
	         global_angle1 =  -1 * (180 - (atan(((intruderx - ownshipx))/((intrudery - ownshipy)))/M_PI * 180));
	          if(angleownship<0 && angleownship>global_angle1){
	             azimuth_angle = abs(global_angle1 - angleownship);
	          }
	          else if(angleownship>-180 && angleownship<global_angle1){
	             azimuth_angle = (angleownship - global_angle1);
	          }
	          else if(angleownship>0 && angleownship<(global_angle1+180)){
	             azimuth_angle = -1* (abs(global_angle1)+abs(angleownship));
	          }
	          else{
	             azimuth_angle = -1*(360-abs(global_angle1)-abs(angleownship));
	          }
        }
      }
      else if(intrudery == ownshipy){ /* situation 7 */
        if(intruderx > ownshipx){
	         global_angle1 = 90;
	          if(angleownship>0 && angleownship<global_angle1){
	             azimuth_angle = -1*(abs(global_angle1) - abs(angleownship));
	            }
	          else if(angleownship<180 && angleownship > global_angle1){
	             azimuth_angle = -1*(abs(angleownship) - abs(global_angle1));
	            }
	          else if(angleownship>-180 && angleownship < -90){
	             azimuth_angle = 360-abs(angleownship)-abs(global_angle1);
	            }
	          else{
	             azimuth_angle = abs(angleownship) + abs(global_angle1);
	            }
            }
        else if (intruderx < ownshipx){ /* situation 8 */
	         global_angle1 = -90;
	          if(angleownship<0 && angleownship>global_angle1){
	             azimuth_angle = abs(global_angle1) - abs(angleownship);
	            }
	          else if(angleownship>-180 && angleownship < global_angle1){
	             azimuth_angle = abs(angleownship) - abs(global_angle1);
	            }
	          else if(angleownship<180 && angleownship > 90){
	             azimuth_angle = -1*(360-abs(angleownship)-abs(global_angle1));
	            }
	          else{
	             azimuth_angle = -1*(abs(angleownship) + abs(global_angle1));
	            }
            }
        else if (intruderx == ownshipx){
	         global_angle1 = 0;
           azimuth_angle = 0;
         }
       }
       return(azimuth_angle);
}

	  /*
	  float zones[4][2];
	  zones[0][0] = -45;
	  zones[0][1] =  45;
	  zones[1][0] = zones[0][1];
	  zones[1][1] = zones[1][0] + 90;
	  zones[2][0] = zones[1][1];
	  zones[2][1] = zones[1][1] * -1;
	  zones[3][0] = zones[2][1];
	  zones[3][1] = zones[0][0];*/
