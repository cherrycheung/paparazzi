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

int valueofdetection2 = 0;
int valueofnavigation2 = 0;
float azimuth2 = 0;
float own_heading2 = 0;
float d_avo2 = 0;

int avoid_detection2()
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
  int ac_id2 = 201;
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

  float angle_global = calcGlobalAngle2(own_pos_x, own_pos_y, int_pos_x, int_pos_y);
  float angle_azimuth = calcAzimuthAngle2(own_pos_x, own_pos_y, int_pos_x, int_pos_y,own_heading_deg);

  /* Avoidance data */
  float rpz = 0.5;
  d_avo2 = 1.75;
  float d_oi = sqrt((own_pos_x - int_pos_x)*(own_pos_x - int_pos_x) + (own_pos_y - int_pos_y)*(own_pos_y - int_pos_y));

  // YAZDI'S EQUATIONS
  float d_vo = (d_oi*d_oi - rpz*rpz)/d_oi;
  float r_vo = rpz*((sqrt(d_oi*d_oi - rpz*rpz))/d_oi);
  float alpha_vo = atan(r_vo/d_vo);
  /* float theta_vo = 0.00; */
  float DD_vo[2];
  DD_vo[0] = d_vo * cos(angle_azimuth); /** cos(theta_vo);*/
  DD_vo[1] = d_vo * sin(angle_azimuth); /** cos(theta_vo);*/
  float BB = ((own_speed_x - int_speed_x) * DD_vo[0] + (own_speed_y - int_speed_y) * DD_vo[1])/(sqrt((own_speed_x - int_speed_x)*(own_speed_x - int_speed_x) + (own_speed_y - int_speed_y)*(own_speed_y - int_speed_y)) *d_vo);
  float avoid_angle = acos(BB);

  printf("drone2: %f  %f  %f  %f  \n",r_vo,avoid_angle,alpha_vo,BB);

  if (d_oi > rpz){
    if (own_heading_deg > (angle_global - 90) &&  own_heading_deg < (angle_global + 90)){
      if (d_oi < d_avo2){
          if (avoid_angle < alpha_vo && BB > 0){
            valueofdetection2 = 1;
            printf("  avoid mode \n");
            azimuth2 = angle_azimuth;
            own_heading2 = own_heading_deg;
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

int avoid_navigation2(uint8_t wpb,float angle_avoid){
  float angle_avoidance;
  angle_avoidance = own_heading2 + angle_avoid;
  float angle_avoidance_rad = angle_avoidance/180*M_PI;

  float x_inc = 1.75 * sin(angle_avoidance_rad);
  float y_inc = 1.75 * cos(angle_avoidance_rad);

printf("%f %f %f %f",stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y,x_inc + stateGetPositionEnu_f()->x, y_inc + stateGetPositionEnu_f()->y);
  NavCherry(wpb,x_inc,y_inc);
  nav_set_heading_towards(x_inc + stateGetPositionEnu_f()->x, y_inc + stateGetPositionEnu_f()->y);
  NavGotoWaypoint(wpb);
  return 0;
}

int safe_setting2(){
  valueofdetection2 = 0;
  printf("Value of detection 2 has been resetted\n");
return 0;
}

float calcGlobalAngle2(float ownshipx, float ownshipy, float intruderx, float intrudery){
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

float calcAzimuthAngle2(float ownshipx, float ownshipy, float intruderx, float intrudery,float angleownship){
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
