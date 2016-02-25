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
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "navigation.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME
#include "subsystems/ins.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_int.h"

int valueofdetection1 = 0;
int valueofnavigation1 = 0;
float azimuth = 0;
float own_direction = 0;
float d_avo = 0;

int avoid_detection1()
{
  int simulation = 1; // 1 for simulation
  int useheading = 1; // 1 for heading
  int useutmorenu = 2; // 2 for enu
  int userow = 0; // 1 for row
  float rpz = 0.5;
  d_avo = 1.5;
  int ac_id1 = 3;

  int ac_id2;
  if (simulation == 1){
    if (ac_id1 == 3){
      ac_id2 = 33;
    }
    else{
      ac_id2 = 44;
    }
  }
  else{
    if (ac_id1 == 3){
      ac_id2 = 3;
    }
    else{
      ac_id2 = 4;
    }
  }


  // OWN data
  float own_pos_x;
  float own_pos_y;

  if (useutmorenu == 1){
    struct UtmCoor_i own_pos;
    own_pos.zone = 31;
    printf("%d\n",own_pos.zone);
    utm_of_lla_i(&own_pos, &gps.lla_pos);
    own_pos_x = own_pos.east/100;                // in cm
    own_pos_y = own_pos.north/100;               // in cm
  }
  else{
    own_pos_x = stateGetPositionEnu_f()->x; // in m
    own_pos_y = stateGetPositionEnu_f()->y; // in m
  }
  float own_speed_x = stateGetSpeedEnu_f()->x;
  float own_speed_y = stateGetSpeedEnu_f()->y;
  float own_direction_rad;
  float own_direction_deg;
  if (useheading==1){
    own_direction_rad = stateGetNedToBodyEulers_f()->psi;
    own_direction_deg = (own_direction_rad/M_PI)*180;
  }
  else{
    own_direction_rad = stateGetHorizontalSpeedDir_f();
    own_direction_deg = (own_direction_rad/M_PI)*180;
  }

  // INTRUDER coordinates, change server.ml file for course/heading switch!!!
  struct ac_info_ * intr = get_ac_info(ac_id2);
  //float delta_t = Max((int)(gps.tow - intr->itow) / 1000., 0.);
  // if AC not responding for too long, continue, else compute force
  //if (delta_t > CARROT) { continue; }
  struct ac_info_ intruder = *intr;
  float intr_pos_x;
  float intr_pos_y;
  if (simulation == 1){
    intr_pos_x = intruder.east- 594534.8125;
    intr_pos_y = intruder.north - 5760891.500;
  }
  else{
    intr_pos_x = intruder.east/100-594534.84;
    intr_pos_y = intruder.north/100-5760891.52;;
  }

  float int_direction_rad;
  if (intruder.course > M_PI){
    int_direction_rad = intruder.course - 2*M_PI;
  }
  else{
    int_direction_rad = intruder.course;
  }
  float int_direction_deg = (int_direction_rad/M_PI)*180;
  float int_speed_x = cos((intruder.course)*-1 + 0.5*M_PI)*intruder.gspeed;
  float int_speed_y = sin((intruder.course)*-1 + 0.5*M_PI)*intruder.gspeed;

  // Relative data
  float angle_global = calcGlobalAngle1(own_pos_x, own_pos_y, intr_pos_x, intr_pos_y);
  float angle_azimuth = calcAzimuthAngle1(own_pos_x, own_pos_y, intr_pos_x, intr_pos_y,own_direction_deg);
  float d_oi = sqrt(powf((own_pos_x - intr_pos_x),2) + powf((own_pos_y - intr_pos_y),2));
  float angle_azimuth_rad;
  if(angle_azimuth < 0){
    angle_azimuth_rad = (angle_azimuth*(-1))/180*M_PI;
  }
  else{
    angle_azimuth_rad = ((angle_azimuth/180)*M_PI);
  }
  //float angle_azimuth_rad = ((angle_azimuth/180)*M_PI);
  //printf("angle azimuth %f %f %f %f \n",angle_azimuth_rad,abs(angle_azimuth_rad),acos(angle_azimuth_rad),acos(abs(angle_azimuth_rad)));



  // Avoidance module
  float d_vo = (d_oi*d_oi - rpz*rpz)/d_oi;
  float r_vo = rpz*((sqrt(d_oi*d_oi - rpz*rpz))/d_oi);
  float alpha_vo = atan(r_vo/d_vo);
  float DD_vo[2];
  DD_vo[0] = d_vo * cos((angle_azimuth_rad)); /** cos(theta_vo);*/
  DD_vo[1] = d_vo * sin((angle_azimuth_rad)); /** cos(theta_vo);*/
  float AA = (own_speed_x-int_speed_x)*DD_vo[0]+(own_speed_y-int_speed_y)*DD_vo[1];
  float AAA = sqrt(powf((own_speed_x-int_speed_x),2)+powf((own_speed_y-int_speed_y),2))*d_vo;
  float BB = AA/AAA;
  float own_speed = sqrt(powf((own_speed_x),2)+powf((own_speed_y),2));
  float avoid_angle = acos(BB);

  printf("drone%d: d_oi %f own enu  x y %f %f int x y %f %f\n",AC_ID,d_oi,own_pos_x,own_pos_y,intr_pos_x,intr_pos_y);

  // Right of way
  int row_zone;
  if(userow==1){
    float row_angle = int_direction_deg - own_direction_deg;

    if (row_angle >= -45 && row_angle <= 45){
      row_zone = 1;
    }
    else if(row_angle > 45 && row_angle < 136){
      row_zone = 2;
    }
    else if(row_angle >= 136 && row_angle <= 180){
      row_zone = 3;
    }
    else if(row_angle >= -180 && row_angle <= -136){
      row_zone = 3;
    }
    else if(row_angle >= -135 && row_angle <= -46){
      row_zone = 4;
    }
  }
  if (row_zone == 1){
    printf("drone %d: same path\n",AC_ID);
  }
  else if(row_zone == 2){
    printf("drone %d: converging left and has right of way\n",AC_ID);
  }
  else if(row_zone == 3){
    printf("drone %d: head on\n",AC_ID);
  }
  else if(row_zone == 3){
    printf("drone %d: head on\n",AC_ID);
  }
  else if(row_zone == 4){
    printf("drone %d: converging right\n",AC_ID);
  }

  printf("drone%d: Vox & Voy %f %f azimuth %f Dvox & Dvoy %f %f\n",AC_ID, own_speed_x, own_speed_y,angle_azimuth,DD_vo[0],DD_vo[1]);
  printf("drone%d: d_vo %f r_vo %f\n",AC_ID, d_vo, r_vo);
  //printf("drone int: Vix & Viy %f %f azimuth %f Dvox & Dvoy %f %f\n",AC_ID, own_speed_x, own_speed_y,angle_azimuth,DD_vo[0],DD_vo[1]);
  printf("drone%d: avoidangle %f alphavo %f BB %f\n", AC_ID,avoid_angle,alpha_vo,BB);

  if(intruder.gspeed>0 && own_speed>0 && own_direction_deg > (angle_global - 100) &&  own_direction_deg < (angle_global + 100)){
    if (d_oi > rpz){
      printf("drone%d: outside the protected zone %f \n", AC_ID,d_oi);
      if (d_oi < d_avo){
        if (avoid_angle < alpha_vo && BB > 0){
          printf("drone%d: inside VO \n", AC_ID);
          if(userow==0){
            valueofdetection1 = 1;
            azimuth = angle_azimuth;
            own_direction = own_direction_deg;
            printf("drone%d: YUP \n", AC_ID);
          }
          else if(userow==1){
            if (row_zone == 1 || row_zone == 3 || row_zone == 4){
              valueofdetection1 = 1;
              azimuth = angle_azimuth;
              own_direction = own_direction_deg;
              printf("drone%d: YUP ", AC_ID);
              if (row_zone == 1){
                printf("drone %d: same path\n",AC_ID);
              }
              else if(row_zone == 2){
                printf("drone %d: converging left and has right of way\n",AC_ID);
              }
              else if(row_zone == 3){
                printf("drone %d: head on\n",AC_ID);
              }
              else if(row_zone == 3){
                printf("drone %d: head on\n",AC_ID);
              }
              else if(row_zone == 4){
                printf("drone %d: converging right\n",AC_ID);
              }
                return(1);
            }
          }
        }
      }
      else{
        printf("drone%d: outside VO \n", AC_ID);
      }
    }
    else{
      printf("drone%d: inside the protected zone DANGER %f\n", AC_ID,d_oi);
      valueofdetection1 = 1;
    }
  }
  else if (intruder.gspeed == 0){
    if (angle_azimuth_rad < atan(rpz/d_oi) && d_oi < d_avo){
      valueofdetection1 = 1;
      azimuth = angle_azimuth;
      own_direction = own_direction_deg;
      printf("drone%d: not moving intruder %f %f", AC_ID,angle_azimuth_rad/M_PI*180,atan(d_oi/rpz)/M_PI*180);
    }
  }
  return(0);
}

int avoid_navigation1(uint8_t wpb,float angle_avoid){
  float angle_avoidance;
  angle_avoidance = own_direction + angle_avoid;
  float angle_avoidance_rad = angle_avoidance/180*M_PI;

  float avoid_x = d_avo*1.2 * sin(angle_avoidance_rad);
  float avoid_y = d_avo*1.2 * cos(angle_avoidance_rad);

  NavCherry(wpb,avoid_x,avoid_y);
  nav_set_heading_towards(avoid_x + stateGetPositionEnu_f()->x, avoid_y + stateGetPositionEnu_f()->y);
  NavGotoWaypoint(wpb);
  return 0;
}

int safe_setting1(){
  valueofdetection1 = 0;
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


/*if (d_oi > rpz){
  if (own_speed > 0){
    if (own_direction_deg > (angle_global - 100) &&  own_direction_deg < (angle_global + 100)){
      if (avoid_angle < alpha_vo && BB > 0){
        printf("drone %d: inside \n", AC_ID);
        if (d_oi < d_avo){
          //if (row_zone == 1 || row_zone == 3 || row_zone == 4){
            valueofdetection1 = 1;
            printf("drone %d: avoid mode \n", AC_ID);
            azimuth = angle_azimuth;
            own_direction = own_direction_deg;
            if (row_zone == 1){
              printf("drone %d: same path\n",AC_ID);
            }
            else if(row_zone == 2){
              printf("drone %d: converging left and has right of way\n",AC_ID);
            }
            else if(row_zone == 3){
              printf("drone %d: head on\n",AC_ID);
            }
            else if(row_zone == 3){
              printf("drone %d: head on\n",AC_ID);
            }
            else if(row_zone == 4){
              printf("drone %d: converging right\n",AC_ID);
            }
            return(1);
          //}
        }
        else{
          printf("drone %d: collide soon\n",AC_ID);
        }
      }
    }
  }
} */
