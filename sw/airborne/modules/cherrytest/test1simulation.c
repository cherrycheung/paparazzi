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
float d_avo = 0;
float new_waypoint_x = 0;
float new_waypoint_y = 0;
int simulation = 0;

struct uav ownship;
struct uav intruder;
struct data relative;
struct data2 init;


int function_init(){
  int simulation;
  if(AC_ID == 3 || AC_ID == 4){
    simulation = 0;
    if(AC_ID == 3){
      ownship.id = AC_ID;
      intruder.id = 4;
    }
    else{
      ownship.id = AC_ID;
      intruder.id = 3;
    }
  }
  else if(AC_ID == 33 || AC_ID == 44){
    simulation = 1;
    if(AC_ID == 33){
      ownship.id = AC_ID;
      intruder.id = 44;
    }
    else{
      ownship.id = AC_ID;
      intruder.id = 33;
    }
  }
  return(simulation);
}

void getOwnship(){
  ownship.pos_x = stateGetPositionEnu_f()->x; // in m
  ownship.pos_y = stateGetPositionEnu_f()->y; // in m
  ownship.speed_x = stateGetSpeedEnu_f()->x;
  ownship.speed_y = stateGetSpeedEnu_f()->y;
  ownship.speed = sqrt(ownship.speed_x * ownship.speed_x + ownship.speed_y * ownship.speed_y);
  ownship.direction = stateGetNedToBodyEulers_f()->psi;
}

void getIntruder(int carrot){
  struct ac_info_ * ac = get_ac_info(intruder.id);
  //float delta_t = Max((int)(gps.tow - intr->itow) / 1000., 0.);
  // if AC not responding for too long, continue, else compute force
  //if (delta_t > CARROT) { continue; }
  struct ac_info_ intr = *ac;

  if (carrot == 1){
    intruder.pos_x = (intr.utm.east-59453483)/100.00;//- 594534.8125;
    intruder.pos_y = (intr.utm.north-576089149)/100.00;// - 5760891.500;
  }
  else if(carrot == 0){
    intruder.pos_x = intr.utm.east/100.-594534.83;
    intruder.pos_y = intr.utm.north/100.-5760891.49;
  }
  if (intr.course > 1800 && intr.course < 3601){
    intruder.direction = (intr.course/10.0)/180*M_PI - 2*M_PI;
  }
  else{
    intruder.direction = (intr.course/10.0)/180*M_PI;
  }

  intruder.speed_x = cos(intruder.direction)*(intr.gspeed/100.0);
  intruder.speed_y = sin(intruder.direction)*(intr.gspeed/100.0);
  intruder.speed = sqrt(intruder.speed_x * intruder.speed_x)+(intruder.speed_y * intruder.speed_y);
  //printf("intruder%d: intruder.direction %f speed x y %f %f\n",intruder.id, intruder.direction, intruder.speed_x,intruder.speed_y);
}

void getRelative(){
  simulation = function_init();
  getOwnship();
  getIntruder(simulation);
  relative.distance = sqrt(powf((ownship.pos_x - intruder.pos_x),2) + powf((ownship.pos_y - intruder.pos_y),2));
  calcGlobalAzimuth(ownship.pos_x, ownship.pos_y, intruder.pos_x, intruder.pos_y, ownship.direction, &relative.global_o, &relative.azimuth_o);
  calcGlobalAzimuth(intruder.pos_x, intruder.pos_y, ownship.pos_x, ownship.pos_y, intruder.direction, &relative.global_i, &relative.azimuth_i);
  printf("drone%d: relative distance %f\n",AC_ID,relative.distance);
  //printf("drone%d: positions o x y %f %f i x y %f %f\n",ownship.id,ownship.pos_x, ownship.pos_y, intruder.pos_y, intruder.pos_y);
  //printf("drone%d: speeds o x y %f %f i x y %f %f\n",ownship.id,ownship.speed_x, ownship.speed_y, intruder.speed_x, intruder.speed);
}

int avoid_detection1(){ // will become the relative function
  int userow = 1; // 1 for row
  init.rpz = 1.0;
  init.factor = 1.0;
  init.avoidance = 45.0/180.0 * M_PI;


  //printf("drone%d: positions o x y %f %f i x y \n",ownship.id,ownship.pos_x, ownship.pos_y, intruder.pos_y, intruder.pos_y);
  // Avoidance module
  float d_vo = (relative.distance*relative.distance - init.rpz*init.rpz)/relative.distance;
  float r_vo = init.rpz*((sqrt(relative.distance*relative.distance - init.rpz*init.rpz))/relative.distance);
  float alpha_vo = atan(r_vo/d_vo);

  float Rx = ownship.speed_x - intruder.speed_x;
  float Ry = ownship.speed_y - intruder.speed_y;
  float delta;
  float gamma;
  if (Rx >= 0 && Ry >= 0){
    delta = atan(Rx/Ry);
    gamma = (delta - relative.global_o);
    printf("zone1\n");
  }
  else if(Rx > 0 && Ry < 0){
    delta = 0.5*M_PI + atan(Rx/((-1)*Ry));
    gamma = (delta - relative.global_o);
    printf("zone2\n");
  }
  else if(Rx < 0 && Ry < 0){
    delta = 0.5*M_PI + atan(Rx/Ry);
    gamma = (delta - (-1)*relative.global_o);
    printf("zone3\n");
  }
  else{
    delta = atan((-1*Rx)/Ry);
    gamma = (delta - (-1)*relative.global_o);
    printf("zone4\n");
  }

  printf("drone%d:Rx Ry %f %f delta %f global %f gamma %f\n",ownship.id,Rx,Ry,delta, relative.global_o,gamma);
  calcAvoidanceDist(init.avoidance, init.rpz, ownship.direction, &d_avo, &new_waypoint_x, &new_waypoint_y);
  printf("drone%d: avoiding from a distance %f\n",ownship.id,d_avo);

  /*float DD_vo[2];
  DD_vo[0] = intruder.pos_x - ownship.pos_x;
  DD_vo[1] = intruder.pos_y - ownship.pos_y;
  float AA = (ownship.speed_x - intruder.speed_x) * DD_vo[0] + (ownship.speed_y - intruder.speed_y)*DD_vo[1];
  float AAA = sqrt(powf((ownship.speed_x - intruder.speed_x),2)+powf((ownship.speed_y - intruder.speed_y),2))*d_vo;
  float BB = AA/AAA;
  float beta_vo = acos(BB);*/

  /*printf("drone%d: ownship speed x y %f %f azimuth %f\n", ownship.id, ownship.speed_x, ownship.speed_y,relative.azimuth);
  printf("drone%d: intruder speed x y %f %f \n",ownship.id, intruder.speed_x, intruder.speed_y);
  printf("drone%d: d_vo %f r_vo %f  DD_vo[0] %f DD_vo[1] %f AA %f AAA %f\n" , ownship.id, d_vo, r_vo, DD_vo[0], DD_vo[1], AA, AAA);*/
  //printf("drone%d: beta_vo %f alpha_vo %f BB %f\n", ownship.id, beta_vo, alpha_vo, BB);

  // Right of way
  int row_zone;
  if(userow == 1){
    float row_angle = (intruder.direction - ownship.direction)/M_PI * 180;
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
  //printf("drone%d: rowzone %d\n", ownship.id, row_zone);


  if (relative.distance > init.rpz){
    if(abs(gamma) < alpha_vo){
      printf("drone%d: inside VO but not close enough\n", ownship.id);
      if (relative.distance < d_avo * 1.2){
        printf("drone%d: smaller than d_avo and inside VO\n", ownship.id);
        if(userow == 0){
          valueofdetection1 = 1;
          printf("drone%d: AVOID \n", ownship.id);
        }
        else if (userow == 1){
          if (row_zone == 1 || row_zone == 3 || row_zone == 4){
            valueofdetection1 = 1;
            printf("drone%d: no row, so action \n", ownship.id);
          }
          else{
            printf("drone%d: has the right of way so no action\n", ownship.id);
          }
        }
      }
    }
  }
  return(0);
}

int avoid_navigation1(uint8_t wpb){
  float avoid_x = new_waypoint_x;
  float avoid_y = new_waypoint_y;

  NavCherry(wpb,avoid_x,avoid_y);
  nav_set_heading_towards(avoid_x + stateGetPositionEnu_f()->x, avoid_y + stateGetPositionEnu_f()->y);
  NavGotoWaypoint(wpb);
  return 0;
}

int safe_setting1(){
  valueofdetection1 = 0;
return 0;
}

void calcGlobalAzimuth(float ownshipx, float ownshipy, float intruderx, float intrudery, float ownshipangle, float* global_angle1, float* azimuth_angle){
  *global_angle1;
  *azimuth_angle;
  if(intrudery > ownshipy){ // situation 1
    if(intruderx == ownshipx){ // situation A
      *global_angle1 = 0;
      *azimuth_angle = ownshipangle;
    }
    else if(intruderx > ownshipx){ // situation B
      *global_angle1 = atan(((intruderx - ownshipx))/((intrudery - ownshipy)));
      if(ownshipangle < 0 && ownshipangle > (*global_angle1 - M_PI)){
         *azimuth_angle = -1* ( fabs(ownshipangle) + *global_angle1) ;
      }
      else if(ownshipangle>0 && ownshipangle < *global_angle1){
         *azimuth_angle = -1 * (*global_angle1 - ownshipangle);
      }
      else if(ownshipangle < (*global_angle1 - M_PI) && ownshipangle > -1*M_PI){
         *azimuth_angle = 2*M_PI - *global_angle1 - fabs(ownshipangle);
      }
      else{
         *azimuth_angle = (ownshipangle) - *global_angle1;
      }
    }
    else if(intruderx < ownshipx){
      *global_angle1 =  atan((intruderx - ownshipx)/(intrudery - ownshipy));
      if(ownshipangle > 0 && ownshipangle < (*global_angle1 + M_PI)){
         *azimuth_angle = fabs(*global_angle1) + ownshipangle;
      }
      else if(ownshipangle < 0 && ownshipangle > *global_angle1){
         *azimuth_angle = fabs(*global_angle1) - fabs(ownshipangle);
      }
      else if(ownshipangle>(*global_angle1 + M_PI) && ownshipangle < M_PI){
         *azimuth_angle = -1 * (2*M_PI - ownshipangle-fabs(*global_angle1));
      }
      else{
         *azimuth_angle = -1 * (fabs(ownshipangle)-fabs(*global_angle1));
      }
    }
  }
  else if(intrudery < ownshipy){ // situation 2
    if(intruderx == ownshipx){ // situation A
      *global_angle1 = M_PI;
      if(ownshipangle > 0){
        *azimuth_angle = -1 *  (*global_angle1 - fabs(ownshipangle));
      }
      else{
        *azimuth_angle = (*global_angle1 - fabs(ownshipangle));
      }
    }
    else if(intruderx > ownshipx){  // situation B
      *global_angle1 = M_PI -(-1)*atan((intruderx - ownshipx)/(intrudery - ownshipy));
      if(ownshipangle > 0 && ownshipangle < *global_angle1){
        *azimuth_angle = -1 * (*global_angle1 - ownshipangle);
      }
      else if(ownshipangle < M_PI && ownshipangle > *global_angle1){
        *azimuth_angle = (ownshipangle - *global_angle1);
      }
      else if(ownshipangle < 0 && ownshipangle > (*global_angle1 - M_PI)){
        *azimuth_angle = ownshipangle - *global_angle1;
      }
      else{
        *azimuth_angle = M_PI - *global_angle1 + M_PI + ownshipangle;
      }
    }
    else if(intruderx < ownshipx){ // situation C
      *global_angle1 =  -1 * (M_PI - atan((intruderx - ownshipx)/(intrudery - ownshipy)));
      if(ownshipangle < 0 && ownshipangle > *global_angle1){
         *azimuth_angle = ownshipangle - *global_angle1;
      }
      else if(ownshipangle> - M_PI && ownshipangle < *global_angle1){
         *azimuth_angle =   ownshipangle - *global_angle1;
      }
      else if(ownshipangle>0 && ownshipangle<(*global_angle1 + M_PI)){
         *azimuth_angle = (fabs(*global_angle1) + (ownshipangle));
      }
      else{
         *azimuth_angle = -1*(2*M_PI + *global_angle1 - (ownshipangle));
      }
    }
  }
  else if(intrudery == ownshipy){ // situation 3
    if(intruderx > ownshipx){
       *global_angle1 = 0.5*M_PI;
        if(ownshipangle > 0 && ownshipangle < *global_angle1){
           *azimuth_angle = -1*(fabs(*global_angle1) - fabs(ownshipangle));
          }
        else if(ownshipangle < M_PI && ownshipangle > *global_angle1){
           *azimuth_angle = -1*(fabs(ownshipangle) - fabs(*global_angle1));
          }
        else if(ownshipangle> - M_PI && ownshipangle < -90){
           *azimuth_angle = M_PI-fabs(ownshipangle)-fabs(*global_angle1);
          }
        else{
           *azimuth_angle = fabs(ownshipangle) + fabs(*global_angle1);
          }
        }
    else if (intruderx < ownshipx){ /* situation 8 */
       *global_angle1 = -0.5 * M_PI;
        if(ownshipangle<0 && ownshipangle>*global_angle1){
           *azimuth_angle = fabs(*global_angle1) - fabs(ownshipangle);
          }
        else if(ownshipangle > -M_PI && ownshipangle < *global_angle1){
           *azimuth_angle = fabs(ownshipangle) - fabs(*global_angle1);
          }
        else if(ownshipangle < M_PI && ownshipangle > M_PI * 0.5){
           *azimuth_angle = -1* (M_PI - fabs(ownshipangle)-fabs(*global_angle1));
          }
        else{
           *azimuth_angle = -1*(fabs(ownshipangle) + fabs(*global_angle1));
          }
        }
    else if (intruderx == ownshipx){
       *global_angle1 = 0;
       *azimuth_angle = 0;
    }
  }
}

void calcAvoidanceDist(float lala, float rpz, float ownshipangle_rad, float* d_avo1, float* x_inc, float* y_inc){
  //printf("inside module %f %f %f\n",lala,rpz,ownshipangle_rad);
  *d_avo1 = rpz/sin(lala);
  float d_avo2 = *d_avo1 * tan(lala);
  float d_avot = sqrt((*d_avo1)*(*d_avo1) + d_avo2*d_avo2);
  int avoidsituation;
  float ownshipangle2;

  if (ownshipangle_rad > 0 && ownshipangle_rad <= 0.5*M_PI){
    //printf("drone%d: situation 1\n", AC_ID);
    avoidsituation = 1;
    ownshipangle2 = ownshipangle_rad;
  }
  else if(ownshipangle_rad > 0.5*M_PI && ownshipangle_rad <= M_PI){
    //printf("drone%d: situation 2\n", AC_ID);
    avoidsituation = 2;
    ownshipangle2 = ownshipangle_rad - 0.5 * M_PI;
  }
  else if(ownshipangle_rad <= -0.5*M_PI && ownshipangle_rad > -1* M_PI){
    //printf("drone%d: situation 3\n", AC_ID);
    avoidsituation = 3;
    ownshipangle2 = ownshipangle_rad +  M_PI;
  }
  else if(ownshipangle_rad <= 0 && ownshipangle_rad > -0.5*M_PI){
    //printf("drone%d: situation 4\n", AC_ID);
    avoidsituation = 4;
    ownshipangle2 = ownshipangle_rad + 0.5 * M_PI;
  }
  else{
    //printf("drone%d: no decision\n", AC_ID);
  }

  float beta = ownshipangle2 + lala;

  if (beta < 0.5*M_PI){

    if(avoidsituation == 1){
      //printf("111");
      *x_inc = sin(beta)*d_avot;
      *y_inc = cos(beta)*d_avot;
      *x_inc = *x_inc*1.2;
      *y_inc = *y_inc*1.2;
    }
    else if(avoidsituation == 2){
      //printf("112 ownshipangle 2 %f lala %f beta %f",ownshipangle2/M_PI*180, lala/M_PI*180, beta/M_PI*180);
      *x_inc = cos(beta)*d_avot;
      *y_inc = sin(beta)*d_avot;
      *x_inc = *x_inc*1.2;
      *y_inc = -1*(*y_inc)*1.2;
    }
    else if(avoidsituation == 3){
      //printf("113");
      *x_inc = sin(beta)*d_avot;
      *y_inc = cos(beta)*d_avot;
      *x_inc = -1*(*x_inc)*1.2;
      *y_inc = -1*(*y_inc)*1.2;
    }
    else if(avoidsituation == 4){
      //printf("114");
      *x_inc = cos(beta)*d_avot;
      *y_inc = sin(beta)*d_avot;
      *x_inc = -1*(*x_inc)*1.2;
      *y_inc = *y_inc*1.2;
    }
  }
  else if(beta == 0.5*M_PI){
    printf("drone%d: situation A\n", AC_ID);
    if(avoidsituation == 1){
      //printf("121");
      *x_inc = -2*rpz*1.2;
      *y_inc = 0;
    }
    else if(avoidsituation == 2){
      //printf("122");
      *x_inc = 0;
      *y_inc = -2*rpz*1.2;
    }
    else if(avoidsituation == 3){
      //printf("123");
      *x_inc = 2*rpz*1.2;
      *y_inc = 0;
    }
    else if(avoidsituation == 4){
      //printf("124");
      *x_inc = 0;
      *y_inc = 2*rpz*1.2;
    }
  }
  else if(beta > 0.5*M_PI){
    printf("drone%d: situation C\n", AC_ID);
    float gamma = M_PI - beta;
    if(avoidsituation == 1){
      //printf("131 %f %f %f %f %f %f\n",ownshipangle2, lala, beta,gamma, *x_inc, *y_inc);
      *x_inc = sin(gamma)*d_avot;
      *y_inc = cos(gamma)*d_avot;
      *x_inc = (*x_inc)*1.2;
      *y_inc = -1*(*y_inc)*1.2;
    }
    else if(avoidsituation == 2){
      //printf("132");
      *x_inc = cos(gamma)*d_avot;
      *y_inc = sin(gamma)*d_avot;
      *x_inc = -1*(*x_inc)*1.2;
      *y_inc = -1*(*y_inc)*1.2;
    }
    else if(avoidsituation == 3){
      //printf("133");
      *x_inc = sin(gamma)*d_avot;
      *y_inc = cos(gamma)*d_avot;
      *x_inc = -1*(*x_inc)*1.2;
      *y_inc = (*y_inc)*1.2;
    }
    else if(avoidsituation == 4){
      //printf("134");
      *x_inc = cos(gamma)*d_avot;
      *y_inc = sin(gamma)*d_avot;
      *x_inc = (*x_inc)*1.2;
      *y_inc = (*y_inc)*1.2;
    }
  }
}
