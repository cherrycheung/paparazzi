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
#include "subsystems/datalink/telemetry.h"

int valueofdetection1 = 0;
int valueofnavigation1 = 0;
float d_avo = 0;
float new_waypoint_x = 0;
float new_waypoint_y = 0;
int simulation = 0;

struct uav ownship;
struct uav intruder;
struct data relative;
struct data2 init2;


int function_init(){
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
}

void getRelative(){
  simulation = function_init();
  getOwnship();
  getIntruder(simulation);
  relative.distance = sqrt(powf((ownship.pos_x - intruder.pos_x),2) + powf((ownship.pos_y - intruder.pos_y),2));
  calcGlobalAzimuth(ownship.pos_x, ownship.pos_y, intruder.pos_x, intruder.pos_y, ownship.direction, &relative.global_o, &relative.azimuth_o);
  calcGlobalAzimuth(intruder.pos_x, intruder.pos_y, ownship.pos_x, ownship.pos_y, intruder.direction, &relative.global_i, &relative.azimuth_i);
}

void printStuff(){
	printf("drone%d: relative distance %f \n",ownship.id,relative.distance);
}

int avoid_detection1(){
  int userow = 1; // 1 for row
  init2.rpz = 1.0;
  init2.factor = 1.0;
  init2.avoidance = 35.0/180.0 * M_PI;
  float add_avoidancedistance = 0.3;

  // Avoidance module
  /*float d_vo = (relative.distance*relative.distance - init2.rpz*init2.rpz)/relative.distance;
  float r_vo = init2.rpz*((sqrt(relative.distance*relative.distance - init2.rpz*init2.rpz))/relative.distance);
  float alpha_vo = atan(r_vo/d_vo);*/
  float alpha_vo2 = asin(init2.rpz/relative.distance);
  if (relative.distance < init2.rpz){
	  alpha_vo2 = 0.25*M_PI;
  }

  float Rx = ownship.speed_x - intruder.speed_x;
  float Ry = ownship.speed_y - intruder.speed_y;
  float delta;
  float gamma = 0;
  int teller = 0;
  float boundary_1;
  float boundary_2;
  
  printf("drone%d: speed %f %f %f %f %f %f",ownship.id,ownship.speed_x,ownship.speed_y,intruder.speed_x,intruder.speed_y,Rx,Ry);
  if (relative.global_o > 0 && relative.global_o < 0.5*M_PI){
	  boundary_1 = -0.1*M_PI;
	  boundary_2 = 0.6*M_PI;
	  if(Rx > 0 && Ry > 0 && ownship.direction > boundary_1 && ownship.direction < boundary_2){
		  printf(" case 1\n");
		  teller = 1;
		  delta = atan(Rx/Ry);
		  gamma = delta - 0.5*M_PI + relative.global_o;
	  }
  }
  else if(relative.global_o > 0.5*M_PI && relative.global_o < M_PI){
	  boundary_1 = 0.6*M_PI;
	  boundary_2 = M_PI;	  
	  if(Rx > 0 && Ry < 0 && ownship.direction > boundary_1 && ownship.direction < boundary_2){
		  printf(" case 2\n");
		  teller = 1;
		  delta = 0.5*M_PI + atan(Rx/((-1)*Ry));
		  gamma = relative.global_o - delta;
	  }
  }
  else if(relative.global_o > -1*M_PI && relative.global_o < -0.5*M_PI){
	  boundary_1 = -1*M_PI;
	  boundary_2 = -0.4*M_PI;	  
	  if(Rx < 0 && Ry < 0 && ownship.direction > boundary_1 && ownship.direction < boundary_2){
		  printf(" case 3\n");
		  teller = 1;
		  delta = -1*(0.5*M_PI + atan(Rx/Ry));
		  gamma = -1*relative.global_o - (-1)*delta;
	  }
  }
  else if(relative.global_o > -0.5*M_PI && relative.global_o < 0){
	  boundary_1 = -0.6*M_PI;
	  boundary_2 = 0.1*M_PI;
	  if(Rx < 0 && Ry > 0 && ownship.direction > boundary_1 && ownship.direction < boundary_2){
		  printf(" case 4\n");
		  teller = 1;
		  delta = atan((-1*Rx)/Ry);
		  gamma = delta - 0.5*M_PI + (-1)*relative.global_o;
	  }
  }  
 

  printf("drone%d:%f %f %f %f %d\n",ownship.id,delta,relative.global_o,gamma,alpha_vo2,teller);
  if(gamma < 0){
	  gamma = gamma*-1;
  }

  calcAvoidanceDist(init2.avoidance, init2.rpz, relative.global_o, &d_avo, &new_waypoint_x, &new_waypoint_y);
  
  new_waypoint_x = new_waypoint_x;
  new_waypoint_y = new_waypoint_y;
  
  // Right of way
  int row_zone = 0;
  if (userow == 1){
	  calcROWzone(ownship.direction, intruder.direction, &row_zone);
	  //printf("drone%d: row zone %d\n",ownship.id,row_zone);
  }

  if (relative.distance > init2.rpz){
    if(gamma < alpha_vo2 && teller == 1){
      printf("drone%d: inside VO but not close enough\n", ownship.id);
      if (relative.distance < (d_avo + add_avoidancedistance)){
        printf("drone%d: smaller than d_avo and inside VO\n", ownship.id);
        if(userow == 0){
          valueofdetection1 = 1;
          printf("drone%d: AVOID \n", ownship.id);
        }
        else if (userow == 1){
          if (row_zone == 1 || row_zone == 3 || row_zone == 4){
            valueofdetection1 = 1;
            if (row_zone == 1){
            	printf("drone%d: same path, NO ROW \n", ownship.id);
            }
            else if (row_zone == 3){
            	printf("drone%d: head on, NO ROW \n", ownship.id);
            }
            else if (row_zone == 4){
            	printf("drone%d: left converging, NO ROW \n", ownship.id);
            }

          }
          else{
        	  printf("drone%d: right converging, ROW \n", ownship.id);
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

void calcAvoidanceDist(float lala, float rpz, float globalangle_rad,float* d_avo1, float* x_inc, float* y_inc){
  *d_avo1 = rpz/sin(lala);
  float addavoid = 0.1;
  float d_avo2 = *d_avo1 * tan(lala);
  float d_avot = sqrt((*d_avo1)*(*d_avo1) + d_avo2*d_avo2);
  int avoidsituation = 0;
  float gamma_angle = 0;
  float beta;

  if (globalangle_rad > 0 && globalangle_rad <= 0.5*M_PI){
    avoidsituation = 1;
    printf("SITUATION1\n");
  }
  else if(globalangle_rad > 0.5*M_PI && globalangle_rad <= M_PI){
    avoidsituation = 2;
    printf("SITUATION2\n");
  }
  else if(globalangle_rad <= -0.5*M_PI && globalangle_rad > -1* M_PI){
    avoidsituation = 3;
    printf("SITUATION3\n");
  }
  else if(globalangle_rad <= 0 && globalangle_rad > -0.5*M_PI){
    avoidsituation = 4;
    printf("SITUATION4\n");
  }

  
  if (beta < 0.5*M_PI){
	  if(avoidsituation == 1){
		  beta = globalangle_rad + lala;
		  gamma_angle = 0.5*M_PI-beta;
		  *x_inc = cos(gamma_angle)*d_avot+addavoid;
		  *y_inc = sin(gamma_angle)*d_avot;
	  }
	  else if(avoidsituation == 2){
		  beta = globalangle_rad + lala-0.5*M_PI;
		  gamma_angle = 0.5*M_PI-beta;
		  *x_inc = sin(gamma_angle)*d_avot+addavoid;
		  *y_inc = -cos(gamma_angle)*d_avot;
    }
    else if(avoidsituation == 3){
    	beta = globalangle_rad + lala + M_PI;
    	gamma_angle = 0.5*M_PI-beta;
    	printf("SITUATION3a\n");
		  *x_inc = -cos(gamma_angle)*d_avot-addavoid;
		  *y_inc = -sin(gamma_angle)*d_avot;
    }
    else if(avoidsituation == 4){
    	beta = globalangle_rad + lala + 0.5*M_PI;
    	gamma_angle = 0.5*M_PI-beta;
		  *x_inc = -sin(gamma_angle)*d_avot-addavoid;
		  *y_inc = cos(gamma_angle)*d_avot;
    }
  }
  else if(beta == 0.5*M_PI){
    if(avoidsituation == 1){
      *x_inc = d_avot+addavoid;
      *y_inc = 0;
    }
    else if(avoidsituation == 2){
      *x_inc = 0;
      *y_inc = -d_avot-addavoid;
    }
    else if(avoidsituation == 3){
      *x_inc = -d_avot-addavoid;
      *y_inc = 0;
    }
    else if(avoidsituation == 4){
      *x_inc = 0;
      *y_inc = d_avot+addavoid;
    }
  }
  else if(beta > 0.5*M_PI){
    if(avoidsituation == 1){
    	beta = globalangle_rad + lala;
    	gamma_angle = M_PI-beta;
    	*x_inc = sin(gamma_angle)*d_avot+addavoid;
		*y_inc = -cos(gamma_angle)*d_avot;
    }
    else if(avoidsituation == 2){
    	beta = globalangle_rad + lala-0.5*M_PI;
    	gamma_angle = M_PI-beta;
    	*x_inc = -cos(gamma_angle)*d_avot+addavoid;
		*y_inc = -sin(gamma_angle)*d_avot;
    }
    else if(avoidsituation == 3){
    	beta = globalangle_rad + lala + M_PI;
    	gamma_angle = M_PI-beta;
    	printf("SITUATION3c\n");
        *x_inc = -sin(gamma_angle)*d_avot-addavoid;
        *y_inc = cos(gamma_angle)*d_avot;
    }
    else if(avoidsituation == 4){
    	beta = globalangle_rad + lala + 0.5*M_PI;
    	gamma_angle = M_PI-beta;
    	*x_inc = cos(gamma_angle)*d_avot-addavoid;
		*y_inc = sin(gamma_angle)*d_avot;
    }
  }
}

void calcROWzone(float odir, float idir, int* rowzone){
  float boundaries[4];
  float midboundaries[4];
  
  if(odir > 2*M_PI){
	  odir = odir - 2*M_PI;
  }
  else if(odir < 0){
	  odir = odir + 2*M_PI;
  }
  if(idir > 2*M_PI){
	  idir = idir - 2*M_PI;
  }
  else if(idir < 0){
	  idir = idir + 2*M_PI;
  }
  
  
  boundaries[0] = odir - 0.25*M_PI;
  if(boundaries[0] > 2*M_PI){
    boundaries[0] = boundaries[0] - 2*M_PI;
  }
  else if(boundaries[0] < 0){
    boundaries[0] = boundaries[0] + 2*M_PI;
  }

  boundaries[1] = boundaries[0] + 0.5*M_PI;
  if(boundaries[1] > 2*M_PI){
    boundaries[1] = boundaries[1] - 2*M_PI;
  }
  else if(boundaries[1] < 0){
    boundaries[1] = boundaries[1] + 2*M_PI;
  }

  boundaries[2] = boundaries[1] + 0.5*M_PI;
  if(boundaries[2] > 2*M_PI){
    boundaries[2] = boundaries[2] - 2*M_PI;
  }
  else if(boundaries[2] < 0){
    boundaries[2] = boundaries[2] + 2*M_PI;
  }

  boundaries[3] = boundaries[2] + 0.5*M_PI;
  if(boundaries[3] > 2*M_PI){
    boundaries[3] = boundaries[3] - 2*M_PI;
  }
  else if(boundaries[3] < 0){
    boundaries[3] = boundaries[3] + 2*M_PI;
  }
  
  midboundaries[0] = odir;
  midboundaries[1] = midboundaries[0] + 0.5*M_PI;
  if(midboundaries[1] > 2*M_PI){
	  midboundaries[1] = midboundaries[1] - 2*M_PI;
  }
  else if(boundaries[1] < 0){
	  midboundaries[1] = midboundaries[1] + 2*M_PI;
  }
  midboundaries[2] = midboundaries[1] + 0.5*M_PI;
  if(midboundaries[2] > 2*M_PI){
	  midboundaries[2] = midboundaries[2] - 2*M_PI;
  }
  else if(boundaries[2] < 0){
	  midboundaries[2] = midboundaries[2] + 2*M_PI;
  }
  midboundaries[3] = midboundaries[2] + 0.5*M_PI;
  if(midboundaries[3] > 2*M_PI){
	  midboundaries[3] = midboundaries[3] - 2*M_PI;
  }
  else if(boundaries[3] < 0){
	  midboundaries[3] = midboundaries[3] + 2*M_PI;
  }
  
  //printf("drone%d: row zone boundaries %f %f %f %f own dir %f intr dir %f\n",ownship.id,boundaries[0],boundaries[1],boundaries[2],boundaries[3],odir,idir);
  
  if(boundaries[1] > boundaries[0]){
	  if(boundaries[2] > boundaries[1]){
		  if(boundaries[3] > boundaries[2]){
			  midboundaries[3] = 0;
		  }
		  else{ ///
			  midboundaries[2] = 0;
		  }
	  }
	  else{
		  midboundaries[1] = 0;
	  }
  }
  else{
	  midboundaries[0] = 0;
  }
  
  if(midboundaries[0] == 0){
	  if(idir > boundaries[0]){
		  *rowzone = 1;
	  }
	  else if(idir > 0 && idir < boundaries[1]){
		  *rowzone = 1;
	  }	
	  else if(idir > boundaries[1] && idir < boundaries[2]){
		  *rowzone = 2;
	  }
	  else if(idir > boundaries[2] && idir < boundaries[3]){
		  *rowzone = 3;
	  }
	  else if(idir > boundaries[3] && idir < boundaries[0]){
		  *rowzone = 4;
	  }
	  else if(idir == boundaries [0]){
		  *rowzone = 1;
	  }
	  else if(idir == boundaries [1]){
		  *rowzone = 2;
	  }
	  else if(idir == boundaries [2]){
		  *rowzone = 3;
	  }
	  else if(idir == boundaries [3]){
		  *rowzone = 4;
	  }
  }
  else if(midboundaries[1] == 0){
	  if(idir > boundaries[1]){
		  *rowzone = 2;
	  }
	  else if(idir > 0 && idir < boundaries[2]){
		  *rowzone = 2;
	  }
	  else if(idir > boundaries[0] && idir < boundaries[1]){
		  *rowzone = 1;
	  }
	  else if(idir > boundaries[2] && idir < boundaries[3]){
		  *rowzone = 3;
	  }
	  else if(idir > boundaries[3] && idir < boundaries[0]){
		  *rowzone = 4;
	  }
	  else if(idir == boundaries [0]){
		  *rowzone = 1;
	  }
	  else if(idir == boundaries [1]){
		  *rowzone = 2;
	  }
	  else if(idir == boundaries [2]){
		  *rowzone = 3;
	  }
	  else if(idir == boundaries [3]){
		  *rowzone = 4;
	  }
  }
  else if(midboundaries[2] == 0){
	  if(idir > boundaries[2]){
		  *rowzone = 3;
	  }
	  else if(idir > 0 && idir < boundaries[3]){
		  *rowzone = 3;
	  }
	  else if(idir > boundaries[0] && idir < boundaries[1]){
		  *rowzone = 1;
	  }
	  else if(idir > boundaries[1] && idir < boundaries[2]){
		  *rowzone = 2;
	  }
	  else if(idir > boundaries[3] && idir < boundaries[0]){
		  *rowzone = 4;
	  }
	  else if(idir == boundaries [0]){
		  *rowzone = 1;
	  }
	  else if(idir == boundaries [1]){
		  *rowzone = 2;
	  }
	  else if(idir == boundaries [2]){
		  *rowzone = 3;
	  }
	  else if(idir == boundaries [3]){
		  *rowzone = 4;
	  }
  }
  else if(midboundaries[3] == 0){
	  if(idir > boundaries[3]){
		  *rowzone = 4;
	  }
	  else if(idir > 0 && idir < boundaries[0]){
		  *rowzone = 4;
	  }
	  else if(idir > boundaries[0] && idir < boundaries[1]){
		  *rowzone = 1;
	  }
	  else if(idir > boundaries[1] && idir < boundaries[2]){
		  *rowzone = 2;
	  }
	  else if(idir > boundaries[2] && idir < boundaries[3]){
		  *rowzone = 3;
	  }
	  else if(idir == boundaries [0]){
		  *rowzone = 1;
	  }
	  else if(idir == boundaries [1]){
		  *rowzone = 2;
	  }
	  else if(idir == boundaries [2]){
		  *rowzone = 3;
	  }
	  else if(idir == boundaries [3]){
		  *rowzone = 4;
	  }
  }
  
}

