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

int avoid_detection2()
{
  /* To construct the package of incoming data */
  int ac_id2 = 201;
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
  
  float ownship_dronex = stateGetPositionEnu_f()->x;
  float ownship_droney = stateGetPositionEnu_f()->y;
  printf("ardrone 2\n");  
  printf("207: Pos x %f\n", ownship_dronex);
  printf("207: Pos y %f\n", ownship_droney);
  printf("aircraft intruder id is %d\n",intruder.ac_id);
  printf("201: Pos x %f\n",intruder.east);
  printf("201: Pos y %f\n",intruder.north);  
  printf("201: C %f\n",intruder.course);
  printf("201: V %f\n",intruder.gspeed); 

  /* Avoidance detection */
  float d_oi = sqrt((ownship_dronex - intruder.east)*(ownship_dronex - intruder.east) + (ownship_droney - intruder.north)*(ownship_droney - intruder.north));
  printf("distance between the two %f\n",d_oi);
    
  float rpz = 0.5; 
  float d_avo = 0.75; 
  
  if (d_oi > rpz){
    float d_vo = (d_oi*d_oi - rpz*rpz)/d_oi;
    float r_vo = rpz * (sqrt(d_oi*d_oi - rpz*rpz))/d_oi;
    float theta_vo = atan(r_vo/d_vo);
    float azimuth;
    
    if(intruder.east - ownship_dronex != 0){
      float azimuth = atan(abs(intruder.north - ownship_droney)/abs(intruder.east - ownship_dronex));
    }
    else{
      float azimuth = 0;
    }
  
    float DD_oi[2];
    DD_oi[0] = d_vo * cos(azimuth);
    DD_oi[1] = d_vo * sin(azimuth);

    float BBB = (0 + 1) * DD_oi[0] + (1 - 0) * DD_oi[1];
    float CCC = 1*d_vo;
    float avoid_angle = acos(BBB/(CCC*d_vo));
    
    if (avoid_angle < theta_vo && BBB > 0){
      if (d_oi < d_avo){
	printf("colliding");
	return(1);
      }
      else{
	printf("not colliding yet");
      }
    }
    else{
      printf("safe");
    }
  }
  else{ printf("too close");} 

  printf(" \n");
  printf(" \n");
  printf(" \n");
  return 0;
}