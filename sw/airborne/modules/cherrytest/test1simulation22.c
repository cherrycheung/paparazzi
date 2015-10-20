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

#include "cherrytest/test1simulation22.h"
#include "state.h"
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"
#include "math.h"
#include "subsystems/navigation/traffic_info.h"
#include "subsystems/gps.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

void avoidancesim2()
{
  struct ac_info_ * intr = get_ac_info(201);
  int aircraftinfo1 = intr->ac_id;
  float aircraftinfo2 = intr->course;
  float aircraftinfo3 = intr->east;
  float aircraftinfo4 = intr->north;      
  float aircraftinfo5 = intr->alt;
  float aircraftinfo6 = intr->itow;    
  float aircraftinfo7 = intr->gspeed;
  float aircraftinfo8 = intr->climb;
  
  printf("ardrone22\n");  
  printf("207 Position east %f\n", stateGetPositionEnu_f()->x);
  printf("207 Position north %f\n", stateGetPositionEnu_f()->y);
  /*
  printf("Speed x %f\n", stateGetSpeedEnu_f()->x);
  printf("Speed y %f\n", stateGetSpeedEnu_f()->y); */

  float testvaluex = aircraftinfo3 - 594534.812500;
  float testvaluey = aircraftinfo4 - 5760891.500000;
  printf("201 Position east %f\n",testvaluex);
  printf("201 Position north %f\n",testvaluey);  
  printf(" \n");
  
  /*
  intruder_drone[i][0] = 0.0000;
  intruder_drone[i][1] = -2.50000;
  ownship_drone[i][0] = stateGetPositionEnu_f()->x;
  ownship_drone[i][1] = stateGetPositionEnu_f()->y;
  
  
  double speed_drone = 0.5;
  double timestep = 0.25;
  double intruder_drone[40][2];
  float ownship_drone[40][2];
  int i;

  for (i = 0; i<40; i++){
    printf( "i = %d\n",i);
    wait(timestep);
    if(i==0){
      intruder_drone[i][0] = 0.0000;
      intruder_drone[i][1] = -2.50000;
      ownship_drone[i][0] = stateGetPositionEnu_f()->x;
      ownship_drone[i][1] = stateGetPositionEnu_f()->y;
    }
    else
    {
      intruder_drone[i][0] = 0.0000;
      intruder_drone[i][1] = intruder_drone[i-1][1] + speed_drone*timestep;
      ownship_drone[i][0] = stateGetPositionEnu_f()->x;
      ownship_drone[i][1] = stateGetPositionEnu_f()->y;
    }
    printf("position of drone [%d] = %f %f\n",i,intruder_drone[i][1],ownship_drone[i][1]);
      printf("Position y %f\n", stateGetPositionEnu_f()->y);
    
  }*/
  return 0;
  
  

}
