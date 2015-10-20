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

#include "cherrytest/test1.h"
#include "state.h"
#include "generated/airframe.h" /* to include the AC_ID */
#include "subsystems/datalink/datalink.h"


int timetesting2()
{
 time_t timer;
  struct tm y2k = {0};
  double seconds;

  y2k.tm_hour = 2;   
  y2k.tm_min = 8; 
  y2k.tm_sec = 0;
  y2k.tm_year = 115; 
  y2k.tm_mon = 9; 
  y2k.tm_mday = 4; /* 4-10-2015 3.08 */

  time(&timer);  /* get current time; same as: timer = time(NULL)  */

  seconds = difftime(timer,mktime(&y2k));
   printf ("%.f", seconds);
  if (seconds < 100) {
    printf("running\n");}
  return 0;
}

void wait(int seconds){
  clock_t end_wait = (clock() + (seconds * CLOCKS_PER_SEC));
  while(clock() < end_wait) {}
};

void timetesting()
{
  time_t go, stop;
  int i;
  go = time(NULL);
  printf("\nStarting countdown... \n\n");
  for (i = 0; i<10; i++){
    printf( "- %d\n",i);
    wait(1);
  }
  stop = time(NULL);
  printf("\nRuntime: %.0f seconds\n", difftime(stop,go));
  return 0;
}



void avoidancesim()
{
	printf("Position x %f\n", stateGetPositionEnu_f()->x);
	printf("Position y %f\n", stateGetPositionEnu_f()->y);
	printf("Position z %f\n", stateGetPositionEnu_f()->z);
	printf("Speed x %f\n", stateGetSpeedEnu_f()->x);
	printf("Speed y %f\n", stateGetSpeedEnu_f()->y);
	printf("Speed z %f\n", stateGetSpeedEnu_f()->z);

	double dummywaypoint1[2] = { 0.00,  -2.75};
	double dummywaypoint2[2] = { 3.00,  -1.00};	
	double dummywaypoint3[2] = {-2.00,   1.00};
	double dummywaypoint4[2] = { 1.00,   2.50};

	double timestepx = 5/17;
	double timestepy = 2/17;

	int i;
	double a[17];
   	for(i = 1; i<17; i = i+1)
   	{
	a[0] = 3;
	a[i] = a[i-1] - timestepx;
 	printf("hello %f\n",a[i]);
 
   	}

   	void timetesting()
{
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
    
  }
  return 0;
}



}
