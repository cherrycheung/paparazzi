/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 */
 
 /* This module creates a flight plan callable function to learn high level guidance via reinforcement learning.  written by Jaime Junell */

#include "rlact.h"
#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME

/** Set the default File logger path to the USB drive for ardrone, other for */
#ifndef FILE_RLACT_PATH
#define FILE_RLACT_PATH "/data/video/usb/"     // for ardrone
//#define FILE_RLACT_PATH "./sw/airborne/modules/rlact/"   // for simulation
#endif

//*************** DECLARE VARIABLES *****************//
// environment and states for RL algorithm
int8_t drow, dcol;
int8_t state_curr, state_next;
int8_t ns_curr, ns_next;//current nectar state 0-5 (base 0)
const int8_t nns = 5;  //last nectar state (base 0)
const int8_t ndim = 6; //number of elements in a dimension = 6 (nrows,ncols, nns)
const int8_t nstates=36; //=ndim*ndim

// flags
int8_t nsflag;  //nectar state flag = nectar full
int8_t hbflag;      //hitbounds flag

// Bellman equation
    double V_old, alphav;
    const double belgam = 0.9;  // gamma for belman equation
    static double V[36][6]= {{0}}; // Value function initialized to zeros
    static int kv[36][6]= {{0}}; // number of times a state has been visited
    static int ka[8]= {0};  //number of times an action has been taken, initialized as 0
  // with ka declared as 'int', as long as the GCS is not stopped, the value will not reinitialize
  // with ka declared as 'static int', same thing.  GCS and server processes can be stopped, but stopping the simulator process, will cause the ka variable to be reinitialized.  
	double reward;

// Storing value function:  Value function, V, is stored in V[36][6] for algorithm purposes.  But each iteration the value function and kv matrices will be written to a file so as to be saved and not as easily rewritten.
    FILE *file_Vfcn, *file_kv;
    FILE *file_reg, *file_regw, *file_actw, *file_act;
    int16_t i, j, k;
    
    char filename_regen[200];
    char filename_act[200];
    char filename_Vfcn[200];
	char filename_kv[200];
	
// for execution of RL in paparazzi
const int16_t del = 80;// distance to move in each action
static int32_t pass;

// policy decisions - just random for now
const int8_t nact = 8; //number of actions possible (NESW + diagonals)
int8_t act;
int16_t eps;  //eps=0 is random, eps=100 is full greedy.

// subfunctions
//int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1)

//int8_t chooseact(uint16_t state_curr_sf2, uint8_t ns_curr_sf2 ,uint8_t nact_sf2, uint16_t eps_sf2)
int8_t a,act_sf2;
//int8_t index[8] = {-1};
double Vact[8], max;


//*********************** FUNCTIONS ***********************//
void rlact_init(void) { 
//initialize variables
printf("init1\n");
	state_curr = 0;
	state_next = 0;
	ns_curr = 0;
	ns_next = 0;
		
	nsflag = 0;
	hbflag = 0;
	
	reward = 0.0;
	
	act= 0; 
	pass=0;
	eps=0;

printf("init2\n");
	srand(time(NULL)); //initialize random number generator just once
printf("init3\n");
	//create filenames for files created
	sprintf(filename_regen, "%sregen_rand.txt", FILE_RLACT_PATH);
	sprintf(filename_act, "%sactions_rand.txt", FILE_RLACT_PATH);
	
	// Just in case file has left over numbers from last time, clear file
	file_actw = fopen(filename_act,"w");
	fclose(file_actw);
	file_regw = fopen(filename_regen,"w");
	fclose(file_regw);

}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool_t rlact_run(uint8_t wpa, uint8_t wpb){


		return FALSE;
}  // end of rlact_run function


/********************* hitsbounds subfunction *************/
/*** determines if boundary is hit within a 6x6 grid environment */
/*** inputs: current state, action *** output: 1/0 integer */
int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1){
    drow = state_curr_sf1 % 6; //= #increments current state is from home in y
	dcol = (int)state_curr_sf1/6;  //= #increments current state is from home in x
    
    int sol = 0;
    switch(act_sf1){  //going north or south?
    	case 1 : case 5 : case 6 :  //north, northwest, northeast
    	if(drow>=(6-1)){sol =1;}
    	break;
    	case 3 : case 7 : case 8 : //south, southeast, southwest
    	if(drow==0){sol =1;}
    	break;
    } //end switch north-south

    switch(act_sf1){  //going east or west?
    	case 2 : case 6 : case 7 : //east, northeast, southeast
    	if(dcol>=(6-1)){sol =1;}
    	break;
    	case 4 : case 5 : case 8 : //west, northwest, southwest
    	if(dcol==0){sol =1;}
    	break;
    } //end switch east-west
    
    if(sol==1){return 1;}
    else{return 0;}
}

/********************* chooseact subfunction *************/
/*** chooses an action between 1-8 to take.  */
/*** input: Value function, policy, current state, nectar state*/
/*** output: action 1-8 (North, east, south, west, NW, NE, SE, SW) */
int8_t chooseact(uint16_t state_curr_sf2, uint8_t ns_curr_sf2 , uint16_t eps_sf2){
//int8_t a,i,act_sf2;
//double Vact[8], max;
//Declare up above instead of inside.  and give them different names so they are not shadowed?
// question: why don't I have to bring in V[36][6], or nact, or ndim?
int8_t index_sf2[8]={-1};  //for some reason I cannot declare this above.

//initialize
a = 0;

if(eps==0){ //random
	act_sf2 = (rand() % nact) + 1; 
}
else{ //ep_greedy or full greedy depending on eps value
	if(eps_sf2<(rand() % 100)){act_sf2 = (rand() % nact) +1;}
	else{
			//for each action fill in goodness value or NaN
		for(a= 1; a < nact+1; a++){
  			hbflag = hitsbounds(state_curr_sf2, a);
			if(hbflag){
				Vact[(a-1)] = 0.0/0.0; //if hits bounds, not an option for movement
				}
			else{
				switch(a){
				case 1 : Vact[0] = V[state_curr_sf2+1][ns_curr_sf2]; break;			//north
				case 2 : Vact[1] = V[state_curr_sf2+ndim][ns_curr_sf2]; break;		//east
				case 3 : Vact[2] = V[state_curr_sf2-1][ns_curr_sf2]; break;			//south
				case 4 : Vact[3] = V[state_curr_sf2-ndim][ns_curr_sf2]; break;  		//west
				case 5 : Vact[4] = V[state_curr_sf2+1-ndim][ns_curr_sf2]; break;		//NW
				case 6 : Vact[5] = V[state_curr_sf2+1+ndim][ns_curr_sf2]; break;		//NE
				case 7 : Vact[6] = V[state_curr_sf2-1+ndim][ns_curr_sf2]; break;		//SE
				case 8 : Vact[7] = V[state_curr_sf2-1-ndim][ns_curr_sf2]; break;		//SW
				}  //switch each state  
			} //else
		} //for each action

//find max value of Vact
max = Vact[0];
for(a=1; a <nact; a++){
	if(isnan(max)){max = Vact[a]; /*printf("\nmax is nan, new max= %f\n",max);*/}
	else if(Vact[a]>max){ max=Vact[a]; }
}

//find all the elements that have max value
i=0;
for(a=0; a<nact; a++){
	if(Vact[a]==max){index_sf2[i] = a; i++; }
}
//there are i elements with the same max value.  Choose randomly between them
a = rand() % i;
act_sf2 = index_sf2[a] + 1;

}  //else if not randomly chosen, then greedy
} // if eps=0(random), else it is ep_greedy

return act_sf2;

}  // end chooseact subfuction
