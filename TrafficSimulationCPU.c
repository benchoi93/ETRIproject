#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <assert.h>

#include "TrafficSim.h"

link *mylink;
vehicle *myveh;
source_cell *mysource;
sink_cell *mysink;
connection_cell *mycon;


/*--------------------------------------------------------------------*/
/// RECENT UPDATES
/// 1. Update header file
/// 2. Update "Setup_Connectioncell"
/// 3. Add "Update_Source, Start_Path"

/// TODO 
/// 4. Update "Relay_numVeh", "Relay_numCF", "Update_nextLink"
/// 5. Add debug functions
/// 5-1. Compare tempArr with CC
/// 6. Source overflow detection 
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Veh(vehicle*, int)
/// @brief   Function that setup the values of struct vehicle.
/// @param   vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Veh(vehicle* v, int numVeh) {
	/// (1) Set vehicle ID, type.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].vehID = i;
		v[i].vehType = 0;
		
		for (int j = 0 ; j < MAX_PATH ; j++) {
			v[i].path[j] = -1;
		}
		v[i].lenPath = 0;
		
		for (int j = 0 ; j < MAX_PATH ; j++) {
			v[i].minTargetLane[j] = -1;
			v[i].maxTargetLane[j] = -1;
		}
		
		v[i].initLink = -1;
		v[i].departTime = i;
	}

	/// vPath[] = list of linkID
	int vPath[200][4] = {
	{0, 1, 6, -2},  {0, 1, -2, -2},  {0, 1, 6, -2},  {0, 1, 4, -2},  {0, 1, 4, -2},
	{0, 1, 4, -2},  {0, 1, 4, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},
	{1, 6, -2, -1}, {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 4, -2, -1}, {1, 4, -2, -1},
	{1, 4, -2, -1}, {1, 4, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1},
	{3, 8, 9, -2},  {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 6, -2, -1}, {3, 6, -2, -1},
	{3, 6, -2, -1}, {3, 6, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1},
	{5, 2, -2, -1}, {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 8, 9, -2},  {5, 8, 9, -2},
	{5, 8, 9, -2},  {5, 8, 9, -2},  {5, 6, -2, -1}, {5, 6, -2, -1}, {5, 6, -2, -1}, 
	{7, 4, -2, -1}, {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 2, -2, -1}, {7, 2, -2, -1},
	{7, 2, -2, -1}, {7, 2, -2, -1}, {7, 8, 3, -2},  {7, 8, 3, -2},  {7, 8, 3, -2},
	{0, 1, 6, -2},  {0, 1, 6, -2},  {0, 1, 6, -2},  {0, 1, 4, -2},  {0, 1, 4, -2},
	{0, 1, 4, -2},  {0, 1, 4, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},
	{1, 6, -2, -1}, {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 4, -2, -1}, {1, 4, -2, -1},
	{1, 4, -2, -1}, {1, 4, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1},
	{3, 8, 9, -2},  {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 6, -2, -1}, {3, 6, -2, -1},
	{3, 6, -2, -1}, {3, 6, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1},
	{5, 2, -2, -1}, {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 8, 9, -2},  {5, 8, 9, -2},
	{5, 8, 9, -2},  {5, 8, 9, -2},  {5, 6, -2, -1}, {5, 6, -2, -1}, {5, 6, -2, -1},
	{7, 4, -2, -1}, {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 2, -2, -1}, {7, 2, -2, -1},
	{7, 2, -2, -1}, {7, 2, -2, -1}, {7, 8, 3, -2},  {7, 8, 3, -2},  {7, 8, 3, -2},
    {0, 1, 6, -2},  {0, 1, -2, -2},  {0, 1, 6, -2},  {0, 1, 4, -2},  {0, 1, 4, -2},
    {0, 1, 4, -2},  {0, 1, 4, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},
    {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 4, -2, -1}, {1, 4, -2, -1},
    {1, 4, -2, -1}, {1, 4, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1},
    {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 6, -2, -1}, {3, 6, -2, -1},
    {3, 6, -2, -1}, {3, 6, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1},
    {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 8, 9, -2},  {5, 8, 9, -2},
    {5, 8, 9, -2},  {5, 8, 9, -2},  {5, 6, -2, -1}, {5, 6, -2, -1}, {5, 6, -2, -1},
    {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 2, -2, -1}, {7, 2, -2, -1},
    {7, 2, -2, -1}, {7, 2, -2, -1}, {7, 8, 3, -2},  {7, 8, 3, -2},  {7, 8, 3, -2},
    {0, 1, 6, -2},  {0, 1, 6, -2},  {0, 1, 6, -2},  {0, 1, 4, -2},  {0, 1, 4, -2},
    {0, 1, 4, -2},  {0, 1, 4, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},  {0, 1, 2, -2},
    {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 6, -2, -1}, {1, 4, -2, -1}, {1, 4, -2, -1},
    {1, 4, -2, -1}, {1, 4, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1}, {1, 2, -2, -1},
    {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 8, 9, -2},  {3, 6, -2, -1}, {3, 6, -2, -1},
    {3, 6, -2, -1}, {3, 6, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1}, {3, 4, -2, -1},
    {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 2, -2, -1}, {5, 8, 9, -2},  {5, 8, 9, -2},
    {5, 8, 9, -2},  {5, 8, 9, -2},  {5, 6, -2, -1}, {5, 6, -2, -1}, {5, 6, -2, -1},
    {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 4, -2, -1}, {7, 2, -2, -1}, {7, 2, -2, -1},
    {7, 2, -2, -1}, {7, 2, -2, -1}, {7, 8, 3, -2},  {7, 8, 3, -2},  {7, 8, 3, -2}
	};
	
	/// (2) Set vehicle path, path length.
	for (int i = 0 ; i < numVeh ; i++) {
		for (int j = 0 ; j < 4 ; j++) {
			v[i].path[j] = vPath[i][j];

			if (v[i].path[j] == -2) v[i].lenPath = j;
		}
	}
	
	/// vMinTL[] = list of minTargetLane
	int vMinTL[200][4] = {
	{0, 0, 0, -1},  {0, 0, -1, -1},  {0, 0, 0, -1},  {0, 1, 0, -1},  {0, 1, 0, -1},
	{0, 1, 0, -1},  {0, 1, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1}, 
	{0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
	{0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, 0, -1},  {1, 0, 0, -1},
	{1, 0, 0, -1},  {1, 0, 0, -1},  {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
	{0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, 0, -1},  {3, 0, 0, -1},  {3, 0, 0, -1},
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 1, 0, -1},  {0, 1, 0, -1},
	{0, 1, 0, -1},  {0, 1, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, 
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, 0, -1},  {1, 0, 0, -1},  
	{1, 0, 0, -1},  {1, 0, 0, -1},  {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
	{0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
	{1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, 0, -1},  {3, 0, 0, -1},  {3, 0, 0, -1},
    {0, 0, 0, -1},  {0, 0, -1, -1},  {0, 0, 0, -1},  {0, 1, 0, -1},  {0, 1, 0, -1},
    {0, 1, 0, -1},  {0, 1, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},
    {0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, 0, -1},  {1, 0, 0, -1},
    {1, 0, 0, -1},  {1, 0, 0, -1},  {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, 0, -1},  {3, 0, 0, -1},  {3, 0, 0, -1},
    {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 1, 0, -1},  {0, 1, 0, -1},
    {0, 1, 0, -1},  {0, 1, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},  {0, 3, 0, -1},
    {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {1, 0, 0, -1},  {1, 0, 0, -1},
    {1, 0, 0, -1},  {1, 0, 0, -1},  {3, 0, -1, -1}, {3, 0, -1, -1}, {3, 0, -1, -1},
    {0, 0, -1, -1}, {0, 0, -1, -1}, {0, 0, -1, -1}, {1, 0, -1, -1}, {1, 0, -1, -1},
    {1, 0, -1, -1}, {1, 0, -1, -1}, {3, 0, 0, -1},  {3, 0, 0, -1},  {3, 0, 0, -1}
	};
		
	/// vMaxTL[] = list of maxTargetLane
	int vMaxTL[200][4] = {
	{3, 0, 3, -1},  {3, 0, -1, -1},  {3, 0, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
	{3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{0, 3, 3, -1},  {0, 3, 3, -1},  {0, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, 3, -1},  {3, 3, 3, -1}, 
	{3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1}, 
	{3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
	{3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1}, 
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{0, 3, 3, -1},  {0, 3, 3, -1},  {0, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, 
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, 3, -1},  {3, 3, 3, -1},
	{3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
	{0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
	{3, 3, -1, -1}, {3, 3, -1, -1}, {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1},
    {3, 0, 3, -1},  {3, 0, -1, -1},  {3, 0, 3, -1},  {3, 3, 3, -1}, {3, 3, 3, -1},
    {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, 3, -1},  {0, 3, 3, -1},  {0, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, 3, -1},  {3, 3, 3, -1},
    {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1},
    {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
    {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, 3, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, 3, -1},  {0, 3, 3, -1},  {0, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, 3, -1},  {3, 3, 3, -1},
    {3, 3, 3, -1},  {3, 3, 3, -1},  {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {0, 3, -1, -1}, {0, 3, -1, -1}, {0, 3, -1, -1}, {3, 3, -1, -1}, {3, 3, -1, -1},
    {3, 3, -1, -1}, {3, 3, -1, -1}, {3, 0, 3, -1},  {3, 0, 3, -1},  {3, 0, 3, -1}
	};

	/// (3) Set vehicle minimum and maximum target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		for (int j = 0 ; j < v[i].lenPath ; j++) {
			v[i].minTargetLane[j] = vMinTL[i][j];
			v[i].maxTargetLane[j] = vMaxTL[i][j];
		}
	}

	/// vPos[] = initLink, departTime
	int vPos[200][2] = {
	{0, 0},  {0, 1},  {0, 2},  {0, 3},  {0, 4},  {0, 5},  {0, 6},  {0, 7},  {0, 8},  {0, 9},
	{1, 10}, {1, 11}, {1, 12}, {1, 13}, {1, 14}, {1, 15}, {1, 16}, {1, 17}, {1, 18}, {1, 19},
	{3, 20}, {3, 21}, {3, 22}, {3, 23}, {3, 24}, {3, 25}, {3, 26}, {3, 27}, {3, 28}, {3, 29},
	{5, 30}, {5, 31}, {5, 32}, {5, 33}, {5, 34}, {5, 35}, {5, 36}, {5, 37}, {5, 38}, {5, 39},
	{7, 40}, {7, 41}, {7, 42}, {7, 43}, {7, 44}, {7, 45}, {7, 46}, {7, 47}, {7, 48}, {7, 49},
	{0, 50}, {0, 51}, {0, 52}, {0, 53}, {0, 54}, {0, 55}, {0, 56}, {0, 57}, {0, 58}, {0, 59},
	{1, 60}, {1, 61}, {1, 62}, {1, 63}, {1, 64}, {1, 65}, {1, 66}, {1, 67}, {1, 68}, {1, 69},
	{3, 70}, {3, 71}, {3, 72}, {3, 73}, {3, 74}, {3, 75}, {3, 76}, {3, 77}, {3, 78}, {3, 79},
	{5, 80}, {5, 81}, {5, 82}, {5, 83}, {5, 84}, {5, 85}, {5, 86}, {5, 87}, {5, 88}, {5, 89},
	{7, 90}, {7, 91}, {7, 92}, {7, 93}, {7, 94}, {7, 95}, {7, 96}, {7, 97}, {7, 98}, {7, 99},
    {0, 0},  {0, 1},  {0, 2},  {0, 3},  {0, 4},  {0, 5},  {0, 6},  {0, 7},  {0, 8},  {0, 9},
    {1, 10}, {1, 11}, {1, 12}, {1, 13}, {1, 14}, {1, 15}, {1, 16}, {1, 17}, {1, 18}, {1, 19},
    {3, 20}, {3, 21}, {3, 22}, {3, 23}, {3, 24}, {3, 25}, {3, 26}, {3, 27}, {3, 28}, {3, 29},
    {5, 30}, {5, 31}, {5, 32}, {5, 33}, {5, 34}, {5, 35}, {5, 36}, {5, 37}, {5, 38}, {5, 39},
    {7, 40}, {7, 41}, {7, 42}, {7, 43}, {7, 44}, {7, 45}, {7, 46}, {7, 47}, {7, 48}, {7, 49},
    {0, 50}, {0, 51}, {0, 52}, {0, 53}, {0, 54}, {0, 55}, {0, 56}, {0, 57}, {0, 58}, {0, 59},
    {1, 60}, {1, 61}, {1, 62}, {1, 63}, {1, 64}, {1, 65}, {1, 66}, {1, 67}, {1, 68}, {1, 69},
    {3, 70}, {3, 71}, {3, 72}, {3, 73}, {3, 74}, {3, 75}, {3, 76}, {3, 77}, {3, 78}, {3, 79},
    {5, 80}, {5, 81}, {5, 82}, {5, 83}, {5, 84}, {5, 85}, {5, 86}, {5, 87}, {5, 88}, {5, 89},
    {7, 90}, {7, 91}, {7, 92}, {7, 93}, {7, 94}, {7, 95}, {7, 96}, {7, 97}, {7, 98}, {7, 99}
	};
	
	/// (4) Set vehicle initial position.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].initLink = vPos[i][0];
		v[i].departTime = vPos[i][1];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Link(link*, int, vehicle*, int)
/// @brief   Function that setup the values of struct link, 
///          source_cell, sink_cell.
/// @param   vehicle* v, int numVeh, link* l, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Link(vehicle* v, int numVeh, link* l, source_cell* sc, sink_cell* sk, int numLink) {
	/// (1) Setup link ID, characteristics and initialize values.
	for (int i = 0 ; i < numLink ; i++) {
		l[i].linkID = i;

		l[i].ffSpeed = FFSPEED * 1000 / 3600;

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
			l[i].lenSection[sect] = 100;

			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].maxNumVeh[sect][lane] = JAMDENSITY * l[i].lenSection[sect] / 1000;
				l[i].maxNumCF[sect][lane] = 3;

				l[i].numVehArr[sect][lane] = 0;

				for (int j = 0 ; j < MAX_VEC ; j++) {
			    	l[i].vehIDArr[sect][lane][j] = -1;
	    			l[i].currLinkOrderArr[sect][lane][j] = -1;
	    			l[i].nextLinkIDArr[sect][lane][j] = -1;
	    			l[i].minTargetLaneArr[sect][lane][j] = -1;
	    			l[i].maxTargetLaneArr[sect][lane][j] = -1;
				}

				l[i].speed[sect][lane] = l[i].ffSpeed;
                l[i].density[sect][lane] = 0;

				l[i].numMLCL[sect][lane] = 0;
	    		l[i].numMLCR[sect][lane] = 0;
	    		l[i].numCF[sect][lane] = 0;

	    		for (int j = 0 ; j < MAX_VEC ; j++) {
	    			l[i].vehMLC[sect][lane][j] = 0;
	    			l[i].vehOLC[sect][lane][j] = 0;
	    			l[i].vehCF[sect][lane][j] = 0;
	    		}
			}
		}

		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			for (int j = 0 ; j < MAX_LEG ; j++) {
				l[i].tempIDArr[lane][j] = -1;
				l[i].tempNumArr[lane][j] = 0;
			}
		}
	}

	/// (2) Setup source/sink cell ID, and initialize values.
	for (int i = 0 ; i < numLink ; i++) {
		sc[i].sourceID = i;

		sc[i].numVeh = 0;
		for (int j = 0 ; j < MAX_SOURCE_VEC ; j++) {
			sc[i].vehIDArr[j] = -1;
            sc[i].currLinkOrderArr[j] = -1;
            sc[i].nextLinkIDArr[j] = -1;
            sc[i].minTargetLaneArr[j] = -1;
            sc[i].maxTargetLaneArr[j] = -1;
		}

		sk[i].sinkID = i;

		sk[i].numVeh = 0;
		for (int j = 0 ; j < MAX_SINK_VEC ; j++) {
			sk[i].vehIDArr[j] = -1;
			sk[i].arrivalTime[j] = -1;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_ConnectionCell(connection_cell*)
/// @brief   Function that setup the values of struct connection_cell.
/// @param   connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_ConnectionCell(connection_cell* cc, int numCC) {
	/// (1) Set connection cell ID and previous, next link, and initialize flags.
	for (int i = 0 ; i < numCC ; i++) {
		cc[i].ccID = i;

		cc[i].prevLinkID = i;
		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			for (int j = 0 ; j < MAX_LEG ; j++) {
				cc[i].nextLinkID[lane][j] = -1;
				cc[i].nextLane[lane][j] = -1;

				cc[i].numVehArr[lane][j] = 0;
				cc[i].numCFArr[lane][j] = 0;
			}

			for (int j = 0 ; j < MAX_VEC ; j++) {
				cc[i].currLinkOrderArr[lane][j] = -1;
				cc[i].nextLinkIDArr[lane][j] = -1;
				cc[i].vehIDArr[lane][j] = -1;
			}
		}
	}

	cc[0].nextLinkID[0][0] = 1;
    cc[0].nextLane[0][0] = 0;
    cc[0].nextLinkID[1][0] = 1;
    cc[0].nextLane[1][0] = 1;
    cc[0].nextLinkID[2][0] = 1;
    cc[0].nextLane[2][0] = 2;
    cc[0].nextLinkID[3][0] = 1;
    cc[0].nextLane[3][0] = 3;
    
    cc[1].nextLinkID[0][0] = 6;
    cc[1].nextLane[0][0] = 0;
    cc[1].nextLinkID[1][0] = 4;
    cc[1].nextLane[1][0] = 1;
    cc[1].nextLinkID[2][0] = 4;
    cc[1].nextLane[2][0] = 2;
    cc[1].nextLinkID[3][0] = 4;
    cc[1].nextLane[3][0] = 3;
    cc[1].nextLinkID[3][1] = 2;
    cc[1].nextLane[3][1] = 3;
    
    cc[2].nextLinkID[0][0] = -2;
    cc[2].nextLinkID[1][0] = -2;
    cc[2].nextLinkID[2][0] = -2;
    cc[2].nextLinkID[3][0] = -2;
    
    cc[3].nextLinkID[0][0] = 8;
    cc[3].nextLane[0][0] = 0;
    cc[3].nextLinkID[1][0] = 6;
    cc[3].nextLane[1][0] = 1;
    cc[3].nextLinkID[2][0] = 6;
    cc[3].nextLane[2][0] = 2;
    cc[3].nextLinkID[3][0] = 6;
    cc[3].nextLane[3][0] = 3;
    cc[3].nextLinkID[3][1] = 4;
    cc[3].nextLane[3][1] = 3;
    
    cc[4].nextLinkID[0][0] = -2;
    cc[4].nextLinkID[1][0] = -2;
    cc[4].nextLinkID[2][0] = -2;
    cc[4].nextLinkID[3][0] = -2;
    
    cc[5].nextLinkID[0][0] = 2;
    cc[5].nextLane[0][0] = 0;
    cc[5].nextLinkID[1][0] = 8;
    cc[5].nextLane[1][0] = 1;
    cc[5].nextLinkID[2][0] = 8;
    cc[5].nextLane[2][0] = 2;
    cc[5].nextLinkID[3][0] = 8;
    cc[5].nextLane[3][0] = 3;
    cc[5].nextLinkID[3][1] = 6;
    cc[5].nextLane[3][1] = 3;
    
    cc[6].nextLinkID[0][0] = -2;
    cc[6].nextLinkID[1][0] = -2;
    cc[6].nextLinkID[2][0] = -2;
    cc[6].nextLinkID[3][0] = -2;
    
    cc[7].nextLinkID[0][0] = 4;
    cc[7].nextLane[0][0] = 0;
    cc[7].nextLinkID[1][0] = 2;
    cc[7].nextLane[1][0] = 1;
    cc[7].nextLinkID[2][0] = 2;
    cc[7].nextLane[2][0] = 2;
    cc[7].nextLinkID[3][0] = 2;
    cc[7].nextLane[3][0] = 3;
    cc[7].nextLinkID[3][1] = 8;
    cc[7].nextLane[3][1] = 3;
    
    cc[8].nextLinkID[0][0] = 9;
    cc[8].nextLane[0][0] = 0;
    cc[8].nextLinkID[1][0] = 9;
    cc[8].nextLane[1][0] = 1;
    cc[8].nextLinkID[2][0] = 9;
    cc[8].nextLane[2][0] = 2;
    cc[8].nextLinkID[3][0] = 9;
    cc[8].nextLane[3][0] = 3;
    
    cc[9].nextLinkID[0][0] = -2;
    cc[9].nextLinkID[1][0] = -2;
    cc[9].nextLinkID[2][0] = -2;
    cc[9].nextLinkID[3][0] = -2;
    
   
	/// (2) Set traffic signals.
	for (int i = 0 ; i < numCC ; i++) {
		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			for (int count = 0 ; count < MAX_CYCLE_DURATION ; count++) {
				cc[i].trafficSignal[lane][count] = 0;
			}
		}
		cc[i].cycleDuration = 16;
	}
    
    //for (int i = 0 ; i < numCC ; numCC++) {
    for (int count = 0 ; count < MAX_CYCLE_DURATION ; count++) {
        
        int trafficCycle = cc[0].cycleDuration;
        int trafficStep = count % trafficCycle;
        
        if (trafficStep < 5) {
            cc[1].trafficSignal[1][count] = 1;
            cc[1].trafficSignal[2][count] = 1;
            cc[1].trafficSignal[3][count] = 1;
            cc[5].trafficSignal[1][count] = 1;
            cc[5].trafficSignal[2][count] = 1;
            cc[5].trafficSignal[3][count] = 1;
        }
        
        else if (trafficStep >= 5 & trafficStep < 8) {
            cc[1].trafficSignal[0][count] = 1;
            cc[5].trafficSignal[0][count] = 1;
        }
        else if (trafficStep >= 8 &trafficStep < 13) {
            cc[3].trafficSignal[1][count] = 1;
            cc[3].trafficSignal[2][count] = 1;
            cc[3].trafficSignal[3][count] = 1;
            cc[7].trafficSignal[1][count] = 1;
            cc[7].trafficSignal[2][count] = 1;
            cc[7].trafficSignal[3][count] = 1;
        }
        else if (trafficStep >= 13 & trafficStep < 16) {
            cc[3].trafficSignal[0][count] = 1;
            cc[7].trafficSignal[0][count] = 1;
        }
        
        cc[0].trafficSignal[0][count] = 1;
        cc[0].trafficSignal[1][count] = 1;
        cc[0].trafficSignal[2][count] = 1;
        cc[0].trafficSignal[3][count] = 1;
        
        cc[2].trafficSignal[0][count] = 1;
        cc[2].trafficSignal[1][count] = 1;
        cc[2].trafficSignal[2][count] = 1;
        cc[2].trafficSignal[3][count] = 1;
        
        cc[4].trafficSignal[0][count] = 1;
        cc[4].trafficSignal[1][count] = 1;
        cc[4].trafficSignal[2][count] = 1;
        cc[4].trafficSignal[3][count] = 1;
        
        cc[6].trafficSignal[0][count] = 1;
        cc[6].trafficSignal[1][count] = 1;
        cc[6].trafficSignal[2][count] = 1;
        cc[6].trafficSignal[3][count] = 1;
        
        cc[8].trafficSignal[0][count] = 1;
        cc[8].trafficSignal[1][count] = 1;
        cc[8].trafficSignal[2][count] = 1;
        cc[8].trafficSignal[3][count] = 1;
        
        cc[9].trafficSignal[0][count] = 1;
        cc[9].trafficSignal[1][count] = 1;
        cc[9].trafficSignal[2][count] = 1;
        cc[9].trafficSignal[3][count] = 1;
        
    }
    //}

}


/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_MLC(link, int)
/// @brief   Function that evaluate Mandatory Lane Change of a vehicle
///          and updates vehMLC Flag.
/// @param   link *l, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
void Evaluate_MLC(link *l) {
 	for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            for (int i = 0 ; i < l->numVehArr[sect][lane] ; i++) {
            	int minTL = l->minTargetLaneArr[sect][lane][i];
	            int maxTL = l->maxTargetLaneArr[sect][lane][i];

	            /// (1) Compare current lane with target lane and determine
	            ///     and update the value of vehMLC Flag.
	            /// If vehicle should move left, set vehMLC Flag to -1
	            if (lane > maxTL) {
	                l->vehMLC[sect][lane][i] = -1;
	                l->numMLCL[sect][lane]++;
	            }
	            /// If vehicle should move left, set vehMLC Flag to +1
	            else if (lane < minTL) {
	                l->vehMLC[sect][lane][i] = 1;
	                l->numMLCR[sect][lane]++;
	            }
	            else {
	            	l->vehMLC[sect][lane][i] = 0;
            	}
            }
        }
    }
}


/*--------------------------------------------------------------------*/
/// @fn      int Evaluate_Prob(double)
/// @brief   Function that randomly returns integer part or 
///          (integer part+1) of a rational number.
/// @param   double inputProb
/// @return  intPart+1 when random > probPart
///          intPart  when random <= probPart
/*--------------------------------------------------------------------*/
int Evaluate_Prob(double inputProb) {
    // int intPart = (int)inputProb;
    // double probPart = inputProb - (double)intPart;

    // double random = ((rand() % 10)/10.);

    // return random > probPart ? (intPart+1):intPart; 

	int intPart = (int)inputProb;
	return intPart;
}


/*--------------------------------------------------------------------*/
/// @fn      void Select_Veh()
/// @brief   Function that select vehicles of Optional Lane Change.
/// @param   link* l, int numOLC_L, int numOLC_R, int sect, int lane
/// @return  None
/*--------------------------------------------------------------------*/
void Select_Veh(link* l, int numOLC_L, int numOLC_R, int sect, int lane) {
	int numVeh = l->numVehArr[sect][lane];
	int possOLC[MAX_VEC] = {0};

	/// (1) Exclude vehMLC from candidates that can OLC.
	for (int i = 0 ; i < numVeh ; i++) {
		if (l->vehMLC[sect][lane][i] != 0) possOLC[i] = 0;
		else possOLC[i] = 2;
	}

	/// (2) Consider when current lane is either maximum or minimum target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		int minTL = l->minTargetLaneArr[sect][lane][i];
        int maxTL = l->maxTargetLaneArr[sect][lane][i];

		if (lane == minTL && lane < maxTL) {
			if (possOLC[i] == 2) possOLC[i] = 1;
			//else if (possOLC[i] == -1) possOLC[i] = 0;
		}
		else if (lane == maxTL && lane > minTL) {
			if (possOLC[i] == 2) possOLC[i] = -1;
			//else if (possOLC[i] == 1) possOLC[i] = 0;
        } else if (lane == minTL && lane == maxTL) {
            if (possOLC[i] == 2) possOLC[i] = 0;
        }
	}

	/// (3) Calculate number of vehicles that can go left, right, or both.
	int possBoth = 0;
	int possLeft = 0;
	int possRight = 0;
	int possLeftArr[MAX_VEC] = {0};
	int possRightArr[MAX_VEC] = {0};

	for (int i = 0 ; i < numVeh ; i++) {
		if (possOLC[i] == 2) {
			possLeftArr[possLeft] = i;
			possRightArr[possRight] = i;
			possLeft++;
			possRight++;
			possBoth++;
		}
		else if (possOLC[i] == -1) {
			possLeftArr[possLeft] = i;
			possLeft++;
		}
		else if (possOLC[i] == 1) {
			possRightArr[possRight] = i;
			possRight++;
		}
	}

	/// (4) Consider when number of OLC is larger than possible vehicle of OLC
	if (possLeft < numOLC_L) numOLC_L = possLeft;
	if (possRight < numOLC_R) numOLC_R = possRight;
	
	int possTotal = possLeft + possRight - possBoth;
	while (possTotal < numOLC_L + numOLC_R) {
		numOLC_R--;
		if (possTotal >= numOLC_L + numOLC_R) break;
		numOLC_L--;
	}

	/// (5) Update values of vehOLC flags.
	int count_R = numOLC_R;
	int count_L = numOLC_L;
    //if (numOLC_L == 0 && numOLC_R == 0); break;
    if (numOLC_L + numOLC_R > 0) {
        if (numOLC_L == 0) {
            while (count_R > 0) {
                int randVeh = rand()%possRight;
                if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
                    count_R--;
                }
            }
        }
        else if (numOLC_R == 0) {
            while (count_L > 0) {
                int randVeh = rand()%possLeft;
                if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
                    count_L--;
                }
            }
        }
        
        else if ((possLeft/numOLC_L) > (possRight/numOLC_R)) {
            while (count_R) {
                int randVeh = rand()%possRight;
                if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
                    count_R--;
                }
            }
            
            while (count_L) {
                int randVeh = rand()%possLeft;
                if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
                    count_L--;
                }
            }
        }
        else if ((possLeft/numOLC_L) <= (possRight/numOLC_R)) {
            while (count_L) {
                int randVeh = rand()%possLeft;
                if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
                    count_L--;
                }
            }
            
            while (count_R) {
                int randVeh = rand()%possRight;
                if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
                    l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
                    count_R--;
                }
            }
        }
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_OLC(link*)
/// @brief   Function that evaluate Optional Lane Change of a vehicle
///          and updates vehOLC Flag.
/// @param   link *l
/// @return  None
/*--------------------------------------------------------------------*/
void Evaluate_OLC(link *l) {
    for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	int numMLC_L = l->numMLCL[sect][lane];
        	int numMLC_R = l->numMLCR[sect][lane];
            double probLC_L, probLC_R;
            double diffSpeed_L = l->speed[sect][lane-1] - l->speed[sect][lane];
            double diffSpeed_R = l->speed[sect][lane+1] - l->speed[sect][lane];

            /// (1) Set probLC to zero in special cases. 
            /// When current lane is leftmost, probLC_L = 0
            /// In other words, the vehicle cannot move left.
            if (lane == 0) probLC_L = 0;
            else probLC_L = (diffSpeed_L / l->ffSpeed) * (double) l->numVehArr[sect][lane];
            
            /// When current lane is rightmost, probLC_R = 0
            /// In other words, the vehicle cannot move right.
            if (lane == NUM_LANE-1) probLC_R = 0;
            else probLC_R = (diffSpeed_R / l->ffSpeed) * (double) l->numVehArr[sect][lane];

            /// (2) Evaluate number of OLC by subtrating number of MLC
            ///     from the total number of LC.
            int numOLC_L = Evaluate_Prob(probLC_L) - numMLC_L;
            int numOLC_R = Evaluate_Prob(probLC_R) - numMLC_R;

            /// numOLC cannot be smaller than zero.
		    if(numOLC_L < 0) numOLC_L = 0;
	        if(numOLC_R < 0) numOLC_R = 0;

		    /// (3) Select vehicle to change lane.
		    Select_Veh(l, numOLC_L, numOLC_R, sect, lane);
        }
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void MoveLC()
/// @brief   Function that moves elements of one array to another array for LC (It should lead the function Remove_Blank).
/// @param   int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index
/// @return  None
/*--------------------------------------------------------------------*/
void MoveLC(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
    
    printf("i: %d     ", index);
    
	double fromArrLoc = ((double) (index+1)/fromArrSize);
    double toArrLoc = (double) toArrSize * fromArrLoc;
    
    //int toIndex = toArrLoc * fromArrSize + 0.5;
    int toIndex = toArrLoc + 0.5;
    
    printf("fromArrLoc: %f, toArrLoc: %f, fromArrSize: %d, toArrSize: %d, fromIndex: %d, toIndex: %d, ", fromArrLoc, toArrLoc, fromArrSize, toArrSize, index, toIndex);
    
    /*
    printf("MOVELC fromArrSize: %d, toArrSize: %d, index: %d fromArrLoc: %f, toArrLoc: %f, toIndex: %d \n", fromArrSize, toArrSize, index, fromArrLoc, toArrLoc, toIndex);
    
    for (int i = 0 ; i < fromArrSize ; i++) {
        printf("fromArr[ %d ] = %d \n", i, fromArr[i]);
    }
    for (int i = 0 ; i < toArrSize+1 ; i++) {
        printf("toArr[ %d ] = %d \n", i, toArr[i]);
    }
    */
    if (toArrSize > 0) {
        for (int i = toArrSize ; i > toIndex ; i--) {
            toArr[i] = toArr[i-1];
        }
    }
    
    toArr[toIndex] = fromArr[index];
    fromArr[index] = -1;
    
    printf("toArr[toIndex]: %d \n", toArr[toIndex]);
    //for (int i = index ; i < fromArrSize ; i++) {
    //    fromArr[i] = fromArr[i+1];
    //}
    
    /*
    printf("  excute\n");
    for (int i = 0 ; i < fromArrSize ; i++) {
        printf("fromArr[ %d ] = %d \n", i, fromArr[i]);
    }
    for (int i = 0 ; i < toArrSize+1 ; i++) {
        printf("toArr[ %d ] = %d \n", i, toArr[i]);
    }
    */
}


/*--------------------------------------------------------------------*/
/// @fn      void Remove_Blank()
/// @brief   Function that removes blank elements in an array.
/// @param   int* Arr, int ArrSize
/// @return  None
/*--------------------------------------------------------------------*/
void Remove_Blank(int* Arr, int ArrSize) {
    if (ArrSize > 0) {
        for (int i = 0 ; i < ArrSize ; i++) {
            int index = Find_Index(Arr, ArrSize, -1);
            //printf("veh : %d , arr[%d]: %d  ", i, i, Arr[i]);
            //if (Arr[i] == -1) {
                //printf("Arr[%d]: %d ", i, Arr[i]);
            if (index != -1) {
                //printf("i: %d, index: %d, vehID: %d, nextVehID: %d \n", i, index, Arr[index], Arr[index+1]);
                for (int j = index ; j < ArrSize ; j++) {
                    Arr[j] = Arr[j+1];
                }
                Arr[ArrSize] = -1;
            }
        }
    }
}

/*--------------------------------------------------------------------*/
/// @fn      void LCSim()
/// @brief   Function to move vehicles that perform lane change.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void LCSim(link* l) {
    int numLC_L[NUM_SECTION+2][NUM_LANE];
    int numLC_R[NUM_SECTION+2][NUM_LANE];
    
    for (int sect = 1 ; sect < NUM_SECTION+1 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            numLC_L[sect][lane] = 0;
            numLC_R[sect][lane] = 0;
        }
    }
    
    //MLC simulation
    printf("MLC \n");
    for (int sect = 1 ; sect < NUM_SECTION+1 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            //printf("sect: %d, lane: %d \n", sect, lane);
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehMLC[sect][lane][i] == 1 && l->numVehArr[sect][lane+1] < MAX_VEC) {
                    printf("direction: 1, veh: %d, from: (%d,%d,%d), to: (%d,%d)  \n", l->vehIDArr[sect][lane][i], sect, lane, i, sect, lane+1);
                    //printf("vehIDArr veh: %d \n", i);
                    MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane],
                        l->vehIDArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("currLinkOrderArr veh: %d \n", i);
                    MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane],
                        l->currLinkOrderArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("nextLinkIDArr veh: %d \n", i);
                    MoveLC(l->nextLinkIDArr[sect][lane], l->numVehArr[sect][lane],
                           l->nextLinkIDArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("minTargetLaneArr veh: %d \n", i);
                    MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane],
                        l->minTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("maxTargetLaneArr veh: %d \n", i);
                    MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane],
                        l->maxTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    l->vehMLC[sect][lane][i] = 0;
                    MoveLC(l->vehMLC[sect][lane], l->numVehArr[sect][lane],
                           l->vehMLC[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    l->numVehArr[sect][lane+1]++;
                    //l->numVehArr[sect][lane]--;
                    numLC_R[sect][lane]++;
	        	} 
	        		
	        	else if (l->vehMLC[sect][lane][i] == -1 && l->numVehArr[sect][lane-1] < MAX_VEC) {
                    printf("direction: -1, veh: %d, from: (%d,%d,%d), to: (%d,%d)  \n", l->vehIDArr[sect][lane][i], sect, lane, i, sect, lane-1);
                    printf("vehIDArr veh: %d \n", i);
                    MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane],
                        l->vehIDArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("currLinkOrderArr veh: %d \n", i);
                    MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane],
                        l->currLinkOrderArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("nextLinkIDArr veh: %d \n", i);
                    MoveLC(l->nextLinkIDArr[sect][lane], l->numVehArr[sect][lane],
                           l->nextLinkIDArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("minTargetLaneArr veh: %d \n", i);
                    MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane],
                        l->minTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("maxTargetLaneArr veh: %d \n", i);
                    MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane],
                        l->maxTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    l->vehMLC[sect][lane][i] = 0;
                    MoveLC(l->vehMLC[sect][lane], l->numVehArr[sect][lane],
                           l->vehMLC[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    l->numVehArr[sect][lane-1]++;
                    //l->numVehArr[sect][lane]--;
                    numLC_L[sect][lane]++;
	        	}
	        }
        }
    }
    
    printf("OLC \n");
    //OLC simulation
    for (int sect = 1 ; sect < NUM_SECTION+1 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            //printf("sect: %d, lane: %d \n", sect, lane);
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehOLC[sect][lane][i] == 1 && l->numVehArr[sect][lane] < MAX_VEC) {
                    printf("direction: 1, veh: %d, from: (%d,%d), to: (%d,%d)  \n", l->vehIDArr[sect][lane][i], sect, lane, sect, lane+1);
                    //printf("direction: 1 \n");
                    //printf("vehIDArr veh: %d \n", i);
	        		MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("currLinkOrderArr veh: %d \n", i);
	        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("nextLinkIDArr veh: %d \n", i);
                    MoveLC(l->nextLinkIDArr[sect][lane], l->numVehArr[sect][lane],
                           l->nextLinkIDArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("minTargetLaneArr veh: %d \n", i);
	        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    //printf("maxTargetLaneArr veh: %d \n", i);
	        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->maxTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
                    l->vehOLC[sect][lane][i] = 0;
                    MoveLC(l->vehOLC[sect][lane], l->numVehArr[sect][lane],
                           l->vehOLC[sect][lane+1], l->numVehArr[sect][lane+1]+1, i);
	        		l->numVehArr[sect][lane+1]++;
	        		//l->numVehArr[sect][lane]--;
                    numLC_R[sect][lane]++;
	        	}
	        	else if (l->vehOLC[sect][lane][i] == -1 && l->numVehArr[sect][lane] < MAX_VEC) {
                    printf("direction: -1, veh: %d, from: (%d,%d), to: (%d,%d)  \n", l->vehIDArr[sect][lane][i], sect, lane, sect, lane-1);
                    //printf("direction: -1 \n");
                    //printf("vehIDArr veh: %d \n", i);
	        		MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("currLinkOrderArr veh: %d \n", i);
	        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("nextLinkIDArr veh: %d \n", i);
                    MoveLC(l->nextLinkIDArr[sect][lane], l->numVehArr[sect][lane],
                           l->nextLinkIDArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("minTargetLaneArr veh: %d \n", i);
	        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    //printf("maxTargetLaneArr veh: %d \n", i);
	        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->maxTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
                    l->vehOLC[sect][lane][i] = 0;
                    MoveLC(l->vehOLC[sect][lane], l->numVehArr[sect][lane],
                           l->vehOLC[sect][lane-1], l->numVehArr[sect][lane-1]+1, i);
	        		l->numVehArr[sect][lane-1]++;
	        		//l->numVehArr[sect][lane]--;
                    numLC_L[sect][lane]++;
	        	}
	        }
        }
    }
    
    for (int sect = 1 ; sect < NUM_SECTION+1 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            if (l->numVehArr[sect][lane] > 0 ) {
                printf("sect: %d, lane: %d \n", sect, lane);
                Remove_Blank(l->vehIDArr[sect][lane], MAX_VEC - 1);
                Remove_Blank(l->currLinkOrderArr[sect][lane], MAX_VEC - 1);
                Remove_Blank(l->nextLinkIDArr[sect][lane], MAX_VEC - 1);
                Remove_Blank(l->minTargetLaneArr[sect][lane], MAX_VEC - 1);
                Remove_Blank(l->maxTargetLaneArr[sect][lane], MAX_VEC - 1);
            }
        }
    }
    
    for (int sect = 1 ; sect < NUM_SECTION+1 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            l->numVehArr[sect][lane] -= (numLC_L[sect][lane] + numLC_R[sect][lane]);
        }
    }
    
    
}


/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_CF()
/// @brief   Function that evaluate CF of a vehicle and updates vehCF Flag.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void Evaluate_CF(link* l) {
	double wSpeed = 4.2;
  
	for (int sect = 0 ; sect < NUM_SECTION ; sect++) {
  		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
      		l->numCF[sect][lane]= 
				MAX( 0, MIN(l->numVehArr[sect][lane], MIN(l->maxNumCF[sect][lane], wSpeed / l->ffSpeed * (l->maxNumVeh[sect+1][lane] - l->numVehArr[sect+1][lane]))) ) * l->ffSpeed * SECONDS_PER_STEP / l->lenSection[sect];
            
            //printf("l->numVehArr[sect][lane]: %d, ");
            //printf("sect: %d, lane: %d, numCF: %f ", sect, lane, l->numCF[sect][lane]);
	      	
            for (int i = 0 ; i < l->numCF[sect][lane] ; i++) {
				l->vehCF[sect][lane][i] = 1;
                //printf("vehicleID: %d ", l->vehIDArr[sect][lane][i]);
      		}
            
            //printf("\n");
    	}
  	}
}


/*--------------------------------------------------------------------*/
/// @fn      void MoveCF()
/// @brief   Function that moves elements of one array to another array.
/// @param   int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index
/// @return  None
/*--------------------------------------------------------------------*/
void MoveCF(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
    
    /*
    printf("MOVECF fromArrSize: %d, toArrSize: %d, index: %d \n", fromArrSize, toArrSize, index);
    for (int i = 0 ; i < fromArrSize ; i++) {
        printf("fromArr[ %d ] = %d \n", i, fromArr[i]);
    }
    for (int i = 0 ; i < toArrSize+1 ; i++) {
        printf("toArr[ %d ] = %d \n", i, toArr[i]);
    }
    */
    toArr[toArrSize] = fromArr[index];
    
    for (int i = index ; i < (fromArrSize-1) ; i++) {
        fromArr[i] = fromArr[i+1];
    }
    fromArr[fromArrSize-1] = -1;
    
    /*
    printf("  excute\n");
    for (int i = 0 ; i < fromArrSize ; i++) {
        printf("fromArr[ %d ] = %d \n", i, fromArr[i]);
    }
    for (int i = 0 ; i < toArrSize+1 ; i++) {
        printf("toArr[ %d ] = %d \n", i, toArr[i]);
    }
    */

}


/*--------------------------------------------------------------------*/
/// @fn      void CFsim()
/// @brief   Function to move vehicles that perform CF.
/// @param   link *l
/// @return  None
/*--------------------------------------------------------------------*/
void CFsim(link *l) {
	for (int sect = NUM_SECTION; sect > 0 ; sect--) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            printf("[ %d , %d ] vehID: ", sect, lane);
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehCF[sect][lane][0] == 1 && l->numVehArr[sect+1][lane] < MAX_VEC) {
                    printf("%d ", l->vehIDArr[sect][lane][0]);
	        		MoveCF(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect+1][lane], l->numVehArr[sect+1][lane], 0);
                    //printf("currLinkOrderArr veh: %d \n", i);
	        		MoveCF(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect+1][lane], l->numVehArr[sect+1][lane], 0);
                    //printf("nextLinkIDArr veh: %d \n", i);
                    MoveCF(l->nextLinkIDArr[sect][lane], l->numVehArr[sect][lane],
                           l->nextLinkIDArr[sect+1][lane], l->numVehArr[sect+1][lane], 0);
                    //printf("minTargetLaneArr veh: %d \n", i);
	        		MoveCF(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect+1][lane], l->numVehArr[sect+1][lane], 0);
                    //printf("maxTargetLaneArr veh: %d \n", i);
	        		MoveCF(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane],
	        			l->maxTargetLaneArr[sect+1][lane], l->numVehArr[sect+1][lane], 0);
                    MoveCF(l->vehCF[sect][lane], l->numVehArr[sect][lane],l->vehCF[sect+1][lane], l->numVehArr[sect+1][lane],0);
	        		l->numVehArr[sect+1][lane]++;
	        		l->numVehArr[sect][lane]--;
	        	}
	        }
            printf("\n");
	    }
	}
}


/*--------------------------------------------------------------------*/
/// @fn      int Find_Index()
/// @brief   Function that find the index of a value in an array.
/// @param   int* findArr, int findArrSize, int findValue, 
/// @return  index if the given value exists in the given array.
///          -1 if the given value doesn't exist in the givne array.
/*--------------------------------------------------------------------*/
int Find_Index(int* findArr, int findArrSize, int findValue) {
	for (int i = 0 ; i < findArrSize ; i++) {
		if (findArr[i] == findValue) return i;
	}

	return -1;
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_Link_TempArr()
/// @brief   Function that update tempIDArr and tempNumArr.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void Update_tempArr(link* l) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        int tempArrSize = 0;
        
        //printf("lane: %d \n", lane);
        for (int i = 0 ; i < l->numVehArr[NUM_SECTION][lane] ; i++) {
        	int tempArrIndex = Find_Index(l->tempIDArr[lane], MAX_LEG, l->nextLinkIDArr[NUM_SECTION][lane][i]);
            
        	if (tempArrIndex == -1) {
        		l->tempIDArr[lane][tempArrSize] = l->nextLinkIDArr[NUM_SECTION][lane][i];
        		l->tempNumArr[lane][tempArrSize] += 1;
        		tempArrSize += 1;
        	}
            else l->tempNumArr[lane][tempArrIndex] += 1;
        }
        /*
        for (int i = 0 ; i < MAX_LEG ; i++) {
            printf("tempIDArr: %d, tempNumArr[%d][%d]: %d \n", l->tempIDArr[lane][i], lane, i, l->tempNumArr[lane][i]);
        }
        */
        
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Relay_numVeh()
/// @brief   Function that relay numVeh of the last cell of a previous 
///          link to the virtual cell of the next link.
/// @param   link* prevl, link* nextl, int nextLane, connection_cell* cc, 
///          int currLane, int i
/// @return  None
/*--------------------------------------------------------------------*/
void Relay_numVeh(link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLane, int leg, int currLink) {
	/// (1) Relay numVeh from tempArr of previous link to next link
	int index = Find_Index(prevl->tempIDArr[currLane], MAX_LEG, cc->nextLinkID[currLane][leg]);

	if (index != -1) {
		/// (1-1) tempArr of previous link -> connection cell
		cc->numVehArr[currLane][leg] = prevl->tempNumArr[currLane][index];
        //printf("index: %d, cc.numVehArr[%d][%d]: %d \n", index, currLane, leg, cc->numVehArr[currLane][leg]);
		/// (1-2) connection cell -> virtual cell of next link
		nextl->numVehArr[0][nextLane] = cc->numVehArr[currLane][leg];
        //printf("index: %d, nextl.numVehArr[0][%d]: %d \n", index, nextLane, nextl->numVehArr[0][nextLane]);
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Relay_numCF()
/// @brief   Function that relay numCF of the virtual cell of a next 
///          link to the last cell of the previous link.
/// @param   link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int i
/// @return  None
/*--------------------------------------------------------------------*/
void Relay_numCF(link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int leg) {
	/// (1) virtual cell of next link -> connection cell
    //printf("signal: %d ", cc->trafficSignal[currLane][currLoop]);
    
    cc->numCFArr[currLane][leg] = nextl->numCF[0][nextLane] * cc->trafficSignal[currLane][currLoop];

	/*
    if (cc->trafficSignal[currLane][currLoop] == 0) {
		cc->numCFArr[currLane][currLoop] = 0;
	}
	else cc->numCFArr[currLane][leg] = nextl->numCF[0][nextLane];
    */
    
	/// (2) connection cell -> previous link
	int index = Find_Index(prevl->tempIDArr[currLane], MAX_LEG, cc->nextLinkID[currLane][leg]);

    if (index != -1) {

        cc->numCFArr[currLane][leg] = ceil(cc->numCFArr[currLane][leg]);
        prevl->tempNumArr[currLane][index] = cc->numCFArr[currLane][leg];
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_Eff_numCF()
/// @brief   Function that evaluates effective numCF.
/// @param   link* l, connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Evaluate_Eff_numCF(link* l) {
    //printf("effNumCF: ");
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		int tempArrSize = 0;
		int totalNumCF = 0;
        
        //printf("lane: %d, ", lane);
		for (int i = 0 ; i < MAX_LEG ; i++) {
			totalNumCF += l->tempNumArr[lane][i];
            //printf("tempNumArr[%d][%d]: %d, ", lane, i, l->tempNumArr[lane][i]);
            //printf("tempIDArr[%d][%d]: %d, ", lane, i, l->tempIDArr[lane][i]);

			if (l->tempIDArr[lane][i] == -1) {
				tempArrSize = i;
				break;
			}
		}
        
        //printf("totalNumCF: %d, tempArrSize: %d, MIN: %f ", totalNumCF, tempArrSize, MIN(totalNumCF, l->maxNumCF[NUM_SECTION][lane]));

		int effNumCF = 0;
		for (int count = 0 ; count < MIN(totalNumCF, l->maxNumCF[NUM_SECTION][lane]) ; count++){
            for (int i = 0 ; i < MAX_LEG ; i++) {
                if (l->tempNumArr[lane][i] == 0) {
                    break;
                }
                else {
                    int currNextLinkID = l->nextLinkIDArr[NUM_SECTION][lane][i];
                    int index = Find_Index(l->tempIDArr[lane], tempArrSize, currNextLinkID);
                    
                    l->tempNumArr[lane][index]--;
                    effNumCF++;
                }
            }
		}

		l->numCF[NUM_SECTION][lane] = effNumCF;
        
        //printf("%f ", l->numCF[NUM_SECTION][lane]);
	}
    //printf("\n");

	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
       	for (int i = 0 ; i < l->numCF[NUM_SECTION][lane] ; i++) {
			l->vehCF[NUM_SECTION][lane][i] = 1;
      	}
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_nextLink()
/// @brief   Function update variables of first cell of a link using
///          variables of connection cell. 
/// @param   vehicle* v, link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane
/// @return  None
/*--------------------------------------------------------------------*/
void Update_nextLink(vehicle* v, link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLane) {
	/// (1) previous link -> connection cell
	for (int i = 0 ; i < MAX_VEC ; i++) {
		cc->currLinkOrderArr[currLane][i] = prevl->currLinkOrderArr[NUM_SECTION+1][currLane][i];
		cc->nextLinkIDArr[currLane][i] = prevl->nextLinkIDArr[NUM_SECTION+1][currLane][i];
		cc->vehIDArr[currLane][i] = prevl->vehIDArr[NUM_SECTION+1][currLane][i];
	}
		
	/// (2) connection cell -> first cell of next link
	int index = nextl->numVehArr[1][currLane];

	for (int i = 0 ; i < MAX_VEC ; i++) {
		int currOrder = cc->currLinkOrderArr[currLane][i];
		int currVehID = cc->vehIDArr[currLane][i];
        
        //printf("index: %d, currOrder: %d, currVehID: %d,  \n", index, currOrder, currVehID);
		if (cc->nextLinkIDArr[currLane][i] == nextl->linkID) {
			nextl->vehIDArr[1][nextLane][index] = cc->vehIDArr[currLane][i];
			nextl->currLinkOrderArr[1][nextLane][index] = cc->currLinkOrderArr[currLane][i] + 1;
			nextl->nextLinkIDArr[1][nextLane][index] = v[currVehID].path[currOrder+2];
			nextl->minTargetLaneArr[1][nextLane][index] = v[currVehID].minTargetLane[currOrder+1];
			nextl->maxTargetLaneArr[1][nextLane][index] = v[currVehID].maxTargetLane[currOrder+1];

            //printf("nextLinkIDArr: %d, vehIDArr: %d, currLinkOrderArr: %d, nextLinkIDArr: %d \n", cc->nextLinkIDArr[currLane][i], nextl->vehIDArr[1][nextLane][index], nextl->currLinkOrderArr[1][nextLane][index], nextl->nextLinkIDArr[1][nextLane][index]);
			
            index++;

		}
	}
	nextl->numVehArr[1][nextLane] = index;
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_ConnectionCell()
/// @brief   Function that resets variables of connection cell.
/// @param   connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Reset_ConnectionCell(connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
	    for (int i = 0 ; i < 3 ; i++) {
	    	cc->numVehArr[lane][i] = 0;
	    	cc->numCFArr[lane][i] = 0;
	    }
	    
	    for (int i = 0 ; i < MAX_VEC ; i++) {
            cc->currLinkOrderArr[lane][i] = -1;
	    	cc->nextLinkIDArr[lane][i] = -1;
			cc->vehIDArr[lane][i] = -1;
	    }
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_Link()
/// @brief   Function that resets flags and temp variables of link.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void Reset_Link(link* l) {

	for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            
            l->numVehArr[0][lane] = 0;
            l->numVehArr[NUM_SECTION+1][lane] = 0;
            
        }
    }
    
    update_Speed(l);
    
    for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
	    	l->numMLCL[sect][lane] = 0;
			l->numMLCR[sect][lane] = 0;
			l->numCF[sect][lane] = 0;
			
	    	for (int i = 0 ; i < MAX_VEC ; i++) {
	    		l->vehMLC[sect][lane][i] = 0;
				l->vehOLC[sect][lane][i] = 0;
				l->vehCF[sect][lane][i] = 0;

				l->vehIDArr[NUM_SECTION+1][lane][i] = -1;
				l->currLinkOrderArr[NUM_SECTION+1][lane][i] = -1;
				l->nextLinkIDArr[NUM_SECTION+1][lane][i] = -1;
				l->minTargetLaneArr[NUM_SECTION+1][lane][i] = -1;
				l->maxTargetLaneArr[NUM_SECTION+1][lane][i] = -1;
	    	}
      	}
    }

    for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    	for (int i = 0 ; i < 3 ; i++) {
    		l->tempIDArr[lane][i] = -1;
    		l->tempNumArr[lane][i] = 0;
    	}
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void update_Speed()
/// @brief   Function that calculate density and speed of link.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void update_Speed(link* l) {
    double wSpeed = WAVESPEED * 1000 / 3600;
    
    for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            
            double jamdensity = (double) JAMDENSITY / 1000;  //도로가 막히기 시작하는 density (veh/km), 링크 특성(고정값)
            l->density[sect][lane] = (double) l->numVehArr[sect][lane] / l->lenSection[sect];  //도로의 현재 density (veh/m), 시뮬레이션 스텝마다 업데이트
            
            //if (l->density[sect][lane] > 0 ) {
            l->speed[sect][lane] = MIN( l->ffSpeed, wSpeed/l->density[sect][lane] * (jamdensity - l->density[sect][lane])) ;
            //}
        }
    }
}

/*--------------------------------------------------------------------*/
/// @fn      void Reset_Sink()
/// @brief   Function that resets flags and temp variables of sink.
/// @param   sink_cell* sk
/// @return  None
/*--------------------------------------------------------------------*/
void Reset_Sink(sink_cell* sk) {
	sk->numVeh = 0;

	for (int i = 0 ; i < MAX_SINK_VEC ; i++) {
		sk->vehIDArr[i] = -1;
		sk->arrivalTime[i] = 0;
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_Source()
/// @brief   Function that update source in every UPDATE_INTERVAL.
/// @param   vehicle* v, int numVeh, source_cell* sc, int currLoop
/// @return  None
/*--------------------------------------------------------------------*/
void Update_Source(vehicle* v, int numVeh, source_cell* sc, int currLoop) {
	

	for (int i = 0 ; i < numVeh ; i++) {
		if ( (currLoop * SECONDS_PER_STEP) <= v[i].departTime && v[i].departTime < ((currLoop + UPDATE_INTERVAL) * SECONDS_PER_STEP)) {
			sc[v[i].initLink].vehIDArr[sc[v[i].initLink].numVeh] = v[i].vehID;
			sc[v[i].initLink].currLinkOrderArr[sc[v[i].initLink].numVeh] = 0;
			sc[v[i].initLink].nextLinkIDArr[sc[v[i].initLink].numVeh] = v[i].path[1];
			sc[v[i].initLink].minTargetLaneArr[sc[v[i].initLink].numVeh] = v[i].minTargetLane[0];
			sc[v[i].initLink].maxTargetLaneArr[sc[v[i].initLink].numVeh] = v[i].maxTargetLane[0];
			
			sc[v[i].initLink].numVeh++;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Start_Path()
/// @brief   Function that moves vehicles from source cell to initial
///          link.
/// @param   link* l, source_cell* sc, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
void Start_Path(link* l, source_cell* sc) {
    int startSection = 1;
    int startLane = 3;
	if (l->numVehArr[startSection][startLane] < MAX_VEC) {
		int numEnter = MIN( MIN(l->maxNumCF[startSection-1][startLane], (MAX_VEC - l->numVehArr[startSection][startLane])), sc->numVeh);
		//numEnter = MIN(numEnter, sc->numVeh);
        
        //printf("numEnter: %d \n", numEnter);
        
		for (int j = 0 ; j < numEnter ; j++) {
			//printf("--------MOVED------- sourceID: %d j: %d \n", sc->sourceID, j);
            //printf("vehIDArr %d \n", j);
			MoveCF(sc->vehIDArr, sc->numVeh, l->vehIDArr[startSection][startLane], l->numVehArr[startSection][startLane], 0);
			//printf("currLinkOrderArr %d \n", j);
            MoveCF(sc->currLinkOrderArr, sc->numVeh, l->currLinkOrderArr[startSection][startLane], l->numVehArr[startSection][startLane], 0);
			//printf("nextLinkIDArr %d \n", j);
            MoveCF(sc->nextLinkIDArr, sc->numVeh, l->nextLinkIDArr[startSection][startLane], l->numVehArr[startSection][startLane], 0);
			//printf("minTargetLaneArr %d \n", j);
            MoveCF(sc->minTargetLaneArr, sc->numVeh, l->minTargetLaneArr[startSection][startLane], l->numVehArr[startSection][startLane], 0);
			//printf("maxTargetLaneArr %d \n", j);
            MoveCF(sc->maxTargetLaneArr, sc->numVeh, l->maxTargetLaneArr[startSection][startLane], l->numVehArr[startSection][startLane], 0);
			
			sc->numVeh--;
			l->numVehArr[startSection][3]++;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Remove_Value()
/// @brief   Function that removes elements of an array.
/// @param   int* fromArr, int fromArrSize, int index
/// @return  None
/*--------------------------------------------------------------------*/
void Remove_Value(int* fromArr, int fromArrSize, int index) {
   
    for (int i = 0 ; i < MAX_VEC-1 ; i++) {
        fromArr[i] = fromArr[i+1];
    }
    fromArr[MAX_VEC] = -1;
    
    for (int i = 0 ; i < MAX_VEC ; i++) {
        printf("fromArr[%d]: %d \n", i, fromArr[i]);
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void End_Path()
/// @brief   Function that moves vehicles from the last link of their 
///          path to sink cell.
/// @param   link* l, sink_cell* sk, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
void End_Path(link* l, sink_cell* sk, int numLoop) {
    int endSection = 3;
    int numVehSink = 0;
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        int vehNum = l->numVehArr[endSection][lane];
    	for (int i = 0 ; i < vehNum ; i++) {
    		if (l->nextLinkIDArr[endSection][lane][i] == -2) {
    			//MoveCF(l->vehIDArr[endSection][lane], l->numVehArr[endSection][lane], sk->vehIDArr, sk->numVeh, 0);
                sk->vehIDArr[sk->numVeh] = l->vehIDArr[endSection][lane][i];
                l->vehIDArr[endSection][lane][i] = -1;
                sk->arrivalTime[sk->numVeh] = numLoop;
                //printf("currLinkOrderArr \n");
                l->currLinkOrderArr[endSection][lane][i] = -1;
                //printf("nextLinkIDArr \n");
                l->nextLinkIDArr[endSection][lane][i] = -1;
                //printf("minTargetLaneArr \n");
                l->minTargetLaneArr[endSection][lane][i] = -1;
                //printf("maxTargetLaneArr \n");
                l->maxTargetLaneArr[endSection][lane][i] = -1;
                
                /*
	        	Remove_Value(l->currLinkOrderArr[endSection][lane], l->numVehArr[endSection][lane], i);
                Remove_Value(l->nextLinkIDArr[endSection][lane], l->numVehArr[endSection][lane], i);
                Remove_Value(l->minTargetLaneArr[endSection][lane], l->numVehArr[endSection][lane], i);
                Remove_Value(l->maxTargetLaneArr[endSection][lane], l->numVehArr[endSection][lane], i);
	        	*/
                //numVehSink++;
	        	sk->numVeh++;
	        	l->numVehArr[endSection][lane]--;
    		}
    	}
    }
    
    for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        //printf("lane: %d \n", lane);
        
        //printf("vehIDArr \n");
        Remove_Blank(l->vehIDArr[endSection][lane], MAX_VEC-1);
        //printf("currLinkOrderArr \n");
        Remove_Blank(l->currLinkOrderArr[endSection][lane], MAX_VEC-1);
        //printf("nextLinkIDArr \n");
        Remove_Blank(l->nextLinkIDArr[endSection][lane], MAX_VEC-1);
        //printf("minTargetLaneArr \n");
        Remove_Blank(l->minTargetLaneArr[endSection][lane], MAX_VEC-1);
        //printf("maxTargetLaneArr \n");
        Remove_Blank(l->maxTargetLaneArr[endSection][lane], MAX_VEC-1);
        
    }
}



/*--------------------------------------------------------------------*/
/// @fn      void SimulationStep()
/// @brief   Function that manages the whole process of simulation.
/// @param   link l[], int numLink, connection_cell cc[], int numCC, 
///          vehicle* v, int numVeh, int numLoop
/// @return  None
/*--------------------------------------------------------------------*/
void SimulationStep(vehicle* v, int numVeh, link l[], source_cell sc[], sink_cell sk[], connection_cell cc[], int numLink, int numLoop) {
	for (int count = 0 ; count < numLoop ; count++) {
		//int updateCycle = UPDATE_INTERVAL;
		int updateStep = count % UPDATE_INTERVAL;
        
        printf("UPDATASOURCE========================================\n");
        
		if (updateStep == 0) {
			for (int link = 0 ; link < numLink ; link++) {
				//PrintOutput();
				Reset_Sink(&sk[link]);
			}
		
			Update_Source(v, numVeh, sc, count);
		}
        
        //PrintOutput(mylink, mysource, mysink, numLink);
        
        printf("ENDPATH STARTPATH========================================\n");
        for (int link = 0 ; link < numLink ; link++) {
            //printf("link: %d \n", link);
        	End_Path(&l[link], &sk[link], numLoop);
            Start_Path(&l[link], &sc[link]);
        }
        
        for (int link = 0 ; link < numLink ; link++) {
            update_Speed(&l[link]);
        }
        //PrintOutput(mylink, mysource, mysink, numLink);

        printf("LCSIM========================================\n");
        
        for (int link = 0 ; link < numLink ; link++) {
            printf("link: %d \n", link);
            Evaluate_MLC(&l[link]);
            Evaluate_OLC(&l[link]);
            LCSim(&l[link]);
            
            Update_tempArr(&l[link]);
            
        }
        
        //PrintOutput(mylink, mysource, mysink, numLink);

        printf("CFSIM========================================\n");
        printf("RELAYNUMVEH========================================\n");
        
        for (int link = 0 ; link < numLink ; link++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		for (int leg = 0 ; leg < MAX_LEG ; leg++) {
        			if (cc[link].nextLinkID[lane][leg] >= 0) {
                        //printf("lane: %d, leg: %d, nextlink: %d \n", lane, leg, cc[link].nextLinkID[lane][leg]);
        				Relay_numVeh(&l[cc[link].prevLinkID], &l[cc[link].nextLinkID[lane][leg]], cc[link].nextLane[lane][leg], &cc[link], lane, leg, link);
                        
        			}
        		}
        	}
            
        	Reset_ConnectionCell(&cc[link]);
        }
        
        //PrintOutput(mylink, mysource, mysink, numLink);
        printf("Evaluate_CF========================================\n");
        
        for (int link = 0 ; link < numLink ; link++) {
        	Evaluate_CF(&l[link]);
        }
        //PrintOutput(mylink, mysource, mysink, numLink);
        printf("Relay_numCF========================================\n");
	
        for (int i = 0 ; i < numLink ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		for (int leg = 0 ; leg < MAX_LEG ; leg++) {
                   
        			if (cc[i].nextLinkID[lane][leg] >= 0) {
        				Relay_numCF(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][leg]], cc[i].nextLane[lane][leg], &cc[i], count, lane, leg); 		
        			}
                    
        		}
        	}
        	Reset_ConnectionCell(&cc[i]);
       	}
        
        //PrintOutput(mylink, mysource, mysink, numLink);
        printf("SIM========================================\n");
        
        for (int link = 0 ; link < numLink ; link++) {
        	Evaluate_Eff_numCF(&l[link]);
            printf("link: %d \n", link);
            CFsim(&l[link]);
        	
        }
        
		//PrintOutput(mylink, mysource, mysink, numLink);
        printf("UPDATENEXTLINK========================================\n");
        
        for (int i = 0 ; i < numLink ; i++) {
            //printf("link: %d \n", i);
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		for (int leg = 0 ; leg < MAX_LEG ; leg++) {
        			if (cc[i].nextLinkID[lane][leg] >= 0) {
                        //printf("lane: %d, leg: %d, nextLink: %d, nextLane: %d \n", lane, leg, cc[i].nextLinkID[lane][leg], cc[i].nextLane[lane][leg]);
        				Update_nextLink(v, &l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][leg]], cc[i].nextLane[lane][leg], &cc[i], lane);
        			}
        		}
        	}
        	Reset_ConnectionCell(&cc[i]);
        }
        
        //PrintOutput(mylink, mysource, mysink, numLink);
        
        for (int link = 0 ; link < numLink ; link++) {
            Reset_Link(&l[link]);
        }
        
        printf("RESETLINK========================================\n");
        PrintOutput(mylink, mysource, mysink, numLink);
        
	}
}



double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);

    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}

void PrintCC (connection_cell cc[], int numLink) {
    printf("=CC================================\n");
    
    for (int i = 0 ; i < numLink ; i++) {
        printf("cc: %d \n", i);
        
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            for (int leg = 0 ; leg < MAX_LEG ; leg++) {
                printf("cc[%d].nextLinkID[%d][%d] = [%d], cc[%d].nextLane[%d][%d] = [%d] \n", i, lane, leg, cc[i].nextLinkID[lane][leg], i, lane, leg, cc[i].nextLane[lane][leg]);
            }
        }
    }
}

void PrintOutput (link l[], source_cell sc[], sink_cell sk[], int numLink) {
	printf("========================================\n");
	
	for (int link = 0 ; link < numLink ; link++) {
		printf("link: %d, \n", link);

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
                printf("section: %d, lane: %d, numVeh: %d, maxNumVeh: %d, density: %f, speed: %f   ",
    				sect, lane, l[link].numVehArr[sect][lane], l[link].maxNumVeh[sect][lane], l[link].density[sect][lane], l[link].speed[sect][lane]);
    			for (int i = 0 ; i < MAX_VEC ; i++) {
    				if (l[link].vehIDArr[sect][lane][i] >= 0) {
    					printf("vehIDArr: %d ", l[link].vehIDArr[sect][lane][i]);
                        printf("(%d) ", l[link].nextLinkIDArr[sect][lane][i]);
    				}
    			}
    			printf("\n");
    		}
    	}

    	printf("source cell: %d, \n", link);
    	printf("numVeh: %d ", sc[link].numVeh);
    	for (int i = 0 ; i < MAX_SOURCE_VEC ; i++) {
    		if (sc[link].vehIDArr[i] >= 0) {
    			printf("vehIDArr: %d ", sc[link].vehIDArr[i]);
    		}
    	}
    	printf("\n");
        
        printf("sink cell: %d, \n", link);
        printf("numVeh: %d ", sk[link].numVeh);
        for (int i = 0 ; i < MAX_SINK_VEC ; i++) {
            if (sk[link].vehIDArr[i] >= 0) {
                printf("vehIDArr: %d ", sk[link].vehIDArr[i]);
            }
        }
        printf("\n");
        
	}
}


int main(int argc, char *argv[]) {
	srand(time(NULL));

	int numVeh = (int) atoi(argv[1]);  /// number of vehicles
    int numLink = (int) atoi(argv[2]); /// number of links
    int numLoop = (int) atoi(argv[3]);   /// number of simulation steps

	printf("========================================\n");
    printf("# of vehicles: %s\n", argv[1]);
    printf("# of links: %s\n", argv[2]);
    printf("# of simulation steps: %s\n", argv[3]);
    printf("========================================\n");
	
	myveh = (vehicle*) calloc(numVeh, sizeof(vehicle));
	mylink = (link*) calloc(numLink, sizeof(link));
	mysource = (source_cell*) malloc(numLink * sizeof(source_cell));
	mysink = (sink_cell*) malloc(numLink * sizeof(sink_cell));
	mycon = (connection_cell*) malloc(numLink * sizeof(connection_cell));

    Setup_Veh(myveh, numVeh);
    Setup_Link(myveh, numVeh, mylink, mysource, mysink, numLink);
    Setup_ConnectionCell(mycon, numLink);
    
    double start, stop;
    start = get_time_ms();
    SimulationStep(myveh, numVeh, mylink, mysource, mysink, mycon, numLink, numLoop);
    stop = get_time_ms();
    
    printf("========================================\n");
    double result = stop - start;
    printf("Elapsed Time: %f\n\n", result);
    printf("========================================\n");

    return 0;
}
