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
connection_cell *mycon;


/// RECENT UPDATES
/// 1. update header file
/// 2. update Setup_Connectioncell
/// 3. add Update_Source, Update_Link

/// TODO 
/// 4. update Relay_numVeh, Relay_numCF, Update_nextLink

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
		
		v[i].initLink = 0;
		v[i].departTime = i;
	}

	/// vPath[] = list of linkID
	int vPath[100][4] = {
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
	int vMinTL[100][4] = {
	{0, 0, 0, -1},  {0, 0, 0, -1},  {0, 0, 0, -1},  {0, 1, 0, -1},  {0, 1, 0, -1},
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
	};
		
	/// vMaxTL[] = list of maxTargetLane
	int vMaxTL[100][4] = {
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
	};

	/// (3) Set vehicle minimum and maximum target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		for (int j = 0 ; j < v[i].lenPath ; j++) {
			v[i].minTargetLane[j] = vMinTL[i][j];
			v[i].maxTargetLane[j] = vMaxTL[i][j];
		}
	}

	/// vPos[] = initLink, departTime
	int vPos[100][2] = {
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

		l[i].ffSpeed = 16;

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
			l[i].lenSection[sect] = 100;

			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].maxNumVeh[sect][lane] = 20;
				l[i].maxNumCF[sect][lane] = 3;

				l[i].numVehArr[sect][lane] = 0;

				for (int j = 0 ; j < MAX_VEC ; j++) {
			    	l[i].vehIDArr[sect][lane][j] = -1;
	    			l[i].currLinkOrderArr[sect][lane][j] = -1;
	    			l[i].nextLinkIDArr[sect][lane][j] = -1;
	    			l[i].minTargetLaneArr[sect][lane][j] = -1;
	    			l[i].maxTargetLaneArr[sect][lane][j] = -1;
				}

				l[i].speed[sect][lane] = 0;

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
		}

		sk[i].sinkID = i;

		sk[i].numVeh = 0;
		for (int j = 0 ; j < MAX_SINK_VEC ; j++) {
			sk[i].vehIDArr[j] = -1;
			sk[i].arrivalTime[j] = 0;
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
				cc[i].currLinkOrderArr[lane][j] = 0;
				cc[i].nextLinkIDArr[lane][j] = 0;
				cc[i].vehIDArr[lane][j] = 0;
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

	for (int count = 0 ; count < MAX_LOOP ; count++) {
		int trafficCycle = 16;
		int trafficStep = count % trafficCycle;

		if (trafficStep < 5) {
			cc[1].trafficSignal[1][count] = 1;
			cc[1].trafficSignal[2][count] = 1;
			cc[1].trafficSignal[3][count] = 1;
			cc[5].trafficSignal[1][count] = 1;
			cc[5].trafficSignal[2][count] = 1;
			cc[5].trafficSignal[3][count] = 1;
		}
		else if (trafficStep < 8) {
			cc[1].trafficSignal[0][count] = 1;
			cc[5].trafficSignal[0][count] = 1;
		}
		else if (trafficStep < 13) {
			cc[3].trafficSignal[1][count] = 1;
			cc[3].trafficSignal[2][count] = 1;
			cc[3].trafficSignal[3][count] = 1;
			cc[7].trafficSignal[1][count] = 1;
			cc[7].trafficSignal[2][count] = 1;
			cc[7].trafficSignal[3][count] = 1;
		}
		else if (trafficStep < 16) {
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

		if (lane == minTL) {
			if (possOLC[i] == 2) possOLC[i] = 1;
			else if (possOLC[i] == -1) possOLC[i] = 0;
		}
		else if (lane == maxTL) {
			if (possOLC[i] == 2) possOLC[i] = -1;
			else if (possOLC[i] == 1) possOLC[i] = 0;
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
		numOLC_L--;
		if (possTotal >= numOLC_L + numOLC_R) break;
		numOLC_R--;
	}

	/// (5) Update values of vehOLC flags.
	int count_R = numOLC_R;
	int count_L = numOLC_L;
	if (numOLC_L == 0 && numOLC_R == 0);

	else if (numOLC_L == 0) {
		while (count_R) {
			int randVeh = rand()%possRight;
			if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}
	}

	else if (numOLC_R == 0) {
		while (count_L) {
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
            int diffSpeed_L = l->speed[sect][lane-1] - l->speed[sect][lane];
            int diffSpeed_R = l->speed[sect][lane+1] - l->speed[sect][lane];

            /// (1) Set probLC to zero in special cases. 
            /// When current lane is leftmost, probLC_L = 0
            /// In other words, the vehicle cannot move left.
            if (lane == 0) probLC_L = 0;
            else probLC_L = (diffSpeed_L / l->ffSpeed) * l->numVehArr[sect][lane];
            
            /// When current lane is rightmost, probLC_R = 0
            /// In other words, the vehicle cannot move right.
            if (lane == NUM_LANE-1) probLC_R = 0;
            else probLC_R = (diffSpeed_R / l->ffSpeed) * l->numVehArr[sect][lane];

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
/// @brief   Function that moves elements of one array to another array.
/// @param   int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index
/// @return  None
/*--------------------------------------------------------------------*/
void MoveLC(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
	double fromArrLoc = ((double)fromArrSize / (index+1));
	int toArrLoc = Evaluate_Prob(toArrSize / fromArrLoc);

	for (int i = MAX_VEC-1 ; i > toArrLoc ; i--) {
		toArr[i] = toArr[i-1];
	}
	toArr[toArrLoc] = fromArr[index];

	for (int i = index ; i < MAX_VEC ; i++) {
		fromArr[i] = fromArr[i+1];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void LCSim()
/// @brief   Function to move vehicles that perform lane change.
/// @param   link* l
/// @return  None
/*--------------------------------------------------------------------*/
void LCSim(link* l) {
    for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehMLC[sect][lane][i] == 1) {
	        		if (l->numVehArr[sect][lane+1] < MAX_VEC) {
	        			MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        				l->vehIDArr[sect][lane+1], l->numVehArr[sect][lane+1], i+1);
		        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->currLinkOrderArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
		        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->minTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
		        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->maxTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
		        		l->numVehArr[sect][lane+1]++;
		        		l->numVehArr[sect][lane]--;
	        		}
	        	} 
	        		
	        	else if (l->vehMLC[sect][lane][i] == -1) {
	        		if (l->numVehArr[sect][lane-1] < MAX_VEC) {
						MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        				l->vehIDArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
		        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->currLinkOrderArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
		        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->minTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
		        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
		        			l->maxTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
		        		l->numVehArr[sect][lane-1]++;
		        		l->numVehArr[sect][lane]--;
	        		}
	        	}
	        }
        }
    }  

    for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehOLC[sect][lane][i] == 1 && l->numVehArr[sect][lane] < MAX_VEC) {
	        		MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
	        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
	        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
	        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->maxTargetLaneArr[sect][lane+1], l->numVehArr[sect][lane+1], i);
	        		l->numVehArr[sect][lane+1]++;
	        		l->numVehArr[sect][lane]--;
	        	}
	        	else if (l->vehOLC[sect][lane][i] == -1 && l->numVehArr[sect][lane] < MAX_VEC) {
	        		MoveLC(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
	        		MoveLC(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
	        		MoveLC(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
	        		MoveLC(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->maxTargetLaneArr[sect][lane-1], l->numVehArr[sect][lane-1], i);
	        		l->numVehArr[sect][lane-1]++;
	        		l->numVehArr[sect][lane]--;
	        	}
	        }
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
				//MIN(MIN((l->ffSpeed / 3.6 * SECONDS_PER_STEP) / l->lenSection[sect] * l->numVeh[sect][lane], l->maxNumCF[sect][lane]), 
				//	MIN(l->maxNumCF[sect+1][lane], wSpeed * SECONDS_PER_STEP / length * (l->maxNumCF[sect][lane] - l->numVeh[sect][lane])));   
				MIN(l->numVehArr[sect][lane], MIN(l->maxNumCF[sect][lane], wSpeed / l->ffSpeed * (l->maxNumVeh[sect+1][lane] - l->numVehArr[sect+1][lane]))) * l->ffSpeed * SECONDS_PER_STEP / l->lenSection[sect];

	      	for (int i = 0 ; i < l->numCF[sect][lane] ; i++) {
				l->vehCF[sect][lane][i] = 1;
      		}
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
	toArr[toArrSize] = fromArr[index];

	for (int i = MAX_VEC-1 ; i >= 0 ; i--) {
		fromArr[i] = fromArr[i-1];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void CFsim()
/// @brief   Function to move vehicles that perform CF.
/// @param   link *l
/// @return  None
/*--------------------------------------------------------------------*/
void CFsim(link *l) {
	for (int sect = NUM_SECTION ; sect > 0 ; sect--) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehCF[sect][lane][i] == 1 && l->numVehArr[sect+1][lane] < MAX_VEC) {
	        		MoveCF(l->vehIDArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->vehIDArr[sect+1][lane], l->numVehArr[sect+1][lane], i);
	        		MoveCF(l->currLinkOrderArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->currLinkOrderArr[sect+1][lane], l->numVehArr[sect+1][lane], i);
	        		MoveCF(l->minTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->minTargetLaneArr[sect+1][lane], l->numVehArr[sect+1][lane], i);
	        		MoveCF(l->maxTargetLaneArr[sect][lane], l->numVehArr[sect][lane], 
	        			l->maxTargetLaneArr[sect+1][lane], l->numVehArr[sect+1][lane], i);
	        		l->numVehArr[sect+1][lane]++;
	        		l->numVehArr[sect][lane]--;
	        	}
	        }
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

        for (int i = 0 ; i < l->numVehArr[NUM_SECTION][lane] ; i++) {
        	int tempArrIndex = Find_Index(l->tempIDArr[lane], 3, l->nextLinkIDArr[NUM_SECTION][lane][i]);

        	if (tempArrIndex == -1) {
        		l->tempIDArr[lane][tempArrSize] = l->nextLinkIDArr[NUM_SECTION][lane][i];
        		l->tempNumArr[lane][tempArrSize] += 1;
        		tempArrSize += 1;
        	}

        	else l->tempNumArr[lane][tempArrIndex] += 1;
        }
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
void Relay_numVeh(link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLane, int i) {
	if (cc->nextLinkID[currLane][i] != -1) {
		int index = Find_Index(prevl->tempIDArr[currLane], 3, cc->nextLinkID[currLane][i]);

		if (index != -1) {
			/// (1) tempArr of previous link -> connection cell
			cc->numVehArr[currLane][i] = prevl->tempNumArr[currLane][index];

			/// (2) connection cell -> virtual cell of next link
			nextl->numVehArr[0][nextLane] = cc->numVehArr[currLane][i];
		}
	}	
}


/*--------------------------------------------------------------------*/
/// @fn      void Relay_numCF()
/// @brief   Function that relay numCF of the virtual cell of a next 
///          link to the last cell of the previous link.
/// @param   link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int i
/// @return  None
/*--------------------------------------------------------------------*/
void Relay_numCF(link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int i) {
	if (cc->nextLinkID[currLane][i] != -1) {
		/// (1) virtual cell of next link -> connection cell
		if (cc->trafficSignal[currLane][currLoop] == 0) {
			cc->numCFArr[currLane][currLoop] = 0;
		}
		else cc->numCFArr[currLane][i] = nextl->numCF[0][nextLane];

		/// (2) connection cell -> previous link
		int index = Find_Index(prevl->tempIDArr[currLane], 3, cc->nextLinkID[currLane][i]);

		prevl->tempNumArr[currLane][index] = cc->numVehArr[currLane][i];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Evaluate_Eff_numCF()
/// @brief   Function that evaluates effective numCF.
/// @param   link* l, connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Evaluate_Eff_numCF(link* l) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		int tempArrSize = 0;
		int totalNumCF = 0;

		for (int i = 0 ; i < 3 ; i++) {
			totalNumCF += l->tempNumArr[lane][i];

			if (l->tempIDArr[lane][i] == -1) {
				tempArrSize = i;
				break;
			}
		}

		int effNumCF = 0;
		for (int count = 0 ; count < MIN(totalNumCF, l->maxNumCF[NUM_SECTION][lane]) ; count++){
			int currNextLinkID = l->nextLinkIDArr[NUM_SECTION][lane][count];
			int index = Find_Index(l->tempIDArr[lane], tempArrSize, currNextLinkID);
			
			if (l->tempNumArr[lane][index] == 0) break;
			l->tempNumArr[lane][index]--;
			
			effNumCF++;
		}

		l->numCF[NUM_SECTION][lane] = effNumCF;
	}

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
	int count = 0;
	int currNumVeh = nextl->numVehArr[1][currLane];
	for (int i = 0 ; i < MAX_VEC ; i++) {
		int currOrder = cc->currLinkOrderArr[currLane][i];
		int currVehID = cc->vehIDArr[currLane][i];

		if (cc->nextLinkIDArr[currLane][i] == nextl->linkID) {
			int index = currNumVeh + count;
			nextl->vehIDArr[1][currLane][index] = cc->vehIDArr[currLane][i];
			nextl->currLinkOrderArr[1][currLane][index] = cc->currLinkOrderArr[currLane][i] + 1;
			nextl->nextLinkIDArr[1][currLane][index] = v[currVehID].path[currOrder+1];
			nextl->minTargetLaneArr[1][currLane][index] = v[currVehID].minTargetLane[currOrder+1];
			nextl->maxTargetLaneArr[1][currLane][index] = v[currVehID].maxTargetLane[currOrder+1];

			count++;
		}
	}

	nextl->numVehArr[1][currLane] += count;
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
	    	cc->nextLinkIDArr[lane][i] = 0;
			cc->vehIDArr[lane][i] = 0;
			cc->currLinkOrderArr[lane][i] = 0;	
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
	double wSpeed = 4.2;

	for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    		int jamdensity = l->maxNumVeh[sect][lane] / l->lenSection[sect];  //도로가 막히기 시작하는 density (veh/km), 링크 특성(고정값)
			int density = l->numVehArr[sect][lane] / l->lenSection[sect];  //도로의 현재 density (veh/km), 시뮬레이션 스텝마다 업데이트

	    	l->speed[sect][lane] = MAX(0, MIN(l->ffSpeed, (-wSpeed + (wSpeed * jamdensity / density))));
	    	l->numMLCL[sect][lane] = 0;
			l->numMLCR[sect][lane] = 0;
			l->numCF[sect][lane] = 0;

			l->numVehArr[0][lane] = 0;
			l->numVehArr[NUM_SECTION+1][lane] = 0;
			
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
    	for (int i ; i < 3 ; i++) { 
    		l->tempIDArr[lane][i] = -1;
    		l->tempNumArr[lane][i] = 0;
    	}
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_sink()
/// @brief   Function that resets flags and temp variables of sink.
/// @param   sink_cell* sk
/// @return  None
/*--------------------------------------------------------------------*/
void Reset_sink(sink_cell* sk) {
	sk->numVeh = 0;

	for (int i = 0 ; i < MAX_SINK_VEC ; i++) {
		sk->vehIDArr[i] = -1;
		sk->arrivalTime[i] = 0;
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_Source()
/// @brief   Function that update source in every UPDATE_INTERVAL.
/// @param   vehicle* v, source_cell* sc
/// @return  None
/*--------------------------------------------------------------------*/
void Update_Source(vehicle* v, int numVeh, source_cell* sc, int currLoop) {
	// once in 60 step
	// vehicle scan
	int j = 0;
	for (int i = 0 ; i < numVeh ; i++) {
		if (currLoop =< v[i].departTime && v[i].departTime < (currLoop + UPDATE_INTERVAL)) {
			sc[v[i].initLink].vehIDArr[j] = v[i].vehID;
			j++;
			sc[v[i].initLink].numVeh++;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_Link()
/// @brief   Function that update source every step.
/// @param   link*, source_cell*
/// @return  None
/*--------------------------------------------------------------------*/
void Enter_Link(link* l, source_cell* sc, int numLink) {
	if (l->numVehArr[3][3] < MAX_VEC) {
		int numEnter = MIN(l[i].maxNumCF[2][3], (MAX_VEC - l[i].numVehArr[3][3]));

		for (int j = 0 ; j < numEnter ; j++) {
			MoveCF(sc->vehIDArr[j], sc->numVehArr, l->vehIDArr[3][3], l->numVehArr[3][3], 0);
			
			sc->numVeh--;
			l->numVeh[3][3]++;
		}
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void SimulationStep()
/// @brief   Function that manages the whole process of simulation.
/// @param   link l[], int numLink, connection_cell cc[], int numCC, 
///          vehicle* v, int numVeh, int numLoop
/// @return  None
/*--------------------------------------------------------------------*/
void SimulationStep(vehicle* v, int numVeh, link l[], int numLink, source_cell sc[], sink_cell sk[], connection_cell cc[], int numCC, int numLoop) {
	for (int count = 0 ; count < numLoop ; count++) {
		int updateCycle = UPDATE_INTERVAL;
		int updateStep = count % UPDATE_INTERVAL;

		if (updateStep == 0) {
			for (int link = 0 ; link < numLink ; link++) {
				//PrintOutput();
				Reset_Sink(&sk[link]);
				Update_source(v, numVeh, &sc[link], currLoop);
			}
		}

        for (int link = 0 ; link < numLink ; link++) {
        	Reset_Link(&l[link]);

        	Enter_Link(&l[link], &sc[link], numLink);

            Evaluate_MLC(&l[link]);
            Evaluate_OLC(&l[link]);
            LCSim(&l[link]);
            
            Update_tempArr(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		Relay_numVeh(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][0]], cc[i].nextLane[lane][0], &cc[i], lane, 0);
        		Relay_numVeh(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][1]], cc[i].nextLane[lane][1], &cc[i], lane, 1);
        		Relay_numVeh(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][2]], cc[i].nextLane[lane][2], &cc[i], lane, 2);
        	}
        	Reset_ConnectionCell(&cc[i]);
        }

        for (int link = 0 ; link < numLink ; link++) {
        	Evaluate_CF(&l[link]);
        }
	
        for (int i = 0 ; i < numCC ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		Relay_numCF(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][0]], cc[i].nextLane[lane][0], &cc[i], count, lane, 0); 		
        		Relay_numCF(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][1]], cc[i].nextLane[lane][1], &cc[i], count, lane, 1);
				Relay_numCF(&l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][2]], cc[i].nextLane[lane][2], &cc[i], count, lane, 2);
        	}
        	Reset_ConnectionCell(&cc[i]);
       	}

        for (int link = 0 ; link < numLink ; link++) {
        	Evaluate_Eff_numCF(&l[link]);
        	Update_vehCF(&l[link]);
        	CFsim(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		Update_nextLink(v, &l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][0]], cc[i].nextLane[lane][0], &cc[i], lane);
        		Update_nextLink(v, &l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][1]], cc[i].nextLane[lane][1], &cc[i], lane);
				Update_nextLink(v, &l[cc[i].prevLinkID], &l[cc[i].nextLinkID[lane][2]], cc[i].nextLane[lane][2], &cc[i], lane);
        	}
        	Reset_ConnectionCell(&cc[i]);
        }
	}
}


double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);

    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}


void PrintOutput (link l[], int numLink) {
	for (int link = 0 ; link < numLink ; link++) {
		printf("link: %d, \n", link);

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    			printf("section: %d, lane: %d, numVeh: %d\n",
    				sect, lane, l[link].numVehArr[sect][lane]);
    		}
    	}
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
    Setup_ConnectionCell(mycon, numCC);

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
