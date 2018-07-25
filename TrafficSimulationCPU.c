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


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Veh(vehicle*, int)
/// @brief   Function that setup the values of struct vehicle.
/// @param   vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Veh(vehicle* v, int numVeh) {
	/// (1) Set vehicle ID, type.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].vehID = i+1;
		v[i].vehType = 0;
	}

	/// (2) Set vehicle path.
	v[0].path = [11, 1, 2, 3, 16];
	v[1].path = [11, 1, 2, 3, 16];
	v[2].path = [11, 1, 2, 3, 16];
	v[3].path = [11, 1, 2, 3, 16];
	v[4].path = [11, 1, 2, 3, 16];
	v[5].path = [11, 1, 2, 5, 17];
	v[6].path = [11, 1, 2, 5, 17];
	v[7].path = [11, 1, 2, 5, 17];
	v[8].path = [11, 1, 2, 5, 17];
	v[9].path = [11, 1, 2, 5, 17];
	v[10].path = [11, 1, 2, 5, 17];
	v[11].path = [11, 1, 2, 5, 17];
	v[12].path = [11, 1, 2, 7, 18];
	v[13].path = [11, 1, 2, 7, 18];
	v[14].path = [11, 1, 2, 7, 18];
	v[15].path = [12, 4, 5, 17, 0];
	v[16].path = [12, 4, 5, 17, 0];
	v[17].path = [12, 4, 5, 17, 0];
	v[18].path = [12, 4, 9, 10, 11];
	v[19].path = [12, 4, 9, 10, 11];
	v[20].path = [12, 4, 7, 18, 0];
	v[21].path = [12, 4, 7, 18, 0];
	v[22].path = [12, 4, 7, 18, 0];
	v[23].path = [12, 4, 7, 18, 0];
	v[24].path = [12, 4, 7, 18, 0];
	v[25].path = [13, 6, 7, 18, 0];
	v[26].path = [13, 6, 7, 18, 0];
	v[27].path = [13, 6, 7, 18, 0];
	v[28].path = [13, 6, 3, 16, 0];
	v[29].path = [13, 6, 3, 16, 0];
	v[30].path = [13, 6, 9, 10, 15];
	v[31].path = [13, 6, 9, 10, 15];
	v[32].path = [13, 6, 9, 10, 15];
	v[33].path = [13, 6, 9, 10, 15];
	v[34].path = [13, 6, 9, 10, 15];
	v[35].path = [14, 8, 9, 10, 15];
	v[36].path = [14, 8, 9, 10, 15];
	v[37].path = [14, 8, 5, 17, 0];
	v[38].path = [14, 8, 5, 17, 0];
	v[39].path = [14, 8, 5, 17, 0];
	v[40].path = [14, 8, 3, 16, 0];
	v[41].path = [14, 8, 3, 16, 0];
	v[42].path = [14, 8, 3, 16, 0];
	v[43].path = [14, 8, 3, 16, 0];
	v[44].path = [14, 8, 3, 16, 0];

	/// (3) Set vehicle path length.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].lenPath = 5;

		for (int j = 0 ; j < 5 ; j++) { 
			if (v[i].path[j] == 0) {
				v[i].lenPath = j;
				break;
			}
		}
	}
	
	/// vMinTL[] = minTargetLane
	int vMinTL[45][5] = {
	{0, 2, 3, 0, 0}, {0, 2, 3, 0, 0}, {0, 2, 3, 0, 0}, {0, 3, 3, 0, 0}, {0, 3, 3, 0, 0}, 
	{0, 1, 1, 1, 0}, {0, 1, 1, 1, 0}, {0, 1, 1, 1, 0}, {0, 1, 1, 1, 0}, {0, 1, 1, 1, 0},
	{0, 1, 1, 1, 0}, {0, 1, 1, 1, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
	{0, 3, 3, 0, 0}, {0, 3, 3, 0, 0}, {0, 3, 3, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
	{0, 1, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 1, 0, 0},
	{0, 3, 3, 0, 0}, {0, 3, 2, 0, 0}, {0, 3, 2, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 2, 0, 0},
	{0, 3, 2, 1, 0}, {0, 3, 2, 1, 0}, {0, 3, 3, 0, 0}, {0, 3, 2, 0, 0}, {0, 3, 2, 0, 0},
	{0, 1, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 1, 0, 0}, {0, 1, 0, 0, 0}, {0, 1, 0, 0, 0}  };
		
	/// vMaxTL[] = maxTargetLane
	int vMaxTL[20][5] = {
	{3, 3, 3, 0, 3}, {3, 3, 3, 0, 3}, {3, 3, 3, 0, 3}, {3, 3, 3, 0, 3}, {3, 3, 3, 0, 3}, 
	{3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, 
	{3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 2, 0, 0, 3}, {3, 1, 0, 0, 3}, {3, 0, 0, 0, 3},
	{3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 0, 0, 1, 3}, {3, 0, 1, 2, 3}, 
	{3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, 
	{3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 0, 0, 3, 0}, {3, 0, 1, 3, 0}, 
	{3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 3, 3, 3, 3}, {3, 3, 3, 2, 3}, {3, 3, 3, 2, 3}, 
	{3, 3, 3, 3, 3}, {3, 3, 3, 2, 3}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, 
	{3, 3, 2, 3, 0}, {3, 3, 2, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}, {3, 3, 3, 3, 0}  };

	/// (4) Set vehicle minimum and maximum target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		for (int j = 0 ; j < v[i].lenPath ; j++) {
			v[i].minTargetLane[j] = vMinTL[i][j];
			v[i].maxTargetLane[j] = vMaxTL[i][j];
		}
	}

	/// vPos[] = initLink, initLane, initSection
	int vPos[45][3] = {
	{11, 4, 0}, {11, 4, 1}, {11, 4, 2}, {11, 3, 0}, {11, 3, 1}, 
	{11, 4, 0}, {11, 4, 1}, {11, 4, 2}, {11, 3, 3}, {11, 3, 0}, 
	{11, 3, 0}, {11, 3, 0}, {11, 4, 3}, {11, 4, 2}, {11, 4, 1},
	{12, 4, 0}, {12, 4, 1}, {12, 4, 2}, {12, 4, 0}, {12, 4, 1},
	{12, 4, 3}, {12, 4, 2}, {12, 4, 1}, {12, 3, 1}, {12, 3, 0},
	{13, 3, 0}, {13, 3, 1}, {13, 4, 2}, {13, 4, 3}, {13, 4, 0},
	{13, 4, 1}, {13, 4, 2}, {13, 4, 3}, {13, 4, 0}, {13, 4, 1},
	{14, 4, 1}, {14, 4, 2}, {14, 4, 3}, {14, 4, 0}, {14, 4, 1},  
	{14, 4, 3}, {14, 4, 2}, {14, 4, 1}, {14, 3, 0}, {14, 3, 1}  };
	
	/// (5) Set vehicle initial position.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].initLink = vPos[i][0];
		v[i].initLane = vPos[i][1];
		v[i].initSection = vPos[i][2];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Link(link*, int, vehicle*, int)
/// @brief   Function that setup the values of struct link.
/// @param   vehicle* v, int numVeh, link* l, int numLink
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Link(vehicle* v, int numVeh, link* l, int numLink) {
	/// (1) Setup link ID, type.
	for (int i = 0 ; i < numLink ; i++) {
		l[i].linkID = i+1;

		if (i < 10) l[i].linkType = 0;
		else if (i < 14) l[i].linkType = -1;
		else if (i < 18) l[i].linkType = 1;
	}

	/// (2) Setup link characteristics and initialize flags.
	for (int i = 0 ; i < numLink ; i++) {
		l[i].ffSpeed = 16;

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
			l[i].lenSection[sect] = 100;

			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].maxNumVeh[sect][lane] = 20;
				l[i].maxNumCF[sect][lane] = 3;
				l[i].numVehArr[sect][lane] = 0;
				l[i].speed[sect][lane] = 0;
				l[i].numMLCL[sect][lane] = 0;
	    		l[i].numMLCR[sect][lane] = 0;
	    		l[i].numCF[sect][lane] = 0;

	    		for (int j = 0 ; j < numVeh ; j++) {
			    	l[i].vehIDArr[sect][lane][j] = 0;
	    			l[i].currLinkOrderArr[sect][lane][j] = 0;
	    			l[i].nextLinkIDArr[sect][lane][j] = 0;
	    			l[i].minTargetLaneArr[sect][lane][j] = 0;
	    			l[i].maxTargetLaneArr[sect][lane][j] = 0;
	    			l[i].vehMLC[sect][lane][j] = 0;
	    			l[i].vehOLC[sect][lane][j] = 0;
	    			l[i].vehCF[sect][lane][j] = 0;
	    		}
			}
		}

		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			l[i].tempIDArr[lane][0] = 0;
			l[i].tempIDArr[lane][1] = 0;
			l[i].tempIDArr[lane][2] = 0;
			l[i].tempNumArr[lane][0] = 0;
			l[i].tempNumArr[lane][1] = 0;
			l[i].tempNumArr[lane][2] = 0;
		}
	}

	/// (3) Add vehicles to links.
	for (int i = 0 ; i < numLink ; i++) {
    	for (int j = 0 ; j < numVeh ; j++) {
    		if (l[i].linkID == v[j].initLink) {
    			int sect = v[j].initSection;
    			int lane = v[j].initLane;
    			int numVeh = l[i].numVehArr[sect][lane];

    			l[i].vehIDArr[sect][lane][numVeh] = v[j].vehID;
    			l[i].nextLinkIDArr[sect][lane][j] = v[j].path[0+1];
    			l[i].minTargetLaneArr[sect][lane][numVeh] = v[j].minTargetLane[0];
    			l[i].maxTargetLaneArr[sect][lane][numVeh] = v[j].maxTargetLane[0];
    			l[i].numVehArr[sect][lane]++;
    		}
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
		cc[i].ccID = i+1;
		cc[i].prevLinkID = i+1;

		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			cc[i].nextLinkID[lane][0] = 0;
			cc[i].nextLaneID[lane][0] = 0;
			cc[i].nextLinkID[lane][1] = 0;
			cc[i].nextLaneID[lane][1] = 0;
			cc[i].nextLinkID[lane][2] = 0;
			cc[i].nextLaneID[lane][2] = 0;
			
			cc[i].numVehArr[lane] = 0;
			cc[i].numCFArr[lane] = 0;

			for (int j = 0 ; j < MAX_VEC ; j++) {
				cc[i].nextLinkIDArr[lane][j] = 0;
				cc[i].vehIDArr[lane][j] = 0;
				cc[i].currLinkOrderArr[lane][j] = 0;
			}
		}
	}

	cc[1].nextLinkID[0][0] = 2;
	cc[1].nextLaneID[0][0] = 0;
	cc[1].nextLinkID[1][0] = 2;
	cc[1].nextLaneID[1][0] = 1;
	cc[1].nextLinkID[2][0] = 2;
	cc[1].nextLaneID[2][0] = 2;
	cc[1].nextLinkID[3][0] = 2;
	cc[1].nextLaneID[3][0] = 3;
	
	cc[2].nextLinkID[0][0] = 7;
	cc[2].nextLaneID[0][0] = 0;
	cc[2].nextLinkID[1][0] = 5;
	cc[2].nextLaneID[1][0] = 1;
	cc[2].nextLinkID[2][0] = 5;
	cc[2].nextLaneID[2][0] = 2;
	cc[2].nextLinkID[3][0] = 5;
	cc[2].nextLaneID[3][0] = 3;
	cc[2].nextLinkID[3][1] = 3;
	cc[2].nextLaneID[3][1] = 3;

	cc[3].nextLinkID[0][0] = 16;
	cc[3].nextLaneID[0][0] = 0;
	cc[3].nextLinkID[1][0] = 16;
	cc[3].nextLaneID[1][0] = 1;
	cc[3].nextLinkID[2][0] = 16;
	cc[3].nextLaneID[2][0] = 2;
	cc[3].nextLinkID[3][0] = 16;
	cc[3].nextLaneID[3][0] = 3;
	
	cc[4].nextLinkID[0][0] = 9;
	cc[4].nextLaneID[0][0] = 0;
	cc[4].nextLinkID[1][0] = 7;
	cc[4].nextLaneID[1][0] = 1;
	cc[4].nextLinkID[2][0] = 7;
	cc[4].nextLaneID[2][0] = 2;
	cc[4].nextLinkID[3][0] = 7;
	cc[4].nextLaneID[3][0] = 3;
	cc[4].nextLinkID[3][1] = 5;
	cc[4].nextLaneID[3][1] = 3;

	cc[5].nextLinkID[0][0] = 17;
	cc[5].nextLaneID[0][0] = 0;
	cc[5].nextLinkID[1][0] = 17;
	cc[5].nextLaneID[1][0] = 1;
	cc[5].nextLinkID[2][0] = 17;
	cc[5].nextLaneID[2][0] = 2;
	cc[5].nextLinkID[3][0] = 17;
	cc[5].nextLaneID[3][0] = 3;
	
	cc[6].nextLinkID[0][0] = 3;
	cc[6].nextLaneID[0][0] = 0;
	cc[6].nextLinkID[1][0] = 9;
	cc[6].nextLaneID[1][0] = 1;
	cc[6].nextLinkID[2][0] = 9;
	cc[6].nextLaneID[2][0] = 2;
	cc[6].nextLinkID[3][0] = 9;
	cc[6].nextLaneID[3][0] = 3;
	cc[6].nextLinkID[3][1] = 7;
	cc[6].nextLaneID[3][1] = 3;

	cc[7].nextLinkID[0][0] = 18;
	cc[7].nextLaneID[0][0] = 0;
	cc[7].nextLinkID[1][0] = 18;
	cc[7].nextLaneID[1][0] = 1;
	cc[7].nextLinkID[2][0] = 18;
	cc[7].nextLaneID[2][0] = 2;
	cc[7].nextLinkID[3][0] = 18;
	cc[7].nextLaneID[3][0] = 3;

	cc[8].nextLinkID[0][0] = 5;
	cc[8].nextLaneID[0][0] = 0;
	cc[8].nextLinkID[1][0] = 3;
	cc[8].nextLaneID[1][0] = 1;
	cc[8].nextLinkID[2][0] = 3;
	cc[8].nextLaneID[2][0] = 2;
	cc[8].nextLinkID[3][0] = 3;
	cc[8].nextLaneID[3][0] = 3;
	cc[8].nextLinkID[3][1] = 9;
	cc[8].nextLaneID[3][1] = 3;

	cc[9].nextLinkID[0][0] = 10;
	cc[9].nextLaneID[0][0] = 0;
	cc[9].nextLinkID[1][0] = 10;
	cc[9].nextLaneID[1][0] = 1;
	cc[9].nextLinkID[2][0] = 10;
	cc[9].nextLaneID[2][0] = 2;
	cc[9].nextLinkID[3][0] = 10;
	cc[9].nextLaneID[3][0] = 3;

	cc[10].nextLinkID[0][0] = 15;
	cc[10].nextLaneID[0][0] = 0;
	cc[10].nextLinkID[1][0] = 15;
	cc[10].nextLaneID[1][0] = 1;
	cc[10].nextLinkID[2][0] = 15;
	cc[10].nextLaneID[2][0] = 2;
	cc[10].nextLinkID[3][0] = 15;
	cc[10].nextLaneID[3][0] = 3;
	
	cc[11].nextLinkID[0][0] = 1;
	cc[11].nextLaneID[0][0] = 0;
	cc[11].nextLinkID[1][0] = 1;
	cc[11].nextLaneID[1][0] = 1;
	cc[11].nextLinkID[2][0] = 1;
	cc[11].nextLaneID[2][0] = 2;
	cc[11].nextLinkID[3][0] = 1;
	cc[11].nextLaneID[3][0] = 3;
	
	cc[12].nextLinkID[0][0] = 4;
	cc[12].nextLaneID[0][0] = 0;
	cc[12].nextLinkID[1][0] = 4;
	cc[12].nextLaneID[1][0] = 1;
	cc[12].nextLinkID[2][0] = 4;
	cc[12].nextLaneID[2][0] = 2;
	cc[12].nextLinkID[3][0] = 4;
	cc[12].nextLaneID[3][0] = 3;

	cc[13].nextLinkID[0][0] = 6;
	cc[13].nextLaneID[0][0] = 0;
	cc[13].nextLinkID[1][0] = 6;
	cc[13].nextLaneID[1][0] = 1;
	cc[13].nextLinkID[2][0] = 6;
	cc[13].nextLaneID[2][0] = 2;
	cc[13].nextLinkID[3][0] = 6;
	cc[13].nextLaneID[3][0] = 3;

	cc[14].nextLinkID[0][0] = 8;
	cc[14].nextLaneID[0][0] = 0;
	cc[14].nextLinkID[1][0] = 8;
	cc[14].nextLaneID[1][0] = 1;
	cc[14].nextLinkID[2][0] = 8;
	cc[14].nextLaneID[2][0] = 2;
	cc[14].nextLinkID[3][0] = 8;
	cc[14].nextLaneID[3][0] = 3;
	
	/// (3) Set traffic signals.

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
		numOLC_R--;
	}

	/// (5) Update values of vehOLC flags.
	int count_R = numOLC_R;
	int count_L = numOLC_L;
	if (numOLC_L == 0 && numOLC_R == 0);

	else if (numOLC_L == 0) {
		while (count_R) {
			int randVeh = rand()%numOLC_R;
			if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}
	}

	else if (numOLC_R == 0) {
		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}      	
	}

	else if ((possLeft/numOLC_L) > (possRight/numOLC_R)) {
		while (count_R) {
			int randVeh = rand()%numOLC_R;
			if (l->vehOLC[sect][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}

		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}      	
	}

	else if ((possLeft/numOLC_L) <= (possRight/numOLC_R)) {
		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[sect][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[sect][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}     

		while (count_R) {
			int randVeh = rand()%numOLC_R;
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
	        
	        /// numOLC cannot be bigger than total number of vehicle.
	        int numLC = numOLC_L + numOLC_R + numMLC_L + numMLC_R;
	        if(numLC > l->numVehArr[sect][lane]) numLC = l->numVehArr[sect][lane];
		    
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
				//MIN(MIN((l->ffSpeed / 3.6 * dt) / l->lenSection[sect] * l->numVeh[sect][lane], l->maxNumCF[sect][lane]), 
				//	MIN(l->maxNumCF[sect+1][lane], wSpeed * dt / length * (l->maxNumCF[sect][lane] - l->numVeh[sect][lane])));   
				MIN(l->numVehArr[sect][lane], MIN(l->maxNumCF[sect][lane], wSpeed / l->ffSpeed * (l->maxNumVeh[sect+1][lane] - l->numVehArr[sect+1][lane]))) * l->ffSpeed * dt / l->lenSection[sect];

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
void Find_Index(int* findArr, int findArrSize, int findValue) {
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

        for (int i = 0 ; i < MAX_VEC ; i++) {
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
	/// (1) tempArr of previous link -> connection cell
	int index = Find_Index(prevl->tempIDArr[currLane], 3, cc->nextLinkID[currLane][i]);

	if (index != -1) cc->numVehArr[currLane][i] = prevl->tempNumArr[currLane][index];

	/// (2) connection cell -> virtual cell of next link
	nextl->numVehArr[0][cc->nextLane[currLane][i]] = cc->numVehArr[currLane][i];
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_numCF()
/// @brief   Function that relay numCF of the virtual cell of a next 
///          link to the last cell of the previous link.
/// @param   link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int i
/// @return  None
/*--------------------------------------------------------------------*/
void Update_numCF(link* prevl, link* nextl, int nextLane, connection_cell* cc, int currLoop, int currLane, int i) {
	/// (1) virtual cell of next link -> connection cell
	if (cc->trafficSignal[currLane][currLoop] == 0) {
		cc->numCFArr[currLane][currLoop] = 0;
	}
	else cc->numCFArr[currLane][i] = nextl->numCF[0][nextLane];

	/// (2) connection cell -> previous link
	int index = Find_Index(prevl->tempIDArr[currLane], 3, cc->nextLinkID[currLane][i]);

	prevl->tempNumArr[currLane][index] = cc->numVehArr[currLane][i];
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

			if (tempIDArr[lane][i] == 0) {
				tempArrSize = i;
				break;
			}
		}

		int count = 0;
		int effNumCF = MAX_VEC;
		while (count < MIN(totalNumCF, maxNumCF)) {
			for (int i = 0 ; i < tempArrSize ; i++) {
				if (l->nextLinkIDArr[NUM_SECTION][lane][count] == l->tempIDArr[lane][i]) {
					if (l->tempNumArr[lane][i] == 0) {
						effNumCF = count;
						count = MAX_VEC;
						break;
					}
					
					l->tempNumArr[lane][i]--;
				}
				
			}

			count++;
		}

		l->numCF[NUM_SECTION][lane] = MIN(effNumCF, MIN(totalNumCF, maxNumCF));
	}
}
	

/*--------------------------------------------------------------------*/
/// @fn      void Update_vehCF()
/// @brief   Function that update numCF and vehCF Flags of a link using  
///          connection cell.
/// @param   link* l, connection_cell* cc, int count
/// @return  None
/*--------------------------------------------------------------------*/
void Update_vehCF(link* l) {
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
	for (int i = 0 ; i < numVeh ; i++) {
		cc->currLinkOrderArr[currLane][i] = prevl->currLinkOrderArr[NUM_SECTION+1][currLane][i];
		cc->nextLinkIDArr[currLane][i] = prevl->nextLinkIDArr[NUM_SECTION+1][currLane][i];
		cc->vehIDArr[currLane][i] = prevl->vehIDArr[NUM_SECTION+1][currLane][i]
	}
	
	/// (2) connection cell -> virtual cell of next link
	int count = 0;
	int currNumVeh = nextl->numVehArr[1][currLane];	
	for (int i = 0 ; i < numVeh ; i++) {
		int currOrder = cc->currLinkOrderArr[currLane][i];
		int currVehID = cc->vehIDArr[currLane][i];

		if (cc->currLinkOrderArr[currLane][i] == nextl->linkID) {
			int index = currNumVeh + count;
			nextl->vehIDArr[1][currLane][index] = cc->vehIDArr[currLane][i];
			nextl->currLinkOrderArr[1][currLane][index] = cc->currLinkOrderArr[currLane][i] + 1;
			nextl->nextLinkIDArr[1][currLane][index] = v[currvehID-1].path[currOrder+1];
			nextl->minTargetLaneArr[1][currLane][index] = v[currvehID-1].minTargetLane[currOrder+1];
			nextl->maxTargetLaneArr[1][currLane][index] = v[currvehID-1].maxTargetLane[currOrder+1];

			count++;
		}

	nextl->numVehArr[1][currLane] += count;
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_ConnectionCell()
/// @brief   Function that resets variables of connection cell.
/// @param   connection_cell* cc
/// @return  None
/*--------------------------------------------------------------------*/
void Reset_ConnectionCell(connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
	    for (int i = 0 ; i < MAX_VEC ; i++) {
	    	cc->numVehArr[lane][i] = 0;
	    	cc->numCFArr[lane][i] = 0;	
	    
	    	for (int j = 0 ; j < MAX_VEC ; j++) {
	    		cc->nextLinkIDArr[lane][j][i];
				cc->vehIDArr[lane][j][i];
				cc->currLinkOrderArr[lane][j][i];
	    	}	
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
			int density = l->numVehArr[sect][lane] /l->lenSection[sect];  //도로의 현재 density (veh/km), 시뮬레이션 스텝마다 업데이트

	    	l->speed[sect][lane] = MAX(0, MIN(l->ffSpeed, (-wSpeed + (wSpeed * jamdensity / density))));
	    	l->numMLCL[sect][lane] = 0;
			l->numMLCR[sect][lane] = 0;

			l->numVehArr[0][lane] = 0;
			l->numVehArr[NUM_SECTION+1][lane] = 0;
			
	    	for (int i = 0 ; i < MAX_VEC ; i++) {
	    		l->vehMLC[sect][lane][i] = 0;
				l->vehOLC[sect][lane][i] = 0;

				l->vehIDArr[NUM_SECTION+1][lane][i] = 0;
				l->currLinkOrderArr[NUM_SECTION+1][lane][i] = 0;
				l->minTargetLaneArr[NUM_SECTION+1][lane][i] = 0;
				l->maxTargetLaneArr[NUM_SECTION+1][lane][i] = 0;
	    	}
      	}
    }

    for (int sect = 0 ; sect < NUM_SECTION+1 ; sect++) {
    	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    		l->numCF[sect][lane] = 0;

    		for (int i = 0 ; i < MAX_VEC ; i++) {
	    		l->vehCF[sect][lane][i] = 0; 
	    	}
    	}
    }

    for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    	for (int i ; i < 3 ; i++) { 
    		tempIDArr[lane][i] = 0;
    		tempNumArr[lane][i] = 0;
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
void SimulationStep(vehicle* v, int numVeh, link l[], int numLink, connection_cell cc[], int numCC, int numLoop) {

    for (int count = 0 ; count < numLoop ; count++) {
        for (int link = 0 ; link < numLink ; link++) {
        	Reset_Link(&l[link]);
            Evaluate_MLC(&l[link]);
            Evaluate_OLC(&l[link]);
            LCSim(&l[link]);
            Update_tempArr(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		int prevLink = &l[cc[i].prevLinkID];
        		int nextLink = &l[cc[i].nextLinkID[lane][0]];
        		int nextLane = &l[cc[i].nextLane[lane][0]];
        		Relay_numVeh(prevLink, nextLink, nextLane, &cc[i], lane);

        		int nextLink = &l[cc[i].nextLinkID[lane][1]];
        		int nextLane = &l[cc[i].nextLane[lane][1]];
        		Relay_numVeh(prevLink, nextLink, nextLane, &cc[i], lane);

        		int nextLink = &l[cc[i].nextLinkID[lane][2]];
        		int nextLane = &l[cc[i].nextLane[lane][2]];
        		Relay_numVeh(prevLink, nextLink, nextLane, &cc[i], lane);
        	}

        	Reset_ConnectionCell(&cc[i]);
        }

        for (int link = 0 ; link < numLink ; link++) {
        	Evaluate_CF(&l[link]);
        }
	
        for (int i = 0 ; i < numCC ; i++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        		int prevLink = &l[cc[i].prevLinkID];
        		int nextLink = &l[cc[i].nextLinkID[lane][0]];
        		int nextLane = &l[cc[i].nextLane[lane][0]];
        		Update_numCF(prevLink, nextLink, nextLane, &cc[i], count, lane, 0);
        		
        		int nextLink = &l[cc[i].nextLinkID[lane][1]];
        		int nextLane = &l[cc[i].nextLane[lane][1]];
        		Update_numCF(prevLink, nextLink, nextLane, &cc[i], count, lane, 1);

        		int nextLink = &l[cc[i].nextLinkID[lane][2]];
        		int nextLane = &l[cc[i].nextLane[lane][2]];
        		Update_numCF(prevLink, nextLink, nextLane, &cc[i], count, lane, 2);
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
        		int prevLink = &l[cc[i].prevLinkID];
        		int nextLink = &l[cc[i].nextLinkID[lane][0]];
        		int nextLane = &l[cc[i].nextLane[lane][0]];
        		Update_nextLink(prevLink, nextLink, nextLane, &cc[i], count, lane, 0);

        		int nextLink = &l[cc[i].nextLinkID[lane][1]];
        		int nextLane = &l[cc[i].nextLane[lane][1]];
        		Update_nextLink(prevLink, nextLink, nextLane, &cc[i], count, lane, 1);

	        	int nextLink = &l[cc[i].nextLinkID[lane][2]];
        		int nextLane = &l[cc[i].nextLane[lane][2]];
        		Update_nextLink(prevLink, nextLink, nextLane, &cc[i], count, lane, 2);
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


void PrintAll (link l[], int numLink) {
	for (int link = 0 ; link < numLink ; link++) {
		printf("----------------------------------------\n");
		printf("link: %d, \n", link);

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    			printf("section: %d, lane: %d, numVeh: %d\n",
    				sect, lane, l[link].numVehArr[sect][lane]);
    		}
    	}
	}

	printf("========================================\n");
}


int main(int argc, char *argv[]) {
	srand(time(NULL));

	int numVeh = (int) atoi(argv[1]); // number of links
    int numLink = (int) atoi(argv[2]); // number of vehicle
    int numCC = (int) atoi(argv[3]);
    int numLoop = (int) atoi(argv[4]); //number of periods

	printf("========================================\n");
    printf("vehicle arguments: %s\n", argv[1]);
    printf("link arguments: %s\n", argv[2]);
    printf("connection cell arguments: %s\n", argv[3]);
	printf("loop count arguments: %s\n", argv[4]);
	
	myveh = (vehicle*) calloc(numVeh, sizeof(vehicle));
	mylink = (link*) calloc(numLink, sizeof(link));
	mycon = (connection_cell*) malloc(numLink * sizeof(connection_cell));

    double start, stop;

    Setup_Veh(myveh, numVeh);
    Setup_Link(myveh, numVeh, mylink, numLink);
    Setup_ConnectionCell(mycon, numCC);

    printf("========================================\n");
    printf("Initial State\n");
    PrintAll(mylink, numLink);

    start = get_time_ms();
    printf("Simulation Started\n");
    SimulationStep(myveh, numVeh, mylink, numLink, mycon, numCC, numLoop);
    printf("Simulation Finished\n");
    stop = get_time_ms();

    printf("========================================\n");
    printf("After Simulation\n");
    PrintAll(mylink, numLink);

    double result = stop - start;
    printf("Elapsed Time: %f\n\n", result);

    return 0;
}
