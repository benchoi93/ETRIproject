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
	/// (1) Create new vehicles.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].vehID = i;
		v[i].vehType = 0;
		v[i].path[0] = 0;
		v[i].path[1] = 1;
		v[i].path[2] = 2;
		v[i].path[3] = 3;
		v[i].lenPath = 4;
	}

	/// vMinTL[] = minTargetLane
	int vMinTL[20][4] = {
	{0, 0, 1, 0}, 	{1, 2, 1, 0},	{1, 2, 1, 2},	{2, 2, 3, 3},
	{0, 0, 0, 0},	{0, 1, 0, 0},	{0, 1, 1, 3},	{0, 1, 2, 3},
	{0, 2, 2, 3},	{1, 2, 0, 0},	{1, 1, 0, 1},	{1, 2, 1, 2},
	{1, 2, 2, 3},	{1, 1, 1, 1},	{0, 0, 0, 0},	{2, 3, 0, 1},
	{2, 3, 1, 2},	{2, 3, 2, 3},	{2, 2, 3, 3},	{0, 0, 3, 3}  };

	/// vMaxTL[] = maxTargetLane
	int vMaxTL[20][4] = {
	{0, 1, 2, 1}, 	{1, 3, 2, 1},	{2, 2, 2, 2},	{3, 3, 3, 3},
	{0, 1, 1, 1},	{1, 2, 0, 0},	{0, 2, 2, 3},	{0, 1, 2, 3},
	{0, 3, 3, 3},	{1, 2, 1, 2},	{2, 2, 0, 1},	{1, 2, 1, 2},
	{1, 2, 2, 3},	{3, 3, 3, 3},	{3, 3, 3, 3},	{2, 3, 1, 2},
	{3, 3, 2, 2},	{3, 3, 3, 3},	{3, 3, 3, 3},	{2, 2, 3, 3}  };

	/// (2) Add information about target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		for (int j = 0 ; j < v[i].lenPath ; j++) {
			v[i].minTargetLane[j] = vMinTL[i][j];
			v[i].maxTargetLane[j] = vMaxTL[i][j];
		}
	}

	/// vPos[] = initLink, initLane, initSection
	int vPos[20][3] = {
	{0, 0, 1}, {0, 0, 2}, {0, 0, 3}, {0, 0, 4},
	{0, 1, 1}, {0, 1, 2}, {0, 1, 3}, {0, 1, 4},
	{0, 2, 1}, {0, 2, 2}, {0, 2, 3}, {0, 2, 4},
	{0, 3, 1}, {0, 3, 2}, {0, 3, 3}, {0, 3, 4},
	{0, 0, 1}, {0, 0, 2}, {0, 0, 3}, {0, 0, 4}  };

	/// (3) Add information about initial position.
	for (int i = 0 ; i < numVeh ; i++) {
		v[i].initLink = vPos[i][0];
		v[i].initLane = vPos[i][1];
		v[i].initSection = vPos[i][2];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Link(link*, int, vehicle*, int)
/// @brief   Function that setup the values of struct link.
/// @param   link* l, int numLink, vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Link(link* l, int numLink, vehicle* v, int numVeh) {
	/// (1) Create new links.
	for (int i = 0 ; i < numLink ; i++) {
		l[i].linkID = i;
		l[i].ffSpeed = 16; // m/s

		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
			l[i].lenSection[sect] = 100;

			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].maxNumVeh[sect][lane] = 20;
				l[i].numVehArr[sect][lane] = 0;
				l[i].speed[sect][lane] = l[i].ffSpeed;
				l[i].numMLCL[sect][lane] = 0;
	    		l[i].numMLCR[sect][lane] = 0;

	    		for (int j = 0 ; j < numVeh ; j++) {
			    	l[i].vehIDArr[sect][lane][j] = 0;
	    			l[i].currLinkOrderArr[sect][lane][j] = 0;
	    			l[i].minTargetLaneArr[sect][lane][j] = 0;
	    			l[i].maxTargetLaneArr[sect][lane][j] = 0;
	    			l[i].vehMLC[sect][lane][j] = 0;
	    			l[i].vehOLC[sect][lane][j] = 0;
	    			l[i].vehCF[sect][lane][j] = 0;
	    		}
			}
		}

		for (int sect = 0 ; sect < NUM_SECTION+1 ; sect++) {
			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].maxNumCF[sect][lane] = 3;
				l[i].numCF[sect][lane] = 0;
			}	
		}
	}

	/// (2) Add vehicles to links.
	for (int i = 0 ; i < numLink ; i++) {
    	for (int j = 0 ; j < numVeh ; j++) {
    		if (l[i].linkID == v[j].initLink) {
    			int x = v[j].initSection;
    			int y = v[j].initLane;
    			int z = l[i].numVehArr[x][y];

    			l[i].vehIDArr[x][y][z] = v[j].vehID;
    			l[i].minTargetLaneArr[x][y][z] = v[j].minTargetLane[0];
    			l[i].maxTargetLaneArr[x][y][z] = v[j].maxTargetLane[0];
    			l[i].numVehArr[x][y]++;
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
	/// (1) Create new connection cells.
	for (int i = 0 ; i < numCC ; i++) {
		cc[i].ccID = i;
		cc[i].prevLinkID = i;
		cc[i].nextLinkID = i+1;
	}

	/// (2) Initialize connection cells.
	for (int i = 0 ; i < numCC ; i++) {
		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
			cc[i].numVehArr[lane] = 0;
			cc[i].numCF[lane] = 0;
			for (int j = 0 ; j < MAX_VEC ; j++) {
				cc[i].vehIDArr[lane][j] = 0;
				cc[i].currLinkOrderArr[lane][j] = 0;
			}
		}
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
/// @param   
/// @return  
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
/// @brief   Function that 
/// @param   
/// @return  
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
/// @brief   Function that 
/// @param   
/// @return  
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
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Evaluate_CF(link* l) {
	double wSpeed = 4.2;
  
	for(int sect = 0 ; sect < NUM_SECTION+1 ; sect++) {
  		for(int lane = 0 ; lane < NUM_LANE ; lane++) {
      		l->numCF[sect][lane]= 
				//MIN(MIN((l->ffSpeed / 3.6 * dt) / l->lenSection[sect] * l->numVeh[sect][lane], l->maxNumCF[sect][lane]), 
				//	MIN(l->maxNumCF[sect+1][lane], wSpeed * dt / length * (l->maxNumCF[sect][lane] - l->numVeh[sect][lane])));   
				MIN(l->numVehArr[sect][lane], MIN(l->maxNumCF[sect][lane], wSpeed / l->ffSpeed * (l->maxNumVeh[sect+1][lane] - l->numVehArr[sect+1][lane]))) * l->ffSpeed * dt / l->lenSection[sect];

	      	for(int i = 0 ; i < l->numCF[sect][lane] ; i++) {
				l->vehCF[sect][lane][i] = 1;
      		}
    	}
  	}
}


/*--------------------------------------------------------------------*/
/// @fn      void MoveCF()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void MoveCF(int* fromArr, int fromArrSize, int* toArr, int toArrSize, int index) {
	toArr[toArrSize] = fromArr[index];

	for (int i = MAX_VEC-1 ; i >= 0 ; i--) {
		fromArr[i] = fromArr[i-1];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void CFsim()
/// @brief   Function that 
/// @param   
/// @return  
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
/// @fn      void Forward_Feedback()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_ConnectionCell(link* l, int sect, connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		cc->numVehArr[lane] = l->numVehArr[sect][lane];
		cc->numCF[lane] = l->numCF[sect][lane];

        for (int i = 0 ; i < MAX_VEC ; i++) {
        	cc->vehIDArr[lane][i] = l->vehIDArr[sect][lane][i];
        	cc->currLinkOrderArr[lane][i] = l->currLinkOrderArr[sect][lane][i];
	    }
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_VirtualCell()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_VirtualCell(link* l, connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        l->numVehArr[0][lane] = cc->numVehArr[lane];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_Link()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_FirstCell(link* l, connection_cell* cc, vehicle* v) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		int currNumVeh = l->numVehArr[1][lane];

        for (int i = 0 ; i < (MAX_VEC - currNumVeh) ; i++) {
        	int currOrder = cc->currLinkOrderArr[lane][i];
        	int currVehID = cc->vehIDArr[lane][i];

        	printf("vehIDArr  ");
        	l->vehIDArr[1][lane][currNumVeh+i] = cc->vehIDArr[lane][i];
        	printf("LinkOrderArr  ");
        	l->currLinkOrderArr[1][lane][currNumVeh+i] = cc->currLinkOrderArr[lane][i] + 1;
        	printf("TargetLaneArr  ");
        	l->minTargetLaneArr[1][lane][currNumVeh+i] = v[currVehID].minTargetLane[currOrder+1];
        	l->maxTargetLaneArr[1][lane][currNumVeh+i] = v[currVehID].maxTargetLane[currOrder+1];
	    }
	    printf("numVehArr  ");
	    l->numVehArr[1][lane] += cc->numVehArr[lane];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_vehCF()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_vehCF(link* l, connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
       	l->numCF[NUM_SECTION][lane] = cc->numCF[lane];
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_ConnectionCell()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Reset_ConnectionCell(connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
	    cc->numVehArr[lane] = 0;
	    cc->numCF[lane] = 0;

	    for (int i = 0 ; i < MAX_VEC ; i++) {
	    	cc->vehIDArr[lane][i] = 0;
	    	cc->currLinkOrderArr[lane][i] = 0;
	    }
    }
}


/*--------------------------------------------------------------------*/
/// @fn      void Reset_Link()
/// @brief   Function that 
/// @param   
/// @return  
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
			l->numCF[sect][lane] = 0;

			l->numVehArr[0][lane] = 0;
			l->numVehArr[NUM_SECTION+1][lane] = 0;
			
	    	for (int i = 0 ; i < MAX_VEC ; i++) {
	    		l->vehMLC[sect][lane][i] = 0;
				l->vehOLC[sect][lane][i] = 0;
				l->vehCF[sect][lane][i] = 0; 
				l->vehIDArr[NUM_SECTION+1][lane][i] = 0;
				l->currLinkOrderArr[NUM_SECTION+1][lane][i] = 0;
				l->minTargetLaneArr[NUM_SECTION+1][lane][i] = 0;
				l->maxTargetLaneArr[NUM_SECTION+1][lane][i] = 0;
	    	}
      	}
    }	
}


/*--------------------------------------------------------------------*/
/// @fn      void SimulationStep()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void SimulationStep(link l[], int numLink, connection_cell cc[], int numCC, vehicle* v, int numVeh, int loop_limit) {

    for (int count = 0 ; count < loop_limit ; count++) {
        for (int link = 0 ; link < numLink ; link++) {
        	printf("Evaluate LC  ");
            Evaluate_MLC(&l[link]);
            Evaluate_OLC(&l[link]);
            printf("LC Sim  ");
            LCSim(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	printf("Update_VirtualCell  ");
        	int prev = cc[i].prevLinkID;
        	int next = cc[i].nextLinkID;

        	Update_ConnectionCell(&l[prev], NUM_SECTION, &cc[i]);
        	Update_VirtualCell(&l[next], &cc[i]);
        	Reset_ConnectionCell(&cc[i]);
        }

        for (int link = 0 ; link < numLink ; link++) {
        	printf("Evaluate CF  ");
        	Evaluate_CF(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	printf("Update vehCF  ");
        	int prev = cc[i].prevLinkID;
        	int next = cc[i].nextLinkID;

        	Update_ConnectionCell(&l[next], 0, &cc[i]);
        	Update_vehCF(&l[prev], &cc[i]);
        	Reset_ConnectionCell(&cc[i]);
        }

        for (int link = 0 ; link < numLink ; link++) {
        	printf("CFsim  ");
        	CFsim(&l[link]);
        }

        for (int i = 0 ; i < numCC ; i++) {
        	printf("Update FirstCell  ");
        	int prev = cc[i].prevLinkID;
        	int next = cc[i].nextLinkID;

        	printf("Update 1  ");
        	Update_ConnectionCell(&l[prev], NUM_SECTION+1, &cc[i]);
        	printf("Update 2  ");
        	Update_FirstCell(&l[next], &cc[i], v);
        	printf("Update 3  ");
        	Reset_ConnectionCell(&cc[i]);
        }

        for (int link = 0 ; link < numLink ; link++) {
        	printf("Reset_Link  ");
            Reset_Link(&l[link]);
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
		for (int sect = 0 ; sect < NUM_SECTION+2 ; sect++) {
    		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    			printf("link: %d, section: %d, lane: %d, numVeh: %d \n",
    				link, sect, lane, l[link].numVehArr[sect][lane]);
    		}
    	}
	}
}

int main(int argc, char *argv[]) {

	srand(time(NULL));

	int numVeh = (int) atoi(argv[1]); // number of links
    int numLink = (int) atoi(argv[2]); // number of vehicle
    int numCC = (int) atoi(argv[3]);
    int loop_limit = (int) atoi(argv[4]); //number of periods

    printf("vehicle arguments: %s\n", argv[1]);
    printf("link arguments: %s\n", argv[2]);
    printf("connection cell arguments: %s\n", argv[3]);
	printf("loop count arguments: %s\n", argv[4]);
	
	myveh = (vehicle*) calloc(numVeh, sizeof(vehicle));
	mylink = (link*) calloc(numLink, sizeof(link));
	//mynode = (node*) calloc(n,sizeof(node));
	mycon = (connection_cell*) malloc(numLink * sizeof(connection_cell));

    double start, stop;

    Setup_Veh(myveh, numVeh);
    Setup_Link(mylink, numLink, myveh, numVeh);
    Setup_ConnectionCell(mycon, numCC);

    start = get_time_ms();
    printf("Simulation Started\n");
    SimulationStep(mylink, numLink, mycon, numCC, myveh, numVeh, loop_limit);
    printf("Simulation Finished\n");
    stop = get_time_ms();

    double result = stop - start;

    printf("Elapsed Time: %f\n\n", result);

    PrintAll(mylink, numLink);

    return 0;
}
