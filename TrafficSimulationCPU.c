#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "TrafficSim.h"

link *mylink;
vehicle *myveh;
//node *mynode;
connection_cell *mycon;

/*--------------------------------------------------------------------*/
/// @fn      void Setup_Veh(vehicle*, int)
/// @brief   Function that setup the values of struct vehicle.
/// @param   vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Veh(vehicle* v, int numVeh) {
	/// (1) Add inputs to vArr in order to create new vehicle.
	for (int i = 0 ; i < 20 ; i++) {
		v[i].vehType = 0;
		v[i].vehID = i;
		v[i].path[0] = 0;
		v[i].path[1] = 1;
		v[i].lenPath = 2;
		v[i].currLink = 0;
	}

	/// vPos[] = currLane, currSection
	int vPos[20][2] = {
	{0, 0}, {0, 1},	{0, 2}, {0, 3},
	{1, 0},	{1, 1},	{1, 2},	{1, 3},
	{2, 0},	{2, 1},	{2, 2},	{2, 3},
	{3, 0},	{3, 1},	{3, 2},	{3, 3},
	{0, 0},	{0, 1},	{0, 2},	{0, 3}	};

	/// 
	for (int i = 0 ; i < 20 ; i++) {
		v[i].currLane = vPos[i][0];
		v[i].currSection = vPos[i][1];
	}

	/// vTL[] = minTargetLane[0], maxTargetLane[0], minTargetLane[1], maxTargetLane[1]
	int vTL[20][4] = {
	{0, 0, 0, 0}, 	{1, 1, 1, 1},	{2, 2, 2, 2},	{3, 3, 3, 3},
	{0, 1, 0, 0},	{0, 1, 0, 1},	{0, 1, 1, 2},	{0, 1, 2, 3},
	{0, 1, 3, 3},	{1, 2, 0, 0},	{1, 2, 0, 1},	{1, 2, 1, 2},
	{1, 2, 2, 3},	{1, 2, 3, 3},	{2, 3, 0, 0},	{2, 3, 0, 1},
	{2, 3, 1, 2},	{2, 3, 2, 3},	{2, 3, 3, 3},	{0, 0, 3, 3} };

	/// 
	for (int i = 0 ; i < 20 ; i++) {
		v[i].minTargetLane[0] = vTL[i][0];
		v[i].maxTargetLane[0] = vTL[i][1];
		v[i].minTargetLane[1] = vTL[i][2];
		v[i].maxTargetLane[1] = vTL[i][3];
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Setup_Link(link*, int, vehicle*, int)
/// @brief   Function that setup the values of struct link.
/// @param   link* l, int numLink, vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Link(link* l, int numLink, vehicle* v, int numVeh) {
	/// (1) Add inputs to vArr in order to create new vehicle.
	/// lArr[] = [linkID, maxNumVeh, maxNumCF, ffSpeed, lenSection];
	int lArr[2][5] = { {0, 20, 3, 27, 100}, {1, 20, 3, 27, 100}	};

	/// (2) Assign value of struct link using values of lArr and struct vehicle.
	for (int i = 0 ; i < 2 ; i++) {
		l[i].linkID = lArr[i][0];
		l[i].ffSpeed = lArr[i][3];

		for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
			l[i].lenSection[cell] = lArr[i][4];

        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		    	l[i].maxNumVeh[cell][lane] = lArr[i][1];
		    	l[i].maxNumCF[cell][lane] = lArr[i][2];
		    }
		}

		///
		for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
			for (int lane = 0 ; lane < NUM_LANE ; lane++) {
				l[i].numVeh[cell][lane] = 0;
				l[i].numMLCL[cell][lane] = 0;
	    		l[i].numMLCR[cell][lane] = 0;
	    		l[i].numCF[cell][lane] = 0;
				l[i].speed[cell][lane] = 0;

				for (int j = 0 ; j < numVeh ; j++) {
			    	l[i].vehIDArr[cell][lane][j] = 0;
	    			l[i].currLinkOrderArr[cell][lane][j] = 0;
	    			l[i].minTargetLaneArr[cell][lane][j] = 0;
	    			l[i].maxTargetLaneArr[cell][lane][j] = 0;
	    			l[i].vehMLC[cell][lane][j] = 0;
	    			l[i].vehOLC[cell][lane][j] = 0;
	    			l[i].vehCF[cell][lane][j] = 0;
	    		}
		    }
		}

		/// 
    	for (int j = 0 ; j < numVeh ; j++) {
    		if (l[i].linkID == v[j].currLink) {
    			int p = v[j].currSection;
    			int q = v[j].currLane;
    			int r = l[i].numVeh[p][q];

    			l[i].vehIDArr[p][q][r] = v[j].vehID;
    			l[i].currLinkOrderArr[p][q][r] = 0;
    			l[i].minTargetLaneArr[p][q][r] = v[j].minTargetLane[0];
    			l[i].maxTargetLaneArr[p][q][r] = v[j].maxTargetLane[0];
    			l[i].speed[p][q] = l[i].ffSpeed;
    			l[i].numVeh[p][q]++;
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
void Setup_ConnectionCell(connection_cell* cc) {
	cc[0].ccID = 0;
	cc[0].prevLinkID = 0;
	cc[0].nextLinkID = 1;

	cc[1].ccID = 1;
	cc[1].prevLinkID = 1;
	cc[1].nextLinkID = 0;

	for (int lane = 0 ; lane < NUM_LANE-1 ; lane++) {
		cc[0].numVeh[lane] = 0;
		for (int i = 0 ; i < MAX_VEC-1 ; i++) {
			cc[0].vehIDArr[lane][i] = 0;
			cc[0].currLinkOrderArr[lane][i] = 0;
		}
	}

	for (int lane = 0 ; lane < NUM_LANE-1 ; lane++) {
		cc[1].numVeh[lane] = 0;
		for (int i = 0 ; i < MAX_VEC-1 ; i++) {
			cc[1].vehIDArr[lane][i] = 0;
			cc[1].currLinkOrderArr[lane][i] = 0;
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
 	for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            for (int i = 0 ; i < l->numVeh[cell][lane] ; i++) {
            	int minTL = l->minTargetLaneArr[cell][lane][i];
	            int maxTL = l->maxTargetLaneArr[cell][lane][i];

	            /// (1) Compare current lane with target lane and determine
	            ///     and update the value of vehMLC Flag.
	            /// If vehicle should move left, set vehMLC Flag to -1
	            if (lane > maxTL) {
	                l->vehMLC[cell][lane][i] = -1;
	                l->numMLCL[cell][lane]++;
	            }
	            /// If vehicle should move left, set vehMLC Flag to +1
	            else if (lane < minTL) {
	                    l->vehMLC[cell][lane][i] = 1;
	                    l->numMLCR[cell][lane]++;
	            }
	            else {
	            	l->vehMLC[cell][lane][i] = 0;
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
    int intPart = (int)inputProb;
    double probPart = inputProb - (double)intPart;

    double random = ((rand() % 10)/10.);

    return random > probPart ? (intPart+1):intPart;
}


/*--------------------------------------------------------------------*/
/// @fn      void Select_Veh()
/// @brief   Function that select vehicles of Optional Lane Change.
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Select_Veh(link* l, int numOLC_L, int numOLC_R, int cell, int lane) {
	int numVeh = l->numVeh[cell][lane];
	int possOLC[MAX_VEC] = {0};

	/// (1) Exclude vehMLC from candidates that can OLC.
	for (int i = 0 ; i < numVeh ; i++) {
		if (l->vehMLC[cell][lane][i] != 0) possOLC[i] = 0;
		else possOLC[i] = 2;
	}

	/// (2) Consider when current lane is either maximum or minimum target lane.
	for (int i = 0 ; i < numVeh ; i++) {
		int minTL = l->minTargetLaneArr[cell][lane][i];
        int maxTL = l->maxTargetLaneArr[cell][lane][i];

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
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}
	}

	else if (numOLC_R == 0) {
		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}      	
	}

	else if ((possLeft/numOLC_L) > (possRight/numOLC_R)) {
		while (count_R) {
			int randVeh = rand()%numOLC_R;
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
				count_R--;
			}
		}

		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}      	
	}

	else if ((possLeft/numOLC_L) <= (possRight/numOLC_R)) {
		while (count_L) {
			int randVeh = rand()%numOLC_L;
			if (l->vehOLC[cell][lane][possLeftArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possLeftArr[randVeh]] = -1;
				count_L--;
			}
		}     

		while (count_R) {
			int randVeh = rand()%numOLC_R;
			if (l->vehOLC[cell][lane][possRightArr[randVeh]] == 0) {
				l->vehOLC[cell][lane][possRightArr[randVeh]] = 1;
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
    for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	int numMLC_L = l->numMLCL[cell][lane];
        	int numMLC_R = l->numMLCR[cell][lane];
            double probLC_L, probLC_R;
            int diffSpeed_L = l->speed[cell][lane-1] - l->speed[cell][lane];
            int diffSpeed_R = l->speed[cell][lane+1] - l->speed[cell][lane];

            /// (1) Set probLC to zero in special cases. 
            /// When current lane is leftmost, probLC_L = 0
            /// In other words, the vehicle cannot move left.
            if (lane == 0) probLC_L = 0;
            else probLC_L = (diffSpeed_L / l->ffSpeed) * l->numVeh[cell][lane];
            
            /// When current lane is rightmost, probLC_R = 0
            /// In other words, the vehicle cannot move right.
            if (lane == NUM_LANE-1) probLC_R = 0;
            else probLC_R = (diffSpeed_R / l->ffSpeed) * l->numVeh[cell][lane];

            /// (2) Evaluate number of OLC by subtrating number of MLC
            ///     from the total number of LC.
            int numOLC_L = Evaluate_Prob(probLC_L) - numMLC_L;
            int numOLC_R = Evaluate_Prob(probLC_R) - numMLC_R;

            /// numOLC cannot be smaller than zero.
		    if(numOLC_L < 0) numOLC_L = 0;
	        if(numOLC_R < 0) numOLC_R = 0;
	        
	        /// numOLC cannot be bigger than total number of vehicle.
	        int numLC = numOLC_L + numOLC_R + numMLC_L + numMLC_R;
	        if(numLC > l->numVeh[cell][lane]) numLC = l->numVeh[cell][lane];
		    
		    /// (3) Select vehicle to change lane.
		    Select_Veh(l, numOLC_L, numOLC_R, cell, lane);
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
    for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehMLC[cell][lane][i] == 1) {
	        		if (l->numVeh[cell][lane+1] < MAX_VEC) {
	        			MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane], 
	        				l->vehIDArr[cell][lane+1], l->numVeh[cell][lane+1], i+1);
		        		MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane], 
		        			l->currLinkOrderArr[cell][lane+1], l->numVeh[cell][lane+1], i);
		        		MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
		        			l->minTargetLaneArr[cell][lane+1], l->numVeh[cell][lane+1], i);
		        		MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
		        			l->maxTargetLaneArr[cell][lane+1], l->numVeh[cell][lane+1], i);
		        		l->numVeh[cell][lane+1]++;
		        		l->numVeh[cell][lane]--;
	        		}
	        	} 
	        		
	        	else if (l->vehMLC[cell][lane][i] == -1) {
	        		if (l->numVeh[cell][lane-1] < MAX_VEC) {
						MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane], 
	        				l->vehIDArr[cell][lane-1], l->numVeh[cell][lane-1], i);
		        		MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane], 
		        			l->currLinkOrderArr[cell][lane-1], l->numVeh[cell][lane-1], i);
		        		MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
		        			l->minTargetLaneArr[cell][lane-1], l->numVeh[cell][lane-1], i);
		        		MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
		        			l->maxTargetLaneArr[cell][lane-1], l->numVeh[cell][lane-1], i);
		        		l->numVeh[cell][lane-1]++;
		        		l->numVeh[cell][lane]--;
	        		}
	        	}
	        }
        }
    }  

    for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehOLC[cell][lane][i] == 1 && l->numVeh[cell][lane] < MAX_VEC) {
	        		MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane], 
	        			l->vehIDArr[cell][lane+1], l->numVeh[cell][lane+1], i);
	        		MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane], 
	        			l->currLinkOrderArr[cell][lane+1], l->numVeh[cell][lane+1], i);
	        		MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->minTargetLaneArr[cell][lane+1], l->numVeh[cell][lane+1], i);
	        		MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->maxTargetLaneArr[cell][lane+1], l->numVeh[cell][lane+1], i);
	        		l->numVeh[cell][lane+1]++;
	        		l->numVeh[cell][lane]--;
	        	}
	        	else if (l->vehOLC[cell][lane][i] == -1 && l->numVeh[cell][lane] < MAX_VEC) {
	        		MoveLC(l->vehIDArr[cell][lane], l->numVeh[cell][lane], 
	        			l->vehIDArr[cell][lane-1], l->numVeh[cell][lane-1], i);
	        		MoveLC(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane], 
	        			l->currLinkOrderArr[cell][lane-1], l->numVeh[cell][lane-1], i);
	        		MoveLC(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->minTargetLaneArr[cell][lane-1], l->numVeh[cell][lane-1], i);
	        		MoveLC(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->maxTargetLaneArr[cell][lane-1], l->numVeh[cell][lane-1], i);
	        		l->numVeh[cell][lane-1]++;
	        		l->numVeh[cell][lane]--;
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
	double wSpeed = 15;
	//double length = 4;
  
	for(int cell = 0 ; cell < NUM_SECTION+1 ; cell++) {
  		for(int lane = 0 ; lane < NUM_LANE ; lane++) {
      		l->numCF[cell][lane]= 
				//MIN(MIN((l->ffSpeed / 3.6 * dt) / l->lenSection[cell] * l->numVeh[cell][lane], l->maxNumCF[cell][lane]), 
				//	MIN(l->maxNumCF[cell+1][lane], wSpeed * dt / length * (l->maxNumCF[cell][lane] - l->numVeh[cell][lane])));   
				MIN(l->numVeh[cell][lane], MIN(l->maxNumCF[cell][lane], wSpeed / l->ffSpeed * (l->maxNumCF[cell+1][lane] - l->numVeh[cell+1][lane])));
	      	
			printf("numCF: %d\n", l->numCF[cell][lane]);

	      	for(int i = 0 ; i < l->numCF[cell][lane] ; i++) {
				l->vehCF[cell][lane][i] = 1;
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
	for (int cell = NUM_SECTION ; cell >= 0 ; cell--) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        	for (int i = 0 ; i < MAX_VEC ; i++) {
	        	if (l->vehCF[cell][lane][i] == 1 && l->numVeh[cell+1][lane] < MAX_VEC) {
	        		MoveCF(l->vehIDArr[cell][lane], l->numVeh[cell][lane], 
	        			l->vehIDArr[cell+1][lane], l->numVeh[cell+1][lane], i);
	        		MoveCF(l->currLinkOrderArr[cell][lane], l->numVeh[cell][lane], 
	        			l->currLinkOrderArr[cell+1][lane], l->numVeh[cell+1][lane], i);
	        		MoveCF(l->minTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->minTargetLaneArr[cell+1][lane], l->numVeh[cell+1][lane], i);
	        		MoveCF(l->maxTargetLaneArr[cell][lane], l->numVeh[cell][lane], 
	        			l->maxTargetLaneArr[cell+1][lane], l->numVeh[cell+1][lane], i);
	        		l->numVeh[cell+1][lane]++;
	        		l->numVeh[cell][lane]--;
	        	}
	        }
	    }
	}
}


/*--------------------------------------------------------------------*/
/// @fn      void Update_ConnectCell()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_ConnectionCell(link* prevl, connection_cell* cc) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		cc->numVeh[lane] = prevl->numVeh[NUM_SECTION+1][lane];

        for (int i = 0 ; i < MAX_VEC ; i++) {
        	cc->vehIDArr[lane][i] = prevl->vehIDArr[NUM_SECTION+1][lane][i];
        	cc->currLinkOrderArr[lane][i] = prevl->currLinkOrderArr[NUM_SECTION+1][lane][i];
	    }
	}
}

/*--------------------------------------------------------------------*/
/// @fn      void Update_Link()
/// @brief   Function that 
/// @param   
/// @return  
/*--------------------------------------------------------------------*/
void Update_Link(link* nextl, connection_cell* cc, vehicle* v) {
	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
        for (int i = 0 ; i < MAX_VEC ; i++) {
        	int currNumVeh = nextl->numVeh[0][lane];
        	int currOrder = cc->currLinkOrderArr[lane][i];
        	int currVehID = cc->vehIDArr[lane][i];

        	nextl->vehIDArr[0][lane][currNumVeh+i] = cc->vehIDArr[lane][i];
        	nextl->minTargetLaneArr[0][lane][currNumVeh+i] = v[currVehID].minTargetLane[currOrder+1];
        	nextl->maxTargetLaneArr[0][lane][currNumVeh+i] = v[currVehID].maxTargetLane[currOrder+1];
	    }

	    nextl->numVeh[0][lane] += cc->numVeh[lane];
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
	    cc->numVeh[lane] = 0;

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
	double wSpeed = 15;

	for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
    	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    		int jamdensity = l->maxNumVeh[cell][lane] / l->lenSection[cell];  //도로가 막히기 시작하는 density (veh/km), 링크 특성(고정값)
			int density = l->numVeh[cell][lane] /l->lenSection[cell];  //도로의 현재 density (veh/km), 시뮬레이션 스텝마다 업데이트

	    	l->speed[cell][lane] =
	    		MAX(0, MIN(l->ffSpeed, (-wSpeed + (wSpeed * jamdensity / density))));
	    	l->numMLCL[cell][lane] = 0;
			l->numMLCR[cell][lane] = 0;
			l->numCF[cell][lane] = 0;

	    	l->numVeh[NUM_SECTION+1][lane] = 0;

	    	for (int i = 0 ; i < MAX_VEC ; i++) {
	    		l->vehMLC[cell][lane][i] = 0;
				l->vehOLC[cell][lane][i] = 0;
				l->vehCF[cell][lane][i] = 0; 
	
				l->vehIDArr[NUM_SECTION+1][lane][i] = 0;
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

            //PrintAll(&l[link],myveh,vehn);
            Evaluate_MLC(&l[link]);
            Evaluate_OLC(&l[link]);

            LCSim(&l[link]);

            Evaluate_CF(&l[link]);

            CFsim(&l[link]);
        }

        for (int cell = 0 ; cell < numCC ; cell++) {
        	int prev = cc[cell].prevLinkID;
        	int next = cc[cell].nextLinkID;

        	printf("prev: %d", prev);
        	printf("next: %d", next);

        	Update_ConnectionCell(&l[prev], &cc[cell]);
        	Update_Link(&l[next], &cc[cell], v);
        	Reset_ConnectionCell(&cc[cell]);
        }

        for (int link = 0 ; link < numLink ; link++) {	
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
		for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
    		for (int lane = 0 ; lane < NUM_LANE ; lane++) {
    			printf("link: %d, cell: %d, lane: %d, numVeh: %d \n",
    				link, cell, lane, l[link].numVeh[cell][lane]);
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
    Setup_ConnectionCell(mycon);

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
