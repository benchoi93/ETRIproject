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
//connection_cell *mycon;

/*--------------------------------------------------------------------*/
/// @fn      void Setup_Veh(vehicle*, int)
/// @brief   Function that setup the values of struct vehicle.
/// @param   vehicle* v, int numVeh
/// @return  None
/*--------------------------------------------------------------------*/
void Setup_Veh(vehicle* v, int numVeh) {
	/// (1) Add inputs to vArr in order to create new vehicle.
	int numVar = 8;
	int vArr[numVeh][numVar];
	
	/// vArr[] = [vehType, vehID, path, 
	///             currLane, currSection, currLink, 
	///             minTargetLane, maxTargetLane];
	vArr[0] = [0, 0, 0, 3, 0, 0, 1, 2]; //go straight
	vArr[1] = [0, 1, 0, 2, 0, 0, 1, 2]; //go straight
	vArr[2] = [0, 2, 0, 0, 0, 0, 1, 2]; //go straight
	vArr[3] = [0, 3, 0, 0, 1, 0, 1, 2]; //go straight
	vArr[4] = [0, 4, 0, 1, 3, 0, 1, 2]; //go straight
	vArr[5] = [0, 5, 0, 3, 2, 0, 1, 2]; //go straight
	vArr[6] = [0, 6, 0, 2, 4, 0, 1, 2]; //go straight
	vArr[7] = [0, 7, 0, 1, 2, 0, 1, 2]; //go straight
	vArr[8] = [0, 8, 0, 3, 0, 0, 0, 0]; //turn left
	vArr[9] = [0, 9, 0, 1, 2, 0, 3, 3]; //turn right

	/// (2) Assign value of struct vehicle as values of vArr.
	for (int i = 0 ; i < numVeh ; i++) {
		int j = vArr[i][5];
		v[i].vehType = vArr[i][0];
    	v[i].vehID = vArr[i][1];
    	v[i].path[j] = vArr[i][2];
    	v[i].currLane = vArr[i][3];
    	v[i].currSection = vArr[i][4];
    	v[i].currLink = vArr[i][5];
    	v[i].minTargetLane[j] = vArr[i][6];
    	v[i].maxTargetLane[j] = vArr[i][7];
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
	int numVar = 5;
	int vArr[numVeh][numVar];
	
	/// lArr[] = [linkID, ffSpeed, maxNumVeh, maxNumCF, lenSection];
	lArr[0] = [0, 27, 20, 3, 100];

	/// (2) Assign value of struct link using values of lArr and struct vehicle.
	for (int i = 0 ; i < numLink ; i++) {
		l[i].linkID = lArr[i][0];
		l[i].ffSpeed = lArr[i][1];

		for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        	for (int lane = 0 ; lane < NUM_LANE ; lane++) {
		    	l[i].maxNumVeh[cell][lane] = lArr[i][2];
		    	l[i].maxNumCF[cell][lane] = lArr[i][3];
		    	l[i].lenSection[cell][lane] = lArr[i][4];

    	for (int j = 0 ; j < numVeh ; j++) {
    		if (l[i].linkID == v[j].currLink) {
    			int p = v[j].currSection;
    			int q = v[j].currLane;
    			int r = l[i].numVeh[p][q];

    			l[i].vehIDArr[p][q][r] = v[j].vehID;
    			l[i].currentLinkOrder[p][q][r] = v[j].currLinkOrder;
    			l[i].minTargetLaneArr[p][q][r] = v[j].minTargetLane[v[j].currLinkOrder];
    			l[i].maxTargetLaneArr[p][q][r] = v[j].maxTargetLane[v[j].currLinkOrder];
    			
    			l[i].speed[p][q] = l[i].ffSpeed;
    			l[i].numVeh[p][q]++;
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
void Evaluate_MLC(link *l, int numLink) {
 	for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++) {
        for (int lane = 0 ; lane < NUM_LANE ; lane++) {
            for (int i = 0 ; i < l->numVeh[cell][lane] ; i++) {

                int vID = l->vehIDArr[cell][lane][i];
                int minTL = l->minTargetLaneArr[cell][lane][i];
                int maxTL = l->maxTargetLaneArr[cell][lane][i];

                if (lane > maxTL) {
                    l->vehMLC[cell][lane][i] = -1;
                    l->numVehLeft[cell][lane]++;
                }
                else if (veh.currentLane < minTL) {
                    l->vehMLC[cell][lane][i] = 1;
                    l->numVehRight[cell][lane]++;
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

    srand(time(NULL));
    double random = ((rand() % 10)/10);

    return random > probPart ? (intPart+1):intPart
}


// OLC는 TARGET RANGE 안에서만 작동
void Select_Veh(int* OLC_veh,int NUMLC_Left, int NUMLC_Right,int* MLC_R,int* MLC_L, int N) {
    // select vehicle for OLC
    // among totNum of vehicle, select selectNum lf vehicle random.y
   	for (int i = 0 ; i < NUMLC_Left + NUMLC_Right ; i++) {
       OLC_veh[i] = -1;
   	}

    //geting number of vehicle w/ MLC
    int MLC_NUM=0;
    for (int i = 0 ; i < N ; i++) {
        if (MLC_R[i]==1 || MLC_L[i]==1) MLC_NUM++;
    }
    
    int i = N-MLC_NUM;
    int flag = 1;
    while (flag) {
    	srand(time(NULL));
      	double rannum= rand()%i;

       	for (int i = 0 ; i < NUMLC_Left + NUMLC_Right ; i++) {
			if (OLC_veh[i]==-1) {
			   OLC_veh[i]==rannum;
			   if (i==NUMLC_Left + NUMLC_Right-1) flag = 0;
			   break;
			}
	 		if (OLC_veh[i]==rannum) {
	   		break;
	 		}
       	}
    }
}


void Check_inEdge() {
	//check if the vehecle is in its target lane
}


void Evaluate_OLC(link *l) {
    // --------------------------------------------------------------------------------------------------
    // Optional Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정
    // --------------------------------------------------------------------------------------------------

    for (int cell = 0 ; cell < NUM_SECTION+2 ; cell++){    //+2 or not
        for (int lane = 0 ; lane < NUM_LANE ; lane++){
        	int numMLC_L = numVehLeft[cell][lane];
        	int numMLC_R = numVehRight[cell][lane];
            double probLC_L, probLC_R;
            int diffSpeed_L = l->speed[cell][lane-1] - l->speed[cell][lane];
            int diffSpeed_R = l->speed[cell][lane+1] - l->speed[cell][lane];

            if (lane == 0) probLC_L = 0;
            else probLC_L = (diffSpeed_L / l->ffSpeed) * l->numVeh[cell][lane];
            
            if (lane == NUM_LANE-1) probLC_R = 0;
            else probLC_R = (diffSpeed_R / l->ffSpeed) * l->numVeh[cell][lane];

            int numOLC_L = Evaluate_Prob(probLC_L) - numMLC_L;
            int numOLC_R = Evaluate_Prob(probLC_R) - numMLC_R;

		    if(numOLC_L < 0) numOLC_L = 0;
	        if(numOLC_R < 0) numOLC_R = 0;
	            
	        int numLC = numOLC_L + numOLC_R + numMLC_L + numMLC_R;
	        
	        if(numLC > l->numVeh[cell][lane]) numLC = l->numVeh[cell][lane];
		    
		    int OLC[MAX_VEC] = {};
		    Select_Veh(OLC, numOLC_L, numOLC_R, l->vehMLC[cell][lane], l->numVeh[cell][lane]);

	        for(int i = 0; i < numOLC_L + numOLC_R ; i++) {
	            for(int j = 0, int count = 0 ; j < l->numVeh[cell][lane] ; j++) {
	                if(OLC[i] == count) {
	                    if(count<numOLC_L)  l->vehOLC[cell][lane][j]= -1;
	                    else l->vehOLC[cell][lane][j]= 1;
	                        break;
	                }
	                count++;
	            }
	        }

        }
    }
}


void LCSim(link* l,vehicle* myveh, int m) {
    // check vehMLC's vehOLC's and update Lateral Movement
    // reset vehMLC's vehOLC's to 0
    // update N (only lateral movement)
    // update veh


    //update veh
    for(int i=0;i<NUM_SECTION+2;i++){
        for(int j=0;j<NUM_LANE;j++){
            for(int k=0;k<l->N[i][j];k++){  
                int vehID= l->veh[i][j][k];
                if((l->vehMLCRight[i][j][k]==1)||(l->vehOLC[i][j][k]==1)){
                    l->vehMLCRight[i][j][k]=0;
                    l->vehOLC[i][j][k]=0;
                    myveh[vehID].currentLane++;       //might be dangerous
                }
                else if((l->vehMLCLeft[i][j][k]==1)||(l->vehOLC[i][j][k]==-1)){
                    l->vehMLCLeft[i][j][k]=0;
                    l->vehOLC[i][j][k]=0;
                    myveh[vehID].currentLane--;       //might be dangerous
                }
            }
            l->N[i][j]=0; //reset the link
        }
    }


    //update link
    for(int i=0;i<m;i++){
        int cursection= myveh[i].currentSection;
        int curlane = myveh[i].currentLane;
        l->veh[cursection][curlane][l->N[cursection][curlane]]= myveh[i].vehID;
        l->N[cursection][curlane]++;
    }    
}


void Evaluate_CF(link* l,vehicle* myveh, int m) {
  double w = 15;  //wave speed
  
  //    double L = l->CellLength;
  double L = 4;
  double Lmin = l->Vf / 3.6 * dt;
  
  for(int cell=0;cell<NUM_SECTION+2;cell++){
    for(int lane=0; lane<NUM_LANE;lane++){
      l->Y[cell][lane]= 
	MIN(MIN(Lmin / l->SectionLength[cell] * l->N[cell][lane], l->MaxY[cell][lane]),MIN(l->MaxY[cell][lane + 1], w * dt / L * (l->MaxN[cell][lane] - l->N[cell][lane])));   
      for(int i=0;i<l->Y[cell][lane];i++){
	l->vehMoveForward[cell][lane][i]=1;
      }
    }
  }
}


void CFsim(link *l, vehicle* myveh,int m) {
  // evaluate CTM Function
  // update l->Y
  // set vehMoveForward Flag to 1



  //update vehicle status
  for (int cell = 0; cell < NUM_SECTION+2; cell++) {
    for (int lane = 0; lane < NUM_LANE; lane++) {
      for(int i=0;i<l->N[cell][lane];i++){
	if(l->vehMoveForward[cell][lane][i]==1){
	  myveh[l->veh[cell][lane][i]].currentSection++;                   //dangerous
	}
      }
      l->N[cell][lane]=0;       //reset the nubmer of car in the cell
    }
  }
  
  
  //update the link
  for(int i=0;i<m;i++){
    int cursec=myveh[i].currentSection;
    int curlane=myveh[i].currentLane;


    //if the car is at the end of the link,
    //it's skipped to be recored in the current link
    //to implement more complex simulation, the convention which represent the element of the car is not used currently is needy  ex)negative vehicle ID

    if(cursec==NUM_SECTION){
      continue;
    }
    l->veh[cursec][curlane][l->N[cursec][curlane]]=myveh[i].vehID;
    l->N[cursec][curlane]++;
  }
    
  //reset the link
  
  for (int cell = 0; cell < NUM_SECTION+2; cell++) {
    for (int lane = 0; lane < NUM_LANE; lane++) {
      l->vehNoMLCL[cell][lane]=0;
      l->vehNoMLCR[cell][lane]=0;
      for(int i=0;i<MAX_VEC;i++){               //really reall inefficient
	if(i<l->N[cell][lane]){
	  vehicle curveh= myveh[l->veh[cell][lane][i]];    //danger
	  l->targetLane1[cell][lane][i]=curveh.targetLane1[curveh.currentLink];            
	  l->targetLane1[cell][lane][i]=curveh.targetLane2[curveh.currentLink];          
	}
	l->vehMLCRight[cell][lane][i]=0;
	l->vehMLCLeft[cell][lane][i]=0;
	l->vehOLC[cell][lane][i]=0;
	l->vehMoveForward[cell][lane][i]=0;
      }
    }
  }

}


void SimulationStep(link l[], int numberOfLinks,vehicle* myveh, int vehn, int loop_limit){
    // int tid = threadIdx.x;
    // int i = blockIdx.x * blockDim.x + tid;

    for (int current = 0; current < loop_limit; current++) {
        for (int cur_link = 0; cur_link < numberOfLinks; cur_link++) {
            // read vehicle from connectCell (i'th vehicle in global memory )
//            mycon[cur_link].connectN[0] = 1;
//            mycon[cur_link].connectN[1] = 1;
//            mycon[cur_link].connectN[2] = 1;
//            mycon[cur_link].connectN[3] = 1;


            //NextCoonectionCell에서 signal 정보 읽어오기 => link

            //Previous ConnectionCell에서 Input 읽어오기 => link

            // 각 링크l[i]별로 Mandatory LC 처리
            PrintAll(&l[cur_link],myveh,vehn);
            Evaluate_MLC(&l[cur_link],myveh,vehn);
            

//            // 각 링크l[i]별로 Optioanl LC 처리
            Evaluate_OLC(&l[cur_link]);
//            OLCsim(&l[cur_link]);
 //           MLCsim(&l[cur_link]);

            //update the status in vehicle(w/ LC), then update link based on it
            LCSim(&l[cur_link],myveh,vehn);
//            //각 링크l[i]별로 CTM SIM 처리
            PrintAll(&l[cur_link],myveh,vehn);
            Evaluate_CF(&l[cur_link],myveh,vehn);
            CFsim(&l[cur_link],myveh,vehn);
            PrintAll(&l[cur_link],myveh,vehn);
//            // 링크별 결과 전송
//
//            //전체 차량들에대해 셀이동 처리
//            Vehicle_Move(&l[cur_link]);

            // write vehicle in connectCell
//            mycon[cur_link+1].connectN[0] += 1;
//            mycon[cur_link+1].connectN[1] += 1;
//            mycon[cur_link+1].connectN[2] += 1;
//            mycon[cur_link+1].connectN[3] += 1;
        }
    }
}


void PrintAll(link* l, vehicle* veh, int m){
    
    printf("\n\n\n----------------PRINT ALL-------------\n");

    for(int i=0;i<NUM_SECTION+2;i++){
        for(int j=0;j<NUM_LANE;j++){
            printf("Cell <%d, %d>\t",i,j);
            printf("N:%d\tV:%g\tY:%g\n",l->N[i][j],l->V[i][j],l->Y[i][j]);
            for(int k=0;k<l->N[i][j];k++){
                printf("ID:%d MLCR:%d MLCL:%d OLC:%d For:%d\n",l->veh[i][j][k],l->vehMLCRight[i][j][k],
                l->vehMLCLeft[i][j][k],l->vehOLC[i][j][k],l->vehMoveForward[i][j][k]);
                printf("T1:%d T2:%d\n",l->targetLane1[i][j][k],l->targetLane2[i][j][k]);
            }
        }
    }

    printf("\n<My vehicle>\n");
    for(int i=0; i<m;i++){
        printf("%dth car//",i);
        printf("ID:%d (%d, %d) currentLink:%d\n",veh[i].vehID,
	       veh[i].currentSection,veh[i].currentLane,veh[i].currentLink);
    }
    printf("--------------------------------------------\n\n\n");
}


double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}


int main(int argc, char *argv[]) {

	// create array using args
	printf("link arguments: %s\n", argv[1]);
	int numLink = (int) atoi(argv[1]); // number of links
    int numVeh = (int) atoi(argv[2]); // number of vehicle
//	printf("threads in block arguments: %s\n", argv[2]);
//	int threadsInBlock = (int) atoi(argv[2]); // number of links

	printf("loop count arguments: %s\n", argv[3]);
	int loop_limit = (int) atoi(argv[3]); //number of periods

	mylink = (link*) calloc(n,sizeof(link));
	mynode = (node*) calloc(n,sizeof(node));
	myveh = (vehicle*) calloc(m,sizeof(vehicle));
	//mycon = (connection_cell*) malloc(n*sizeof(connection_cell));

    // read data
    double start, stop, interval;

    Setup_Veh(myveh, numVeh);
    Setup_Link(mylink, numLink, myveh, numVeh);

    start = get_time_ms();
    // run simulation_step
    printf("Simulation Started\n");
    SimulationStep(mylink, n,myveh,m ,loop_limit);
    printf("Simulation Finished\n");
    stop = get_time_ms();

    double result = stop - start;

    printf("Elapsed Time: %f\n\n", result);
    return 0;
}
