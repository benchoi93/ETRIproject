
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>
#include <math.h>
#include "TrafficSim.h"
#include <time.h>

/*
works to do
1. longitudinal movement function
 --> based on CTM function
 --> function
    --> input : (N, maxN, vf, maxY, w{wave speed}, dt)
    --> output : y {array}
2. lateral movement function
 --> LC probability : logit function (~ speed diff. / vf)
3. vehicle transmission function
*/


int current_simtime;
int dt = 5;

int external_y = 10;


link *mylink;
node *mynode;
vehicle *myveh;
connection_cell *mycon;

int main(int argc, char *argv[]) {

	// create array using args
	printf("link arguments: %s\n", argv[1]);
	int n = (int) atoi(argv[1]); // number of links

//	printf("threads in block arguments: %s\n", argv[2]);
//	int threadsInBlock = (int) atoi(argv[2]); // number of links

	printf("loop count arguments: %s\n", argv[3]);
	int loop_limit = (int) atoi(argv[3]); // number of links

	mylink = (link*) malloc(n*sizeof(link));
	mynode = (node*) malloc(n*sizeof(node));
	myveh = (vehicle*) malloc(n*sizeof(vehicle));
	mycon = (connection_cell*) malloc(n*sizeof(connection_cell));

    // read data
    double start, stop, interval;

    setup_roadlink();

    start = get_time_ms();
    // run simulation_step
    printf("Simulation Started\n");
    SimulationStep(mylink, n, loop_limit);
    printf("Simulation Finished\n");
    stop = get_time_ms();

    double result = stop - start;

    printf("Elapsed Time: %f\n\n", result);
    return 0;
}

void SimulationStep(link l[], int numberOfLinks, int loop_limit){
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
            Evaluate_MLC(&l[cur_link]);
            MLCsim(&l[cur_link]);

//            // 각 링크l[i]별로 Optioanl LC 처리
            Evaluate_OLC(&l[cur_link]);
            OLCsim(&l[cur_link]);

//            //각 링크l[i]별로 CTM SIM 처리
            CFsim(&l[cur_link]);
//
//            // 링크별 결과 전송
//
//            //전체 차량들에대해 셀이동 처리
            Vehicle_Move(&l[cur_link]);

            // write vehicle in connectCell
//            mycon[cur_link+1].connectN[0] += 1;
//            mycon[cur_link+1].connectN[1] += 1;
//            mycon[cur_link+1].connectN[2] += 1;
//            mycon[cur_link+1].connectN[3] += 1;

        }
    }
}

void CFsim(link *l, vehicle *v) {
    double w = 15;  //wave speed
    double L = 4;   //Section length

    int NoCell = NUM_SECTION;
    int NoLane = NUM_LANE;

    double Lmin = l->Vf / 3.6 * dt;

    for (int section = 0; section < NUM_SECTION; section++) {
        for (int lane = 0; lane < NUM_LANE; lane++) {
            l->Y[cell][lane] = MIN (MIN(Lmin/L * l->N[section][lane], l->MaxY[section][lane]), 
				    MIN(l->MaxY[section][lane+1], w*dt/L*(l->MaxN[section][lane]-l->N[section][lane])));
//                    MIN(
//                    MIN(Lmin / L[cell] * l->N[cell][lane], l->MaxY[cell][lane]),
//                    MIN(l->MaxY[cell][lane + 1], w * dt / L * (l->MaxN[cell][lane] - l->N[cell][lane])));
//            // moveforward flag update
		

        }
    }
    return;
}

void MLCsim(link* l) {
	double w = 15;  //wave speed

//	double L = l.SectionLength;
	double L = 70;
//	int NoSection = l.NoSection;
	int NumOfSection = 4;
//	int NoLane = l.NoLane;
	int NumberOfLane = 4;
	int dt = 5;
//	double Lmin = l.Vf/3.6 * dt;
	double Lmin = l->Vf / 3.6 * dt;

	for (int section = 0; section < NUM_SECTION; section++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			l->Y[section][lane] = 30;

			//l->Y[section][lane] = MIN( MIN( Lmin/L[section] * l->N[section][lane], l->maxY[section][lane]),	MIN( l->maxY[section][lane+1], w * dt / L * (l->maxN[section][lane] - l->N[section][lane] ));
			// moveforward flag update

		}
	}
	return;
}
void OLCsim(link* l) {
	double w = 15;  //wave speed

	//	double L = l.SectionLength;
	double L = 70;
	//	int NoSection = l.NoSection;
	int NumOfSection = 4;
	//	int NoLane = l.NoLane;
	int NumberOfLane = 4;
	int dt = 5;
	//	double Lmin = l.Vf/3.6 * dt;
	double Lmin = l->Vf / 3.6 * dt;
//
	for (int section = 0; section < NUM_SECTION; section++) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			l->Y[section][lane] = L * l->N[section][lane];

			//l->Y[section][lane] = MIN( MIN( Lmin/L[section] * l->N[section][lane], l->MaxY[section][lane]),
				//	MIN( l->MaxY[section][lane+1], w * dt / L * (l->MaxN[section][lane] - l->N[section][lane] ));


			//l->Y[section][lane] = 0;
//			l->Y[section][lane] = (min( min(Lmin/L[section] * l->N[section][lane], l.MaxY[section][lane]) < min( l->MaxY[section][lane+1], w * dt / L * (l->MaxN[section][lane] - l->N[section][lane] )) ? 10: 20;
 			              //			;
//			float temp_b = );
//
//			if (temp_a < temp_b)	{
//				l->Y[section][lane] = temp_a;
//			} else	{
//				l->Y[section][lane] = temp_b;
//			}
			// moveforward flag update
		}
	}
}


void Evaluate_MLC(link *l) {

    // --------------------------------------------------------------------------------------------------
    // Mandatory Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정
    // --------------------------------------------------------------------------------------------------
    for (int cell = 0; cell < NUM_SECTION; cell++) {
        for (int lane = 0; lane < NUM_LANE; lane++) {
            for (int i = 0; i < 20; i++) {
				l->Y[cell][lane]  = 30;

//                int veh = l->veh[cell][lane][i];
//                if (cell < lane)    {
//                    veh = 1;
//                    veh++;
//                } else{
//                    veh =0;
//                }




//                vehicle veh = l->veh[cell][lane][i]; // 차량데이터 가지고 오기
//
//                int TargetLaneLeft = veh.targetLane1[veh.currentLinkOrder];  // 타겟 레인 하한 가지고 오기
//                int TargetLaneRight = veh.targetLane2[veh.currentLinkOrder];  // 타겟 레인 상한 가지고 오기
//
//                if (veh.currentLane < TargetLaneLeft) {
//                    veh.lanechange = 1;
//                    // l.LC_Left[veh.currentCell][veh.currentLane]=1
//                }     // 오른쪽으로 차로 변경이 필요
//                else if (veh.currentLane > TargetLaneRight) {
//                    veh.lanechange = -1;
//                    // l[veh.currentLink].LC_Righft[veh.currentCell][veh.currentLane]=1;}  // 왼쪽으로 차로 변경이 필요
//                    else (veh.lanechange = 0;)
//                }
            }
        }
        // --------------------------------------------------------------------------------------------------
    }
   // return;
}

void Evaluate_OLC(link *l) {

    // --------------------------------------------------------------------------------------------------
    // Optional Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정
    // --------------------------------------------------------------------------------------------------

    for (int cell = 0; cell < NUM_SECTION; cell++) {
        for (int lane = 1; lane < NUM_LANE; lane++) {
        	l->LC_Left[cell][lane] += (l->V[cell][lane - 1] - l->V[cell][lane]) / l->Vf;
        }
    }
    //return;

}

void Vehicle_Move(link *l) {

    for (int cell=0; cell < 16; cell++)     {
        for(int lane=0; lane < 4; lane++)   {
    for (int vehID = 0; vehID < sizeof(vehID); vehID++) {
//        vehcle veh = v[vehID]; // 차량데이터 베이스에서  가지고 오기
        int veh = 0;
        // --------------------------------------------------------------------------------------------------
        // 차로변경이 있는 경우 차량의 현재 Cell과 링크를 업데이트 한다.
        //--------------------------------------------------------------------------------------------------

        // dummy code
        if (l->vehMoveForward[cell][lane][vehID] == 1)   {

        } else if (l->vehMoveForward[cell][lane][vehID]  == -1){

        } else if (l->vehMoveForward[cell][lane][vehID] == 0)   {

        }

        // real code
//        if (veh.Lanechange = +1) {}    // Move vehicle to left lane
//        if (veh.Lanechange = -1) {}    // Move vehicle to Right lane
//        if (veh.moveforward = 1) {}     // Move vehicle to frent cell

        // --------------------------------------------------------------------------------------------------


        // --------------------------------------------------------------------------------------------------
        // 차량이 다음셀로 전진하는 경우 차량의 현재 Cell과 링크를 업데이트 한다.
        //--------------------------------------------------------------------------------------------------

        if (l->vehMoveForward[cell][lane]== 1) {
            myveh[veh].currentCell++;
        } else if (l->vehMoveForward[cell][lane] == 4)  {
            myveh[veh].currentLinkOrder++;
            myveh[veh].currentLink = myveh[veh].path[myveh[veh].currentLinkOrder];
            myveh[veh].currentCell = 0;
        }
        l->N[cell][lane] = l->N[cell][lane] + 1;
        myveh[veh].currentCell++;
//        if (veh.moveForward == 1) {
//
//            //글로벌 메모리의 Vehicle 정보를 업데이트
//
//            if ((veh.currentCell == l[veh.currentLink].NOLANE)) {
//                //connection Cell로 바꾸기
//
//                // 현재 셀이 링크의 마지막셀인 경우
//                veh.currentLinkOrder++; // Path의 현재 링크 순서를 1 증가
//                veh.currentLink = veh.path[veh.currentLinkOrder];
//                veh.currentCell = 0;  // Cell position을 링크 시작점으로 \
//
//
//            } else {
//                // N Update
//
//
//                veh.currentCell++;   //마지막 셀이 아니면, 다음 셀로 차량을 옮긴다.
//            }
//        }
    }
    }}
     //--------------------------------------------------------------------------------------------------
   // return;
}

void setup_roadlink()   {
	//printf("Number of Links:\t%d\n", n);
	printf("Size of Links:\t%d\n", sizeof(link));

}
double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}
