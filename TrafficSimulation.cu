#include <stdlib.h>
#include <stdio.h>

#test branch


#define MAX(a,b) \  
({ __typeof__ (a) _a = (a); \  
__typeof__ (b) _b = (b); \  
_a > _b ? _a : _b; })  
  
#define MIN(a,b) \  
({ __typeof__ (a) _a = (a); \  
__typeof__ (b) _b = (b); \  
_a < _b ? _a : _b; })


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

int dt = 5;     //time step

int maxNconst = 20;
float maxYconst = 1800;
float Vfconst = 50;
float CellLengthconst = 100;

/*typedef struct {
	// about link
	int linkID;
	int fromNode;
	int toNode;
	int speedlimit;
	int roadlevel;	// 고속국도, 국도, 지방도
	int road_type;
	// about section
	int sectionID;
	int no_lanes;
	int cell_id;
	int speed;
	int numberOfVehicle;
	int length;
	int nextSectionID;
	int startSectionID;
	int endSectionID;
	int distanceToNode;
	int busLane;
	// yin
	int y_in;
	// lane_change
	// int lane_change;		// no need
	// insert vehicle
} lane_cell;
*/

typedef struct {
	int NoLane;
	int NoCell;
	int N[NoCell][NoLane];  		// 2D Array [NoCell	,NoLane]
	int MaxN[NoCell][NoLane];		// 2D Array [NoCell	,NoLane]
	int LC_Left[NoCell][NoLane]; 	// 2D Array [NoCell	,NoLane]
	int LC_Right[NoCell][NoLane]; 	// 2D Array [NoCell	,NoLane]
	float V[NoCell][NoLane];		// 2D Array [NoCell	,NoLane]
	float Vf[NoCell][NoLane];		// 2D Array [NoCell	,NoLane]
	float Y[NoCell+1][NoLane];		// 2D Array [NoCell+1	,NoLane]
	float MaxY[NoCell+1][NoLane];		// 2D Array [NoCell	,NoLane]
	float CellLength[NoCell];
		
		
	int NextLink[NoLane]
	int NextLane[NoLane]
		
} link;


typedef struct {
	int nodeID;
	int type; 	// intersection, source, sink
	int speedlimit;
	int intersection_type;
	//connection_info
} node;

typedef struct {
	// current position
	int currentLane;
	int currentCell;      // 현재 cell, 링크 시작점이 0,  
	int currentLink;      // 현재 링크 ID 
	int currentLinkOrder; // path[]에서 현재 링크 순서  
	// int distanceToNode;

	int type;
	
	int MandatoryLC;
	int moveForward;  // 1이면 시뮬레이션 시 다음셀로 차량을 이동할 필요  
	
//	int freeflowspeed;
//	int minSpacing;
//	int reactionTime;
//	int driving_moment;
	
	int lanechange;  // 1이면 오른쪽으로 차로변경, -1이면 왼쪽으로 변경이 필요 
	
	int path[20];  // Array of Link IDs EX) [15, 17, 19,...,0,0] 
	int NoLinksinPath;  //size of array path path[NoLinksinPath]  path 의 데이터 크기 
	int targetLane1[20]; // minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
	int targetLane2[20]; // max Target Lane  Ex) 3   타겟 레인 가안 값 설정 
	
	
} vehicle;

/* 
*/


typedef struct {
	int startLinkID;
	int startSectionID;
	int startLaneID;
	int endLinkID;
	int endSectionID;
	int endLaneID;

	int greenTime;
	int LeftTime;
	int yellowTime;
	int redTime;
	int offset;
} turning_info;

__global__ void simulationStep(int loop_limit, link *l, node *n,
		vehicle *v) {
	int tid = threadIdx.x;
	int i = blockIdx.x * blockDim.x + tid;

	// simulation time
	for (int current = 0; current < loop_limit; current++) {
			
		// 전체 차량들에  대해 Mandatory LC 처리	
		Evaluate_MLC(v);	 
	 		
		//  각 링크별로 처리  
		for (int current_link = 0; current_link < sizeof(l);i + current_link++) {			 				
			// Optioanl LC 처리
			Evlauate_OLC(v,l[current_link]);
			
			//CTM SIM 처리    
			CFsim(l[current_link]);

			
			// 링크별 결과 전송

		}
																	

		//전체 차량들에대해 셀이동 처리  
		Vehicle_Move(v);				
								
										
	}
}

void CFsim(link* l){
	double w = 15;  //wave speed
	
	int N = l.N;
	int maxN = l.maxN;
	float Y = l.Y;
	float maxY = l.maxY;
	float V = l.V;
	float Vf = l.Vf;

	float L = l.CellLength;
	
	int NoCell = l.NoCell;
	int NoLane = l.NoLane;
	
	float Lmin = Vf/3.6 * dt;
	
	int cell;
	int lane;
	

	
	for (cell = 0; cell < NoCell; cell++) {
		for (lane = 0; lane < NoLane; lane++) {
			if (cell == 0) {
				Y[cell][lane] = 1;
			} else if {
				Y[cell][lane] = min( min( Lmin/L[cell] * N[cell][lane], maxY[cell][lane]), 
						min( maxY[cell][lane+1], w * dt / L * (maxN[cell][lane] - N[cell][lane] ));
			}
		N[cell][lane] += Y[cell][lane];
		}
	}
	//움직일 vehicle들의 moveForward를 1로 바꿈
}


void Evaluate_MLC(vehicle* v){

	// --------------------------------------------------------------------------------------------------
	// Mandatory Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정 
	// --------------------------------------------------------------------------------------------------
	for(int vehID = 0; vehID < sizeof(v); vehID++){
		vehcle veh=v[vehID]; // 차량데이터 베이스에서 가지고 오기 
				
		int TargetLaneLeft=veh.targetLane1[veh.currentLinkOrder];  // 타겟 레인 하한 가지고 오기 
		int TargetLaneRight=veh.targetLane2[veh.currentLinkOrder];  // 타겟 레인 상한 가지고 오기 
				
		if(veh.currentLane < TargetLaneLeft){veh.lanechange=1;}     // 오른쪽으로 차로 변경이 필요 
		elseif(veh.currentLane < TargetLaneLeft) {veh.lanechange=-1;}  // 왼쪽으로 차로 변경이 필요
		else (veh.lanechange=0;) 
	}				
	// --------------------------------------------------------------------------------------------------
}

void Evaluate_OLC(vehicle* v, link* l){

	// --------------------------------------------------------------------------------------------------
	// Optional Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정 
	// --------------------------------------------------------------------------------------------------
				
	// --------------------------------------------------------------------------------------------------
	
	// LC Matrix 설정 
		
	LC_Left 
	LC_Right
}
										
void Vehicle_Move(vehicle* v){
	
	for(int vehID = 0; vehID < sizeof(v); vehID++){
		vehcle veh=v[vehID]; // 차량데이터 베이스에서  가지고 오기 	
		// --------------------------------------------------------------------------------------------------
		// 차로변경이 있는 경우 차량의 현재 Cell과 링크를 업데이트 한다.
		//--------------------------------------------------------------------------------------------------
		if (veh.Lanechange = +1) {}	// Move vehicle to left lane 
		if (veh.Lanechange = -1) {} 	// Move vehicle to Right lane 	
		if (veh.moveforward = 1) {}     // Move vehicle to frent cell
	
		// --------------------------------------------------------------------------------------------------
		

		// --------------------------------------------------------------------------------------------------
		// 차량이 다음셀로 전진하는 경우 차량의 현재 Cell과 링크를 업데이트 한다.
		//--------------------------------------------------------------------------------------------------
		if (veh.moveForward==1){
			if((veh.currentCell == l[veh.currentLink].NoCell)) { // 현재 셀이 링크의 마지막셀인 경우 
				veh.currentLinkOrder++; // Path의 현재 링크 순서를 1 증가 
				veh.currentLink = veh.path[currentLinkOrder];
				veh.currentCell= 0;  // Cell position을 링크 시작점으로 
			} else {
				veh.currentCell++;   //마지막 셀이 아니면, 다음 셀로 차량을 옮긴다.
			}
		}
	}
	// --------------------------------------------------------------------------------------------------

}								
						

int main(void) {
	// simulation info
	int total_simulation_time = 100, period = 5;

	// set data size
	int numLink = 16, gpuBlockSize = 4, linkSize = sizeof(link),
			numBytes = numLaneCell * linkSize, gpuGridSize = numLaneCell
					/ gpuBlockSize;
	int numNode = 16, nodeSize = sizeof(node), numNodesBytes = numNode
			* nodeSize;

	int numVehicle = 16, vehicleSize = sizeof(vehicle), numVehicleBytes =
			numVehicle * vehicleSize;

	int numVehicleQueue = 16, numVehicleQueueBytes = numVehicleQueue
			* vehicleSize;

	int numResult = total_simulation_time / period, numResultBytes = numResult
			* lcSize;

	// allocate memory
	link *cpuLinkArray, *gpuLCArray;
	node *cpuNodeArray, *gpuNodeArray;
	vehicle *cpuVehicleArray, *gpuVehicleArray;
	link *cpuResultArray, *gpuResultArray;

	cpuLinkArray = (link*) malloc(numBytes);
	cpuNodeArray = (node *) malloc(numNodesBytes);
	cpuVehicleArray = (vehicle *) malloc(numVehicleBytes);
	cpuResultArray = (link*) malloc(
			numBytes * total_simulation_time / period);

	// input initial data
	printf("%d\n", numBytes);
	printf("%d\n", numNodesBytes);
	printf("%d\n", numVehicleBytes);
	// copy host memory to device memory
	cudaMalloc((void**) &gpuLinkArray, numBytes);
	cudaMalloc((void**) &gpuNodeArray, numNodesBytes);
	cudaMalloc((void**) &gpuVehicleArray, numVehicleBytes);

	// launch kernel
	simulationStep<<<gpuGridSize, gpuBlockSize>>>(numResult, gpuLCArray,
			gpuNodeArray, gpuVehicleArray);

	// retrieve the results
	cudaMemcpy(cpuResultArray, gpuResultArray, numResultBytes,
			cudaMemcpyDeviceToHost);
	// 필요없음
	// cudaMemcpy(cpuLCArray, gpuLCArray, numBytes, cudaMemcpyDeviceToHost);
	// cudaMemcpy(cpuNodeArray, gpuNodeArray, numNodesBytes, cudaMemcpyDeviceToHost);
	// cudaMemcpy(cpuVehicleArray, gpuNodeArray, numVehicleBytes, cudaMemcpyDeviceToHost);

	// record the simulation result
	printf("simulation results:\n");
	for (int i = 0; i < numResult; ++i) {
		//printf("point.a: %f, point.b: %f\n",cpuPointArray[i].a,cpuPointArray[i].b);
	}

	printf("end");

	// deallocate memory
	free(cpuLinkArray);
	free(cpuNodeArray);
	free(cpuVehicleArray);
	free(cpuResultArray);

	cudaFree(gpuLinkArray);
	cudaFree(gpuNodeArray);
	cudaFree(gpuVehicleArray);
	cudaFree(gpuResultArray);

	return 0;
}
