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
double maxYconst = 1800;
double Vfconst = 50;
double CellLengthconst = 100;


typedef struct {
	
	int NoLane;  	 //INPUT argument 
	int NoSection;      //INPUT argument 
	int VehMax;      //INPUT argument 
	
	int N[NoSection][NoLane];  		// 2D Array [NoCell	,NoLane]
	int MaxN[NoSection][NoLane];		// 2D Array [NoCell	,NoLane]
	int LC_Left[NoSection][NoLane]; 		// 2D Array [NoCell	,NoLane]
	int LC_Right[NoSection][NoLane]; 		// 2D Array [NoCell	,NoLane]
	double V[NoSection][NoLane];		// 2D Array [NoCell	,NoLane]
	double Y[NoSection+1][NoLane];		// 2D Array [NoCell+1	,NoLane]
	double MaxY[NoSection+1][NoLane];		// 2D Array [NoCell	,NoLane]
	double SectionLength[NoSection];
	double Vf;// Free flow speed 	
	
	// Vehicle Move 관련 
	int veh[NoSection+2][NoLane][VehMax];		// vehID per each cell (include buffer cell)
	int vehMLC[NoSection+2][NoLane][VehMax];    	// 1이면 오른쪽으로차로변경,-1이면 왼쪽으로 변경
	int vehOLC[NoSection+2][NoLane][VehMax]; 		// 1이면 오른쪽으로차로변경,-1이면 왼쪽으로 변경
	int vehMoveForward[NoSection+2][NoLane][VehMax];   //1이면 다음셀로 전진,0이면 현재셀에 머무르기
	
	// Vehicle 속성중에서 MLC 관련 속성 넣기 
	int targetLane1[NoSection+2][NoLane][VehMax]; 	// minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
	int targetLane2[NoSection+2][NoLane][VehMax]; 	// max Target Lane  Ex) 3   타겟 레인 가안 값 설정 	
	
	// 시그널 넣기 
	int greenTime[NoLane];	 			// 1이면 Green signal, 0이면 Red signal
	
	// 글로벌 메모리를 잘 쓰자 -- 글로벌 메모리에서 링크 MLC 결정을 위한 차량보

	int NextConnectionSection;
	int PreviousConnectionSection;
		
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
	int currentSection;      // 현재 cell, 링크 시작점이 0,  
	int currentLink;      // 현재 링크 ID 
	int currentLinkOrder; // path[]에서 현재 링크 순서  
	// int distanceToNode;

	int type;
	
//	int MandatoryLC;
//	int moveForward;  // 1이면 시뮬레이션 시 다음셀로 차량을 이동할 필요  
	
//	int freeflowspeed;
//	int minSpacing;
//	int reactionTime;
//	int driving_moment;
	
//	int lanechange;  // 1이면 오른쪽으로 차로변경, -1이면 왼쪽으로 변경이 필요 
	
	int path[20];  // Array of Link IDs EX) [15, 17, 19,...,0,0] 
	int NoLinksinPath;  //size of array path path[NoLinksinPath]  path 의 데이터 크기 
	int targetLane1[20]; // minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
	int targetLane2[20]; // max Target Lane  Ex) 3   타겟 레인 상한 값 설정 	
} vehicle;


typedef struct {
	int NoConnection;   			// 전체 커넥션 수 INPUT argument 
	int VehMax;                     	// 커넥션의 차량 수  
	int fromLinkID[NoConnection];  		// 커넥션의 fromLink ID
	int toLinkID[NoConnection];           	// 커넥션의 toLink ID
	int veh[NoConnection][VehMax];        	// 커넥션상의 차량 ID  

	int greenTime[NoConnection];	         //각 커넥션의 한 시뮬레이션 스텝 중의 신호 1: Green, 0: Red

} cennection_cell;				// 커넥션은 글로벌 메모리상에서 상주하여, 링크의 아웃풋 데이터들을 다음 링크와 공유할수 있도록 해준다. 


__global__ void simulationStep(int loop_limit, link *l, node *n,
		vehicle *v) {
	int tid = threadIdx.x;
	int i = blockIdx.x * blockDim.x + tid;

	// simulation time
	for (int current = 0; current < loop_limit; current++) {
		
		
		// read vehicle from connectCell (i'th vehicle in global memory )
		//NextCoonectionCell에서 signal 정보 읽어오기 => link
		
		
		//Previous ConnectionCell에서 Input 읽어오기 => link
		
		
		
		// 각 링크l[i]별로 Mandatory LC 처리	
		Evaluate_MLC(l[i]);	 // 
	 		
					 				
		// 각 링크l[i]별로 Optioanl LC 처리
		Evalauate_OLC(l[i]);
			
		//각 링크l[i]별로 CTM SIM 처리    
		CFsim(l[i]);

			
		// 링크별 결과 전송

		
																
		//전체 차량들에대해 셀이동 처리  
		Vehicle_Move(l[i]);	
		
		// write vehicle in connectCell
			
		
		//synchronize
		__syncthreads();								
	}
}

__device__ CFsim(link* l){
	double w = 15;  //wave speed
	
	double L = l.SectionLength;
	
	int NoSection = l.NoSection;
	int NoLane = l.NoLane;
	
	double Lmin = l.Vf/3.6 * dt;
	
	
	for (int section = 0; section < l.NoSection; section++) {
		for (int lane = 0; lane < l.NoLane; lane++) {
			l.Y[section][lane] = min( min( Lmin/L[section] * l.N[section][lane], l.maxY[section][lane]), 
					min( l.maxY[section][lane+1], w * dt / L * (l.maxN[section][lane] - l.N[section][lane] ));
		// moveforward flag update
					      
		}
	}
	
}


__device__ Evaluate_MLC(link *l){

	// --------------------------------------------------------------------------------------------------
	// Mandatory Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정 
	// --------------------------------------------------------------------------------------------------
	for(int section = 0; section < l.NoSection; section++){
		for(int lane = 0; lane < l.NoLane; lane++){
			for (int i =0 ; i < 20; i++){
				vehicle veh=l.veh[section][lane][i]; // 차량데이터 가지고 오기 
				
				int TargetLaneLeft=veh.targetLane1[veh.currentLinkOrder];  // 타겟 레인 하한 가지고 오기 
				int TargetLaneRight=veh.targetLane2[veh.currentLinkOrder];  // 타겟 레인 상한 가지고 오기 
				
				if(veh.currentLane < TargetLaneLeft){
					veh.lanechange=1;
					// l.LC_Left[veh.currentCell][veh.currentLane]=1  
				}     // 오른쪽으로 차로 변경이 필요 
				else if(veh.currentLane > TargetLaneRight) {
					veh.lanechange=-1;
					// l[veh.currentLink].LC_Righft[veh.currentCell][veh.currentLane]=1;}  // 왼쪽으로 차로 변경이 필요
				else (veh.lanechange=0;) 
			}
		}
	}				
	// --------------------------------------------------------------------------------------------------
}

__device__ Evaluate_OLC(link* l){

	// --------------------------------------------------------------------------------------------------
	// Optional Lane Change 대상 차량 선정 및 차량 데이터베이스에 차로변경 플래그(veh.lanechange) 설정 
	// --------------------------------------------------------------------------------------------------
		
	for (int section = 0; section<l.NoCell; section++){
		for(int lane = 1; lane <l.NoLane; lane++){
			if (lane >0){
				l.LC_Left[section][lane] += (l.V[section][lane-1] - l.V[section][lane])/l.Vf;	// cell별 optional LC 확률 계산 (왼쪽)
			}
			//if (lane <l.NoLane) {
			//	l.LC_Right[section][lane] += (l.V[section][lane+1] - l.V[section][lane])/l.Vf;	// cell별 optional LC 확률 계산 (오른쪽)
			//}
			
			for (int i = 0; i <20; i++) {
				vehicle veh = l.veh[section][lane][i];  // 차량 데이터 가지고 오기
				
				srand(time(NULL));
				float prob = (rand() % 10) / 10.;	// 차량별 OLC 확률 생성
				
				if (prob <= l.LC_Left[section][lane]) {
					veh.lanechange = -1;		// 왼쪽으로 차로 변경 필요
				}
			
		}
	}
	
	
	
}
										
__device__ Vehicle_Move(link* l){
	
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
			
			//글로벌 메모리의 Vehicle 정보를 업데이트 
			
			if((veh.currentCell == l[veh.currentLink].NoCell)) { 
				//connection Cell로 바꾸기
				
				// 현재 셀이 링크의 마지막셀인 경우 
				veh.currentLinkOrder++; // Path의 현재 링크 순서를 1 증가 
				veh.currentLink = veh.path[currentLinkOrder];
				veh.currentCell= 0;  // Cell position을 링크 시작점으로 \
				
				
			} else {
				// N Update  
				
				
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
			numBytes = numLink * linkSize, gpuGridSize = numLink
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
	link *cpuLinkArray, *gpuLinkArray;
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
