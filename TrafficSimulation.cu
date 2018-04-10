#include <stdlib.h>
#include <stdio.h>
#say hello

typedef struct {
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
	int currentSection;
	int currentLink;
	int distanceToNode;

	int type;
	int freeflowspeed;
	int minSpacing;
	int reactionTime;

	int driving_moment;
	int lane_change;
	int path[10];
} vehicle;

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

__global__ void simulationStep(int loop_limit, lane_cell *lc, node *n,
		vehicle *v) {
	int tid = threadIdx.x;
	int i = blockIdx.x * blockDim.x + tid;

	// simulation time
		for (int current = 0; current < loop_limit; current++) {
			for (int current_lane = 0; current_lane < sizeof(lc);
					i + current_lane++) {
				// insert vehicle from vehicle queue

				// update n <= n_agent + n
				lc[current_lane].numberOfVehicle ++;

				// update v <= v_agent + v
				lc[current_lane].speed = 60 * lc[current_lane].numberOfVehicle;

				// lane change

				// y_out -> y_in
				lc[current_lane].
				// record result

			}
	}
}

int main(void) {
	// simulation info
	int total_simulation_time = 100, period = 5;

	// set data size
	int numLaneCell = 16, gpuBlockSize = 4, lcSize = sizeof(lane_cell),
			numBytes = numLaneCell * lcSize, gpuGridSize = numLaneCell
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
	lane_cell *cpuLCArray, *gpuLCArray;
	node *cpuNodeArray, *gpuNodeArray;
	vehicle *cpuVehicleArray, *gpuVehicleArray;
	lane_cell *cpuResultArray, *gpuResultArray;

	cpuLCArray = (lane_cell*) malloc(numBytes);
	cpuNodeArray = (node *) malloc(numNodesBytes);
	cpuVehicleArray = (vehicle *) malloc(numVehicleBytes);
	cpuResultArray = (lane_cell*) malloc(
			numBytes * total_simulation_time / period);

	// input initial data
	printf("%d\n", numBytes);
	printf("%d\n", numNodesBytes);
	printf("%d\n", numVehicleBytes);
	// copy host memory to device memory
	cudaMalloc((void**) &gpuLCArray, numBytes);
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
	free(cpuLCArray);
	free(cpuNodeArray);
	free(cpuVehicleArray);
	free(cpuResultArray);

	cudaFree(gpuLCArray);
	cudaFree(gpuNodeArray);
	cudaFree(gpuVehicleArray);
	cudaFree(gpuResultArray);

	return 0;
}
