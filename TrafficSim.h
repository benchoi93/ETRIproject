#ifndef TRAFFICSIM_H_
#define TRAFFICSIM_H_

#define NUM_LANE    4
#define NUM_SECTION 4
#define MAX_VEC     20
#define dt          5

#define MIN(a,b) (((a)<(b)) ? (a):(b))
#define MAX(a,b) (((a)>(b)) ? (a):(b))

typedef struct {
   	int vehType;       // type of vehicle
	int vehID;         // ID of vehicle

	int path[MAX_VEC];      // Array of Link IDs in the order in which vechicle passes
	int lenPath;       // Length of path, total number of links in path

    int currLane;      // current Lane ID
	int currSection;   // current Section ID
	int currLink;      // current Link ID
	int currLinkOrder; // current Link order, index of array path 
	
	int minTargetLane[MAX_VEC]; // Minimum Target Lane
	int maxTargetLane[MAX_VEC]; // Maximum Target Lane

} vehicle;


typedef struct {
	int linkID;
			  	
	int maxNumVeh[NUM_SECTION+2][NUM_LANE];   	// 2D Array [NoCell	,NoLane]
	double maxNumCF[NUM_SECTION+2][NUM_LANE];	// 2D Array [NoCell	,NoLane]
	double ffSpeed;	// Free flow speed 	
	double lenSection[NUM_SECTION+2];
	
	int numVeh[NUM_SECTION+2][NUM_LANE];  	  	// 
	int vehIDArr[NUM_SECTION+2][NUM_LANE][MAX_VEC];	// vehID per each cell (include buffer cell)
	double speed[NUM_SECTION+2][NUM_LANE];		// 2D Arr [NoCell	,NoLane]

	int currLinkOrderArr[NUM_SECTION+2][NUM_LANE][MAX_VEC];
	int minTargetLaneArr[NUM_SECTION+2][NUM_LANE][MAX_VEC]; 	// minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
	int maxTargetLaneArr[NUM_SECTION+2][NUM_LANE][MAX_VEC]; 	// max Target Lane  Ex) 3   타겟 레인 상한 값 설정 	

	int vehMLC[NUM_SECTION+2][NUM_LANE][MAX_VEC];    	// 1이면 오른쪽으로차로변경, 0=> not right turn
	int vehOLC[NUM_SECTION+2][NUM_LANE][MAX_VEC]; 		// if 1, right; if 0, stay(or MLC); if -1, left
	int vehCF[NUM_SECTION+2][NUM_LANE][MAX_VEC];   //1이면 다음셀로 전진,0이면 현재셀에 머무르기

	int numVehLeft[NUM_SECTION+2][NUM_LANE];     	// 2D Array [NoCell	,NoLane]
	int numVehRight[NUM_SECTION+2][NUM_LANE]; 		// 2D Array [NoCell	,NoLane]
	int numVehCF[NUM_SECTION+1][NUM_LANE];		// 2D Array [NoCell+1	,NoLane]

} link;


/*
typedef struct {
	
	int nodeID;
	int type; 	// intersection, source, sink
	int speedlimit;
	int intersection_type;

} node;


typedef struct {
	int NoConnection;   			// 전체 커넥션 수 INPUT argument 
	int VehMax;                     	// 커넥션의 차량 수  
	int fromLinkID[NoConnection];  		// 커넥션의 fromLink ID
	int toLinkID[NoConnection];           	// 커넥션의 toLink ID
	int veh[NoConnection][VehMax];        	// 커넥션상의 차량 ID  

	int greenTime[NoConnection];	         //각 커넥션의 한 시뮬레이션 스텝 중의 신호 1: Green, 0: Red

} cennection_cell;				// 커넥션은 글로벌 메모리상에서 상주하여, 링크의 아웃풋 데이터들을 다음 링크와 공유할수 있도록 해준다. 
*/


void Setup_Veh(vehicle*, int);
void Setup_Link(link*, int, vehicle*, int);


void Evaluate_MLC(link*, int);
void Evaluate_OLC(link *l);
	int Evaluate_Prob(double);
	void Select_Veh(int*, int, int, int*, int*, int);
void LCSim(link*,vehicle*,int);

void Evaluate_CF(link*, vehicle*, int);
void CFsim(link*, vehicle*, int);


void SimulationStep(link l[], int, vehicle*, int, int);


void PrintAll(link*, vehicle*, int);
double get_time_ms();

#endif /* TRAFFICSIM_H_ */
