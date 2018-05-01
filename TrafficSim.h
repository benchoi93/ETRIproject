#ifndef TRAFFICSIM_H_
#define TRAFFICSIM_H_

#define NUM_LANE 4
#define NUM_SECTION 4
#define VEH_MAX 20

#define MAX(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })

#define MIN(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

typedef struct {
    int id;

    // current position
    int currentLane;
    int currentCell;      // 현재 cell, 링크 시작점이 0,
    int currentLink;      // 현재 링크 ID
    int currentLinkOrder; // path[]에서 현재 링크 순서
    // int distanceToNode;

    int type;

    int MandatoryLC;
    int moveForward;  // 1이면 시뮬레이션 시 다음셀로 차량을 이동할 필요

//    int freeflowspeed;
//    int minSpacing;
//    int reactionTime;
//    int driving_moment;

    int lanechange;  // 1이면 오른쪽으로 차로변경, -1이면 왼쪽으로 변경이 필요

    int path[20];  // Array of Link IDs EX) [15, 17, 19,...,0,0]
    int NoLinksinPath;  //size of array path path[NoLinksinPath]  path 의 데이터 크기
    int targetLane1[20]; // minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
    int targetLane2[20]; // max Target Lane  Ex) 3   타겟 레인 가안 값 설정
} vehicle;

typedef struct {
    int linkid;

    int N[NUM_SECTION][NUM_LANE];          // 2D Array [NUM_SECTION    ,NUM_LANE]
    int MaxN[NUM_SECTION][NUM_LANE];        // 2D Array [NUM_SECTION    ,NUM_LANE]
    int LC_Left[NUM_SECTION][NUM_LANE];         // 2D Array [NUM_SECTION    ,NUM_LANE]
    int LC_Right[NUM_SECTION][NUM_LANE];         // 2D Array [NUM_SECTION    ,NUM_LANE]
    double V[NUM_SECTION][NUM_LANE];        // 2D Array [NUM_SECTION    ,NUM_LANE]
    double Y[NUM_SECTION+1][NUM_LANE];        // 2D Array [NUM_SECTION+1    ,NUM_LANE]
    double MaxY[NUM_SECTION+1][NUM_LANE];        // 2D Array [NUM_SECTION    ,NUM_LANE]
    double CellLength[NUM_SECTION];
    double Vf;// Free flow speed
    double sectionLength[NUM_SECTION];

    // 재고
    vehicle veh[NUM_SECTION+2][NUM_LANE][VEH_MAX];        // vehID per each cell (include buffer cell)
    int vehMLC[NUM_SECTION+2][NUM_LANE][VEH_MAX];
    int vehOLC[NUM_SECTION+2][NUM_LANE][VEH_MAX];
    int vehMoveForward[NUM_SECTION+2][NUM_LANE][VEH_MAX];


    // Vehicle 속성중에서 MLC 관련 속성 넣기
    int targetLane1[NUM_SECTION+2][NUM_LANE][VEH_MAX]; 	// minimum Target Lane  EX) 2  타겟 레인의 하한값 설정
    int targetLane2[NUM_SECTION+2][NUM_LANE][VEH_MAX]; 	// max Target Lane  Ex) 3   타겟 레인 가안 값 설정

    // 시그널 넣기
    int greenTime[NUM_LANE];	 			// 1이면 Green signal, 0이면 Red signal

    // 글로벌 메모리를 잘 쓰자 -- 글로벌 메모리에서 링크 MLC 결정을 위한 차량보

    int NextConnectionCell;
    int PreviousConnectionCell;
} link;

typedef struct {

    int nodeID;
    int type; 	// intersection, source, sink
    int speedlimit;
    int intersection_type;
    //connection_info
} node;

typedef struct {
    int NoConnection;   			// 전체 커넥션 수 INPUT argument
    int VehMax;                     	// 커넥션의 차량 수
    int fromLinkID;  		// 커넥션의 fromLink ID
    int toLinkID;           	// 커넥션의 toLink ID
    int veh[VEH_MAX];        	// 커넥션상의 차량 ID

    int greenTime;	         //각 커넥션의 한 시뮬레이션 스텝 중의 신호 1: Green, 0: Red

} connection_cell;				// 커넥션은 글로벌 메모리상에서 상주하여, 링크의 아웃풋 데이터들을 다음 링크와 공유할수 있도록 해준다.

void SimulationStep(link l[], int numberOfLinks, int loop_limit);
void MLCsim(link* l);
void OLCsim(link* l);
void Evaluate_MLC(link *l);
void Evaluate_OLC(link *l);
void Vehicle_Move(link *l);
void CFsim(link *l);

void setup_roadlink();
double get_time_ms();



#endif /* TRAFFICSIM_H_ */
