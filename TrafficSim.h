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
	
    int linkid;
    
//	int NoLane;  	 //INPUT argument 
//	int NoSection;      //INPUT argument 
//	int VehMax;      //INPUT argument 
	
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
	int targetLane2[NoSection+2][NoLane][VehMax]; 	// max Target Lane  Ex) 3   타겟 레인 상한 값 설정 	
	
	// Signal
	int greenTime[NoLane];	 			// 1이면 Green signal, 0이면 Red signal

	int NextConnectionSection;
	int PreviousConnectionSection;
		
} link;


typedef struct {
	
	int nodeID;
	int type; 	// intersection, source, sink
	int speedlimit;
	int intersection_type;

} node;

typedef struct {

   	int type;

    int currentLane;
	int currentSection;      // 현재 cell, 링크 시작점이 0,  
	int currentLink;      // 현재 링크 ID 
	int currentLinkOrder; // path[]에서 현재 링크 순서  
	
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
