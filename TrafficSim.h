#ifndef TRAFFICSIM_H_
#define TRAFFICSIM_H_

#define NUM_LANE    		4	/// Number of lane is fixed to 4
#define NUM_SECTION 		4	/// Number of section is fixed to 4
#define MAX_VEC     		20	/// Number of vehicle in one cell is limited to 20
#define MAX_LOOP    		100	/// Number of simulation loop(simulation time step) is limited to 100
#define MAX_LEG			5	/// Number of intersection leg is limited to 3
#define SECONDS_PER_STEP   	5	/// Time interval of one simulation loop is fixed as 5 seconds
#define UPDATE_INTERVAL 	60 	/// Time interval between vehicle update is fixed as 60 simulation time step
#define MAX_CYCLE_DURATION	100	/// Max duration of traffic signal cycle is fixed as 100 simulation time step

#define MIN(a,b) (((a)<(b)) ? (a):(b))
#define MAX(a,b) (((a)>(b)) ? (a):(b))


typedef struct {
	int vehID;		/// ID of vehicle: 1~45
	int vehType;	/// Type of vehicle

	int path[120];	/// Array of Link IDs in the order in which vechicle passes
	int lenPath;	/// Length of path

	int minTargetLane[120];	/// Minimum Target Lane
	int maxTargetLane[120];	/// Maximum Target Lane

	int departTime; 	/// depart time

	int initLink;		/// initial Link ID

	
} vehicle;


typedef struct {
	int linkID;		/// ID of link: 1~18
					/// 11~14: source, 15~18: sink

	double ffSpeed;								/// Free flow speed: 16(m/s)
	double lenSection[NUM_SECTION+2];			/// Length of section: 100(m)
	int maxNumVeh[NUM_SECTION+2][NUM_LANE];   	/// Maximum possible number of vehicles in one cell
	double maxNumCF[NUM_SECTION+2][NUM_LANE];	/// Maximum possible number of CF
	
	int numVehArr[NUM_SECTION+2][NUM_LANE];  	  			/// Number of vehicles
	int vehIDArr[NUM_SECTION+2][NUM_LANE][MAX_VEC];			/// ID of vehicles
	int currLinkOrderArr[NUM_SECTION+2][NUM_LANE][MAX_VEC];	/// Current link order in vehicle path
	int nextLinkIDArr[NUM_SECTION+2][NUM_LANE][MAX_VEC];	/// Next link ID of vehicle
	int minTargetLaneArr[NUM_SECTION+2][NUM_LANE][MAX_VEC]; /// Minimum target lane in current link
	int maxTargetLaneArr[NUM_SECTION+2][NUM_LANE][MAX_VEC]; /// Maximum target lane in current link

	int speed[NUM_SECTION+2][NUM_LANE];		/// Average speed of vehicles in one cell

	int numMLCL[NUM_SECTION+2][NUM_LANE];  	/// Number of vehicles that performs MLCL
	int numMLCR[NUM_SECTION+2][NUM_LANE]; 	/// Number of vehicles that performs MLCR
	double numCF[NUM_SECTION+2][NUM_LANE];	/// Number of vehicles that performs CF
	
	int vehMLC[NUM_SECTION+2][NUM_LANE][MAX_VEC];   /// MLC flag of vehicles /// MLCL: -1, MLCR: 1
	int vehOLC[NUM_SECTION+2][NUM_LANE][MAX_VEC]; 	/// OLC flag of vehicles /// OLCL: -1, OLCR: 1
	int vehCF[NUM_SECTION+2][NUM_LANE][MAX_VEC];   	/// CF flag of vehicles /// CF: 1, notCF: 0

	int tempIDArr[NUM_LANE][MAX_LEG]; 	/// 2D array to temporary store ID information to or from the connection cell
	int tempNumArr[NUM_LANE][MAX_LEG];	/// 2D array to temporary store number information to or from the connection cell

} link;


typedef struct {
	int ccID;	/// ID of connection cell

	int prevLinkID;			 		/// ID of previous link, equal to ID of connection cell
	int nextLinkID[NUM_LANE][MAX_LEG];	/// ID of next link
	int nextLane[NUM_LANE][MAX_LEG];		/// Next lane of next link
	
	int cycleDuration;			/// duration of one traffic cycle
	
	int trafficSignal[NUM_LANE][MAX_CYCLE_DURATION];	/// Traffic signal information
						/// Red light: 0, Green light: 1

	int numVehArr[NUM_LANE][MAX_LEG];		/// 2D array to store number of vehicles

	double numCFArr[NUM_LANE][MAX_LEG];	/// 2D array to store number of CF

	int currLinkOrderArr[NUM_LANE][MAX_VEC]; /// 2D array to store current link order
	int nextLinkIDArr[NUM_LANE][MAX_VEC];	/// 2D array to store ID of next link of vehicles
	int vehIDArr[NUM_LANE][MAX_VEC];			/// 2D array to store ID of vehicles
	
} connection_cell;


void Setup_Veh(vehicle*, int);
void Setup_Link(vehicle*, int, link*, int);
void Setup_ConnectionCell(connection_cell*, int);

void Evaluate_MLC(link*);
void Evaluate_OLC(link *l);
	int Evaluate_Prob(double);
	void Select_Veh(link*, int, int, int, int);
void LCSim(link*);
	void MoveLC(int*, int, int*, int, int);

void Evaluate_CF(link*);
void CFsim(link*);
	void MoveCF(int*, int, int*, int, int);

void Update_tempArr(link*);
	int Find_Index(int*, int, int);
void Relay_numVeh(link*, link*, int, connection_cell*, int, int);
void Update_numCF(link*, link*, int, connection_cell*, int, int, int);
void Evaluate_Eff_numCF(link*);
void Update_vehCF(link*);
void Update_nextLink(vehicle*, link*, link*, int, connection_cell*, int);

void Reset_ConnectionCell(connection_cell*);
void Reset_Link(link*);

void SimulationStep(vehicle*, int, link l[], int, connection_cell cc[], int, int);

double get_time_ms();
void PrintAll(link l[], int);

#endif /* TRAFFICSIM_H_ */
