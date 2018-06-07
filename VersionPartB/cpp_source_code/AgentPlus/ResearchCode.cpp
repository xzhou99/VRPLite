
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

#include "AgentPlus.h"
#include "CSVParser.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern FILE* g_pFileDebugLog;
//#define _MAX_NUMBER_OF_CELLS 101

#define _RESOLUTION_PER_STEP 10

#define _MAX_NUMBER_OF_CELLS  100

#define _MAX_NUMBER_OF_POWER_DISTRICTS  2

//1552 16383

#define _MAX_MOVE_STEPS 4
#define _MAX_TRAINS 1

#define _MAX_TIME_STEPS 360

float g_SegmentGrade[_MAX_NUMBER_OF_CELLS] = { 0 };
float g_SegmentCurve[_MAX_NUMBER_OF_CELLS] = { 0 };
int	  g_SegmentPowerSupplyDistrict[_MAX_NUMBER_OF_CELLS] = { 0 };

int   g_TrainStationFlag[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS];


int   g_TrainStationTimeWindowBeginning[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS];
int   g_TrainStationTimeWindowEnding[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS];

int   g_TrainDepartureTimeStart[_MAX_TRAINS] = { 0 };
int   g_TrainDepartureTimeEnd[_MAX_TRAINS] = { 0 };
int   g_TrainArrivalTimeStart[_MAX_TRAINS] = { 0 };
int   g_TrainArrivalTimeEnd[_MAX_TRAINS] = { 0 };
int   g_TrainOriginStation[_MAX_TRAINS] = { -1 };
int   g_TrainDestinationStation[_MAX_TRAINS] = { -1 };


int g_NumberOfTrains = 4;
int g_safety_time_headway = 180;
int*****   g_TrainArcFlag = NULL; //k, i,j',u,v
float g_TrainArcCost[_MAX_NUMBER_OF_CELLS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS][_MAX_MOVE_STEPS]; // i,j',u,v

float*** g_LabelCost = NULL;

int***   g_PredecessorNode = NULL;
int***   g_PredecessorSpeed = NULL;
int***   g_PredecessorTime = NULL;
float***   g_PredecessorPower = NULL;

int**   g_BasicSegmentUsageFlag = NULL;
float**   g_BasicSegmentUsagePrice = NULL;

int  g_PowerSupplySegmentUsageValue[_MAX_NUMBER_OF_POWER_DISTRICTS][_MAX_TIME_STEPS] = { 0 };
float  g_PowerSupplySegmentUsagePrice[_MAX_NUMBER_OF_POWER_DISTRICTS][_MAX_TIME_STEPS] = { 0 };

class JVElement
{
public:
	int k;
	int i;
	int u;
	int j_p;  //relative distance 
	int v;
	int delta_t;
	float cost;  // total cost
	float per_time_cost; 
	char type;

};

std::vector <JVElement> g_JVVector[_MAX_TRAINS][_MAX_NUMBER_OF_CELLS][_MAX_MOVE_STEPS];

int g_get_integer_by_multiplier(int number, int multiplier_value)
{
	number = int(number / multiplier_value+0.5)* multiplier_value;
	return number;

}
void g_ProgramStop()
{

	cout << "Agent+ Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};

void g_STVComputing()
{


	//cout << " __g_TrainArcFlag  " << endl;

	// g_TrainArcFlag = Allocate5DDynamicArray <int>(_MAX_TRAINS, _MAX_NUMBER_OF_CELLS, _MAX_MOVE_STEPS, _MAX_MOVE_STEPS, _MAX_MOVE_STEPS);

	cout << " __g_LabelCost  " << endl;

	g_LabelCost = Allocate3DDynamicArray<float>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, _MAX_MOVE_STEPS);

	cout << " __g_PredecessorNode  " << endl;

	g_PredecessorNode = Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, _MAX_MOVE_STEPS);

	cout << " __g_PredecessorSpeed  " << endl;

	g_PredecessorSpeed = Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, _MAX_MOVE_STEPS);

	cout << " g_PredecessorPower  " << endl;

	g_PredecessorPower = Allocate3DDynamicArray<float>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, _MAX_MOVE_STEPS);

	cout << " g_PredecessorTime  " << endl;

	g_PredecessorTime = Allocate3DDynamicArray<int>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, _MAX_MOVE_STEPS);

	


	cout << " memory allocation completes  " << endl;

	g_BasicSegmentUsageFlag = AllocateDynamicArray<int>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, 0);

	g_BasicSegmentUsagePrice = AllocateDynamicArray<float>(_MAX_NUMBER_OF_CELLS, _MAX_TIME_STEPS, 0);

	//read grade data

	CCSVParser parser;

	//if (parser.OpenCSVFile("input_grade.csv", true))
	//{
	//	while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
	//	{
	//		int start_node_index = 0;
	//		int end_node_index = 0;
	//		parser.GetValueByFieldName("start_node_index", start_node_index);
	//		parser.GetValueByFieldName("end_node_index", end_node_index);

	//		float grade = 0;
	//		parser.GetValueByFieldName("grade", grade, false);

	//		for (int i = min(_MAX_NUMBER_OF_CELLS - 1, start_node_index); i < min(_MAX_NUMBER_OF_CELLS - 1, end_node_index); i++)
	//		{
	//			g_SegmentGrade[i] = grade;
	//		}

	//	}

	//	for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
	//	{
	//		if (fabs(g_SegmentGrade[i]) >0.01)
	//		{
	//			fprintf(g_pFileDebugLog, "node index: %d, grade = %.2f\n", i, g_SegmentGrade[i]);
	//		}
	//	}

	//}

	parser.CloseCSVFile();


	if (parser.OpenCSVFile("input_train_station.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int node_index = 0;
			parser.GetValueByFieldName("node_index", node_index);

			node_index = g_get_integer_by_multiplier((node_index / _RESOLUTION_PER_STEP), _RESOLUTION_PER_STEP);

			int train_index = 0;
			parser.GetValueByFieldName("train_index", train_index);

			if (train_index == 0)				//default stopping pattern for all trains 
			{

				if (node_index < _MAX_NUMBER_OF_CELLS)
				{
					for (int k = 1; k <= g_NumberOfTrains; k++)
					{
						train_index = k;
					g_TrainStationFlag[train_index][node_index] = 1;

					// add transition arc: waiting: 

					JVElement element;
					element.k = train_index;
					element.i = node_index;
					element.u = 0;
					element.v = 0;
					element.j_p = 0;
					element.delta_t = 1;

					if (node_index == g_TrainOriginStation[k])  // origin 
						element.cost = 50;
					else
						element.cost = 0;

					element.type = 'w';

					g_JVVector[train_index][element.i][element.u].push_back(element);

					// add transition arc: starting 

					element.i = node_index;
					element.u = 0;
					element.v = _MAX_MOVE_STEPS - 1;
					element.delta_t = 420 / _RESOLUTION_PER_STEP;  // 420 seconds 
					element.j_p = element.delta_t * (_MAX_MOVE_STEPS - 1 + 0) *0.5;
					element.per_time_cost = 200;
					element.cost = element.per_time_cost * element.delta_t;

					element.type = 's';
					g_JVVector[train_index][element.i][element.u].push_back(element);

					// add transition arc: ending 
					element.delta_t = 200 / _RESOLUTION_PER_STEP;  // 150 seconds
					element.i = max(0, node_index - (element.delta_t * (_MAX_MOVE_STEPS - 1 + 0) *0.5)); 
					element.u = _MAX_MOVE_STEPS - 1; 
					element.v =0;
					element.j_p = element.delta_t * (_MAX_MOVE_STEPS - 1 + 0) *0.5;

					element.per_time_cost = 0;
					element.cost = element.per_time_cost * element.delta_t;

					element.type = 'e';
					g_JVVector[train_index][element.i][element.u].push_back(element);

					}
				}
			}

		}


	}

	parser.CloseCSVFile();

	for (int k = 1; k <= g_NumberOfTrains; k++)
	for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
	{
		if (g_TrainStationFlag[k][i] == 0)  // no stop node
		{
			JVElement element;

			element.k = k;
			element.i = i;
			element.u = _MAX_MOVE_STEPS - 1;
			element.v = _MAX_MOVE_STEPS - 1;
			element.j_p = _MAX_MOVE_STEPS - 1;
			element.delta_t = 1;
			element.per_time_cost = 50;
			element.cost = element.per_time_cost * element.delta_t;

			element.type = 'c';

			g_JVVector[k][element.i][element.u].push_back(element);

		}
	}


	//read curve data


	if (parser.OpenCSVFile("curve.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int start_node_index = 0;
			int end_node_index = 0;
			parser.GetValueByFieldName("start_node_index", start_node_index);
			parser.GetValueByFieldName("end_node_index", end_node_index);

			float curve = 0;
			parser.GetValueByFieldName("curve", curve);

			for (int i = min(_MAX_NUMBER_OF_CELLS - 1, start_node_index); i < min(_MAX_NUMBER_OF_CELLS - 1, end_node_index); i++)
			{
				g_SegmentCurve[i] = curve;
			}
		}

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{
			if (fabs(g_SegmentCurve[i]) >0.01)
			{
				fprintf(g_pFileDebugLog, "node index: %d, curve = %.2f\n", i, g_SegmentCurve[i]);
			}
		}


	}

	parser.CloseCSVFile();

	
	if (parser.OpenCSVFile("input_train.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int start_node_index = 0;
			int end_node_index = 0;
			int agent_id = 0;


			parser.GetValueByFieldName("agent_id", agent_id);


			int agent_no = agent_id;

			if (agent_no > g_NumberOfTrains || agent_no >= _MAX_TRAINS)
				break;
			
			int origin_node_id = 0;
			int destination_node_id = 0;
			parser.GetValueByFieldName("origin_node_id", origin_node_id);
			parser.GetValueByFieldName("destination_node_id", destination_node_id);

			g_TrainOriginStation[agent_no] = g_get_integer_by_multiplier((origin_node_id / _RESOLUTION_PER_STEP), _RESOLUTION_PER_STEP);

			g_TrainDestinationStation[agent_no] = g_get_integer_by_multiplier(min(_MAX_NUMBER_OF_CELLS - 1, int(destination_node_id / _RESOLUTION_PER_STEP)), _RESOLUTION_PER_STEP);


			fprintf(g_pFileDebugLog, "--Basic data: train = %d; start_station= %d,end_station = %d--\n",
				agent_no, g_TrainOriginStation[agent_no], g_TrainDestinationStation[agent_no]);


			g_TrainStationFlag[agent_no][g_TrainOriginStation[agent_no]] = 1;
			g_TrainStationFlag[agent_no][g_TrainDestinationStation[agent_no]] = 1;
			

			int departure_time_start = 0;
			int departure_time_end = 0;
			int arrival_time = 0;
			parser.GetValueByFieldName("departure_time_start", departure_time_start);
			parser.GetValueByFieldName("departure_time_end", departure_time_end);
			parser.GetValueByFieldName("arrival_time", arrival_time);

			g_TrainDepartureTimeStart[agent_no] = min(_MAX_TIME_STEPS - 1, departure_time_start / _RESOLUTION_PER_STEP);
			g_TrainDepartureTimeEnd[agent_no] = min(_MAX_TIME_STEPS - 1, departure_time_end / _RESOLUTION_PER_STEP);
			g_TrainArrivalTimeEnd[agent_no] = min(_MAX_TIME_STEPS - 1, arrival_time / _RESOLUTION_PER_STEP);

		}


	}

	parser.CloseCSVFile();


	// transition arc

	FILE * st = NULL;

	st = fopen("GAMS_input.txt", "w");

	if (st != NULL)
	{

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{

			for (int j_p = 1; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					g_TrainArcFlag[0][i][j_p][u][v] = 0;  // init
					g_TrainArcCost[i][j_p][u][v] = 0;


				}

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				{
					int v = j_p * 2 - u;
					if (v >= 0 && v < _MAX_MOVE_STEPS && (i + j_p < _MAX_NUMBER_OF_CELLS))
					{  // v feasible
						g_TrainArcFlag[0][i][j_p][u][v] = 1;
						g_TrainArcCost[i][j_p][u][v] = max(0, (v - u) * (u + v) / 2.0);

					}
				}

			}


		}

		// output speed_arc
		fprintf(st, "parameter speed_arcs(i,j,u,v) /\n");

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{
			for (int j_p = 0; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					if (g_TrainArcFlag[0][i][j_p][u][v] == 1)
						fprintf(st, "%d . %d . %d . %d  1\n", i + 1, i + 1 + j_p, u, v);

				}


			}


		}

		fprintf(st, "/;\n");

		// output arc cost
		fprintf(st, "parameter cost(i,j,u,v) / \n");

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		{
			for (int j_p = 0; j_p < _MAX_MOVE_STEPS; j_p++)
			{

				for (int u = 0; u < _MAX_MOVE_STEPS; u++)
				for (int v = 0; v < _MAX_MOVE_STEPS; v++)
				{
					if (g_TrainArcFlag[0][i][j_p][u][v] == 1)  // only for feasible arcs 
						fprintf(st, "%d . %d . %d . %d %4.2f\n", i + 1, i + 1 + j_p, u, v, g_TrainArcCost[i][j_p][u][v]);

				}
			}

		}
		fprintf(st, "/;\n");


		fclose(st);
	}



	for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
	{

		for (int j_p = 1; j_p < _MAX_MOVE_STEPS; j_p++)
		{

			for (int u = 0; u < _MAX_MOVE_STEPS; u++)
			for (int v = 0; v < _MAX_MOVE_STEPS; v++)
			{
				g_TrainArcCost[i][j_p][u][v] = 0;
			}
		}
	}



	// initialize price 
	for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
	for (int t = 0; t < _MAX_TIME_STEPS; t++)
	{
		g_BasicSegmentUsagePrice[i][t] = 0;
	}


	for (int p = 0; p < _MAX_NUMBER_OF_POWER_DISTRICTS; p++)
	for (int t = 0; t < _MAX_TIME_STEPS; t++)
	{
		g_PowerSupplySegmentUsagePrice[p][t] = 0;
	}

	// setup transition arcs

	// 


	for (int iteration = 0; iteration < 10; iteration++)
	{
		float step_size = max(0.05, 1.0 / (iteration + 1));

		cout << "************ performing LR algorithm at iteration " << iteration << "************" << endl;

		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		for (int t = 0; t < _MAX_TIME_STEPS; t++)
		{
			g_BasicSegmentUsageFlag[i][t] = 0;  // reset the usage of basic segments
		}

		for (int p = 0; p < _MAX_NUMBER_OF_POWER_DISTRICTS; p++)
		for (int t = 0; t < _MAX_TIME_STEPS; t++)
		{
			g_PowerSupplySegmentUsageValue[p][t] = 0;  // reset the usage of basic segments
		}



		for (int k = 1; k <= g_NumberOfTrains; k++)
		{
			cout << " performing dynamic programming  for train  " << k << endl;

			if (g_TrainOriginStation[k] >= 0)  // feasible train with data

			{
				g_DynamicProgramming_transition_arc(k);
			}
		}

		// subgradient 
		for (int i = 0; i < _MAX_NUMBER_OF_CELLS; i++)
		for (int t = 0; t < _MAX_TIME_STEPS; t++)
		{
			g_BasicSegmentUsagePrice[i][t] += step_size*(g_BasicSegmentUsageFlag[i][t] - 1);  // reset the usage of basic segments

			if (g_BasicSegmentUsagePrice[i][t] < 0)
				g_BasicSegmentUsagePrice[i][t] = 0;
			else
			{
				//	fprintf(g_pFileDebugLog, "\nbasic segment price [%d][%d]= %f", i, t, g_BasicSegmentUsagePrice[i][t]);
			}
		}


		for (int p = 0; p < _MAX_NUMBER_OF_POWER_DISTRICTS; p++)
		for (int t = 0; t < _MAX_TIME_STEPS; t++)
		{
			g_PowerSupplySegmentUsagePrice[p][t] += step_size*(g_PowerSupplySegmentUsageValue[p][t] - 500); // 500 total power per district

			if (g_PowerSupplySegmentUsagePrice[p][t] < 0)
				g_PowerSupplySegmentUsagePrice[p][t] = 0;
			else
			{
				fprintf(g_pFileDebugLog, "\npower supply district price [%d][%d]= %f", p, t, g_PowerSupplySegmentUsagePrice[p][t]);

			}

		}




	}  // iteration

}



void g_DynamicProgramming_transition_arc(int k)  // train index k  // 
{
	// initialization 
	int optimal_path_node_seqence[_MAX_TIME_STEPS] = { -1 };
	int optimal_path_speed_seqence[_MAX_TIME_STEPS] = { -1 };
	int optimal_path_time_seqence[_MAX_TIME_STEPS] = { -1 };


	for (int i = g_TrainOriginStation[k]; i <= g_TrainDestinationStation[k]; i++)
	for (int t = g_TrainDepartureTimeStart[k]; t < g_TrainArrivalTimeEnd[k]; t++)
	for (int u = 0; u < _MAX_MOVE_STEPS; u++)
	{
		g_LabelCost[i][t][u] = _MAX_LABEL_COST;

		g_PredecessorNode[i][t][u] = -1;
		g_PredecessorSpeed[i][t][u] = -1;
		g_PredecessorTime[i][t][u] = -1;
		g_PredecessorPower[i][t][u] = 0;

	}

	int t0 = g_TrainDepartureTimeStart[k];
	int station_0 = g_TrainOriginStation[k];  //
	int station_destination = min(_MAX_NUMBER_OF_CELLS - 1, g_TrainDestinationStation[k]); //e.g. 499

	g_LabelCost[station_0][t0][0] = 0;

	int trip_time = 0;
	int trip_remaining_time = 0;


	int final_arrival_time = min(g_TrainArrivalTimeEnd[k], _MAX_TIME_STEPS);
	fprintf(g_pFileDebugLog, "--Basic data: train = %d; start_time= %d, arrival_time =%d;start_station= %d,end_station = %d--\n", 
		k, g_TrainDepartureTimeStart[k], final_arrival_time, station_0, station_destination);

	for (int t = g_TrainDepartureTimeStart[k]; t <final_arrival_time; t++)
	{
		if (t ==185)
			cout << "computing time = " << t  << " min" << endl;

		trip_time = t - g_TrainDepartureTimeStart[k];
		trip_remaining_time = g_TrainArrivalTimeEnd[k] - t;

		int min_station_index = max(g_TrainDestinationStation[k] - (trip_remaining_time - 1)*_MAX_MOVE_STEPS, g_TrainOriginStation[k]);
		int max_station_index = min(g_TrainOriginStation[k] + (trip_time + 1)*_MAX_MOVE_STEPS, g_TrainDestinationStation[k]);
		for (int i = min_station_index; i < max_station_index; i++)  // i is the basic segment
		{
			int starting_station_flag = 1;
			if (g_TrainStationFlag[k][i] == 1)  /*current station is stop statsion, allow waiting arcs at stations */
				starting_station_flag = 0;


			for (int u = 0; u < _MAX_MOVE_STEPS; u++)  // we should have a feasible ranage for driving speed at any location 
			{

				int element_size = g_JVVector[k][i][u].size();

				for (int e_i = 0; e_i < element_size; e_i++)  // e_i is an index for dynamic vector at i and u 
				{

					JVElement element = g_JVVector[k][i][u][e_i];

					// skip 

					if (i == g_TrainOriginStation[k] && t > g_TrainDepartureTimeEnd[k])  // no more starting or waiting arcs after departure ending time
						continue;


					int j_p = element.j_p;
					int v = element.v;
					int s = min(_MAX_TIME_STEPS-1, t + element.delta_t);

					if ((g_TrainStationFlag[k][i + j_p] == 1 && v == 0)  /*next station is stop statsion, only allow approaching speed v= 0 */
						|| (g_TrainStationFlag[k][i + j_p] == 0 && (v >= 1 && v < _MAX_MOVE_STEPS && (i + j_p < _MAX_NUMBER_OF_CELLS))))  /*non-stop statsion, only allow v >= 1 */
					{
						if (g_LabelCost[i][t][u] < _MAX_LABEL_COST) // feasible label at starting node
						{

							float LRPrice = 0;

							if (j_p >= 1)  // moving
							{
								LRPrice += g_BasicSegmentUsagePrice[i][t];
								LRPrice += g_PowerSupplySegmentUsagePrice[i][t];
								
								//if (LRPrice[i][t] >= 0.01)
								//	TRACE("positive resource price at i=%, t= %d: %f \n ", i, t, LRPrice[i][t]);

							}

							float NewLabelCost = g_LabelCost[i][t][u] + element.cost + LRPrice;

							if (i + j_p < _MAX_NUMBER_OF_CELLS)  // feasibility checking 
							{
								if (NewLabelCost < g_LabelCost[i + j_p][s][v])
								{
									g_LabelCost[i + j_p][s][v] = NewLabelCost;  // update the cost

									g_PredecessorNode[i + j_p][s][v] = i;   // update predecessor of node
									g_PredecessorSpeed[i + j_p][s][v] = u;  // update predecessor of speed index
									g_PredecessorTime[i + j_p][s][v] = t;  // update predecessor of time index
									g_PredecessorPower[i + j_p][s][v] = element.per_time_cost; // record the per time cost 

									// fprintf(g_pFileDebugLog, " updating:k=%d,%c,t=%d,s=%d,i=%d,j_p=%d,i+j_p=%d,u=%d,v=%d,LC=%.0f\n", k, element.type, t, s, i, j_p, i + j_p, u, v, NewLabelCost);

									if (i + j_p == g_TrainDestinationStation[k])
									{

										fprintf(g_pFileDebugLog, " destination: train = %d,  s=%d i = %d, j_p = %d, i+j_p = %d, u = %d, v = %d,label cost = %f\n", k, s, i, j_p, i + j_p, u, v, NewLabelCost);
									}
								}
							}

					}
				}
			}

			}

		}
	}	

	// backtrace trajectories

	 final_arrival_time = 0;
	float least_cost = _MAX_LABEL_COST; // initial value
	int backtraced_node_index = g_TrainDestinationStation[k]; // update predecessor of node
	int backtraced_time_index = min(g_TrainArrivalTimeEnd[k], _MAX_TIME_STEPS - 1);
	int u = 0;
	float power_per_time = 0;
	fprintf(g_pFileDebugLog, "--backtrace for train %d at station %d = %d\n", k, backtraced_node_index);

	for (int t = g_TrainDepartureTimeStart[k]; t < min(g_TrainArrivalTimeEnd[k], _MAX_TIME_STEPS) - 1; t++)
	{
		int speed = 0;
		// fprintf(g_pFileDebugLog, "train %d, time %d with cost = %f\n", k, t, g_LabelCost[backtraced_node_index][t][speed]);


			if (g_LabelCost[backtraced_node_index][t][speed] < least_cost)
			{
				least_cost = g_LabelCost[backtraced_node_index][t][speed]; // update cost
				final_arrival_time = t;                                // update arrival time
				u = g_PredecessorSpeed[backtraced_node_index][t][speed];   //update approaching speed to the final station
				backtraced_time_index = g_PredecessorTime[backtraced_node_index][t][speed];   //update approaching speed to the final station

				power_per_time = g_PredecessorPower[backtraced_node_index][t][speed];

			}
	}

	fprintf(g_pFileDebugLog, "--final arrival time for train %d = %d with cost = %f, approaching speed = %d\n", k, final_arrival_time, least_cost, u);
	cout << " train " << k << " final arrival time , " << final_arrival_time << endl;
	u = 0;

	int backtrace_sequence_index = 0;
	optimal_path_node_seqence[backtrace_sequence_index] = backtraced_node_index;
	optimal_path_speed_seqence[backtrace_sequence_index] = u;
	optimal_path_time_seqence[backtrace_sequence_index] = final_arrival_time;

	int time2 = final_arrival_time;

	int node, speed, time;
	while (optimal_path_node_seqence[backtrace_sequence_index] >=0)
	{

	    node = optimal_path_node_seqence[backtrace_sequence_index];
		speed = optimal_path_speed_seqence[backtrace_sequence_index];
		time = optimal_path_time_seqence[backtrace_sequence_index];


		fprintf(g_pFileDebugLog, "train %d, time %d at noded = %d\n", k, time, node);

		for (int tau = time; tau < min(_MAX_TIME_STEPS, time2); tau++)
		{
			int p = g_SegmentPowerSupplyDistrict[node];  // get power supply district no.

			g_PowerSupplySegmentUsageValue[p][tau] += power_per_time;

		}

		time2 = time;

		// mark safety time headway constraint
		for (int tau = time; tau < min(_MAX_TIME_STEPS, time + g_safety_time_headway / _RESOLUTION_PER_STEP - 1); tau++)
		{
			g_BasicSegmentUsageFlag[min(_MAX_NUMBER_OF_CELLS, max(0, node))][min(_MAX_TIME_STEPS, tau)] += 1;
		}



		backtrace_sequence_index++;
		optimal_path_node_seqence[backtrace_sequence_index] = g_PredecessorNode[node][time][speed];
		optimal_path_speed_seqence[backtrace_sequence_index] = g_PredecessorSpeed[node][time][speed];
		optimal_path_time_seqence[backtrace_sequence_index] = g_PredecessorTime[node][time][speed];
		power_per_time = g_PredecessorPower[node][time][speed];
		
	}

	bool bDebugFlag = false;

	if (bDebugFlag)
		fprintf(g_pFileDebugLog, "-- train %d trajectory-- node,time,speed \n", k);
	// print out train trajectories


			////if (bDebugFlag)
			////	fprintf(g_pFileDebugLog, "%d, %d,%d, use: %d \n", optimal_path_node_seqence[t], t, optimal_path_speed_seqence[t], g_BasicSegmentUsageFlag[optimal_path_node_seqence[t]][t]);
}



void g_DynamicProgramming_fined_grid(int k)  // train index k
{
	// initialization 

	int optimal_path_node_seqence[_MAX_TIME_STEPS] = { -1 };
	int optimal_path_speed_seqence[_MAX_TIME_STEPS] = { -1 };


	for (int i = g_TrainOriginStation[k]; i <= g_TrainDestinationStation[k]; i++)
	for (int t = g_TrainDepartureTimeStart[k]; t < g_TrainArrivalTimeEnd[k]; t++)
	for (int u = 0; u < _MAX_MOVE_STEPS; u++)
	{
		g_LabelCost[i][t][u] = _MAX_LABEL_COST;

		g_PredecessorNode[i][t][u] = -1;
		g_PredecessorSpeed[i][t][u] = -1;

	}

	int t0 = g_TrainDepartureTimeStart[k];
	int station_0 = g_TrainOriginStation[k];  //
	int station_destination = _MAX_NUMBER_OF_CELLS - 1; //e.g. 499

	g_LabelCost[station_0][t0][0] = 0;

	int trip_time = 0;
	int trip_remaining_time = 0;
	for (int t = g_TrainDepartureTimeStart[k]; t < min(g_TrainArrivalTimeEnd[k], _MAX_TIME_STEPS); t++)
	{
		//if (t % 60 == 0)
		//	cout << "computing time = " << t / 60 << " min" << endl;

		trip_time = t - g_TrainDepartureTimeStart[k];
		trip_remaining_time = g_TrainArrivalTimeEnd[k] - t;

		int min_station_index = max(g_TrainDestinationStation[k] - (trip_remaining_time - 1)*_MAX_MOVE_STEPS, g_TrainOriginStation[k]);
		int max_station_index = min(g_TrainOriginStation[k] + (trip_time + 1)*_MAX_MOVE_STEPS, g_TrainDestinationStation[k]);
		for (int	i = min_station_index; i < max_station_index; i++)  // i is the basic segment
		{
			int starting_station_flag = 1;
			if (g_TrainStationFlag[k][i] == 1)  /*current station is stop statsion, allow waiting arcs at stations */
				starting_station_flag = 0;

			for (int j_p = starting_station_flag; j_p < _MAX_MOVE_STEPS; j_p++)  // we should start from a feasible rannge for driving distance 
			{
				for (int u = 0; u < _MAX_MOVE_STEPS; u++)  // we should have a feasible ranage for driving speed at any location 
				{
					if (g_TrainStationFlag[k][i] == 1 && u != 0)  /*current station is stop statsion, only allow starting speed u= 0 */
					{
						continue;
					}
					int v = j_p * 2 - u;  // v is uniquely determined by u and j_p

					if ((g_TrainStationFlag[k][i + j_p] == 1 && v == 0)  /*next station is stop statsion, only allow approaching speed v= 0 */
						|| (g_TrainStationFlag[k][i + j_p] == 0 && (v >= 1 && v < _MAX_MOVE_STEPS && (i + j_p < _MAX_NUMBER_OF_CELLS))))  /*non-stop statsion, only allow v >= 1 */
					{
						if (g_LabelCost[i][t][u] < _MAX_LABEL_COST) // feasible label at starting node
						{
							//if (g_TrainArcLRPrice[i][t][j_p] >= 0.01)
							//	TRACE("positive resource price at i=%, t= %d: %f \n ", i, t, g_TrainArcLRPrice[i][t][j_p]);

							float LRPrice = 0;
							float NewLabelCost = g_LabelCost[i][t][u] + g_TrainArcCost[i][j_p][u][v] + LRPrice;

							if (i + j_p < _MAX_NUMBER_OF_CELLS && t + 1 < _MAX_TIME_STEPS)  // feasibility checking 
							{
								if (NewLabelCost < g_LabelCost[i + j_p][t + 1][v])
								{
									g_LabelCost[i + j_p][t + 1][v] = NewLabelCost;  // update the cost

									g_PredecessorNode[i + j_p][t + 1][v] = i;   // update predecessor of node
									g_PredecessorSpeed[i + j_p][t + 1][v] = u;  // update predecessor of speed index

									if (i + j_p == g_TrainDestinationStation[k])
									{
										TRACE("i = %d, j_p = %d, i+j_p = %d, u = %d, v = %d,label cost = %f\n", i, j_p, i + j_p, u, v, NewLabelCost);
									}
								}
							}
						}
					}
				}

			}

		}
	}

	// backtrace trajectories

	int final_arrival_time = 0;
	float least_cost = _MAX_LABEL_COST; // initial value
	int backtraced_node_index = g_TrainDestinationStation[k]; // update predecessor of node
	int u = 0;


	for (int t = g_TrainDepartureTimeStart[k]; t < min(g_TrainArrivalTimeEnd[k], _MAX_TIME_STEPS) - 1; t++)
	{
		for (int speed = 0; speed < 1; speed++)
		{
			if (g_LabelCost[backtraced_node_index][t][speed] < least_cost)
			{
				least_cost = g_LabelCost[backtraced_node_index][t][speed]; // update cost
				final_arrival_time = t;                                // update arrival time
				u = g_PredecessorSpeed[backtraced_node_index][t][speed];   //update approaching speed to the final station

			}
		}
	}

	fprintf(g_pFileDebugLog, "--final arrival time for train %d = %d with cost = %f, approaching speed = %d\n", k, final_arrival_time, least_cost, u);
	cout << " train " << k << " final arrival time , " << final_arrival_time << endl;
	u = 0;

	optimal_path_node_seqence[final_arrival_time] = backtraced_node_index;
	optimal_path_speed_seqence[final_arrival_time] = u;

	for (int t = final_arrival_time; t >= g_TrainDepartureTimeStart[k]; t--)
	{
		int old_backtraced_node_index = backtraced_node_index;
		backtraced_node_index = g_PredecessorNode[old_backtraced_node_index][t][u];
		u = g_PredecessorSpeed[old_backtraced_node_index][t][u];
		optimal_path_node_seqence[t - 1] = backtraced_node_index;
		optimal_path_speed_seqence[t - 1] = u;

	}

	bool bDebugFlag = false;

	if (bDebugFlag)
		fprintf(g_pFileDebugLog, "-- train %d trajectory-- node,time,speed \n", k);
	// print out train trajectories
	for (int t = g_TrainDepartureTimeStart[k]; t <= final_arrival_time; t++)
	{
		//		if (optimal_path_node_seqence[t + 1] = g_TrainOriginStation[k])
		{
			for (int tau = max(0, t - g_safety_time_headway + 1); tau < min(_MAX_TIME_STEPS, t + g_safety_time_headway - 1); tau++)
			{
				g_BasicSegmentUsageFlag[optimal_path_node_seqence[t]][tau] += 1;
			}
			//if (bDebugFlag)
			//	fprintf(g_pFileDebugLog, "%d, %d,%d, use: %d \n", optimal_path_node_seqence[t], t, optimal_path_speed_seqence[t], g_BasicSegmentUsageFlag[optimal_path_node_seqence[t]][t]);
		}
	}
}


