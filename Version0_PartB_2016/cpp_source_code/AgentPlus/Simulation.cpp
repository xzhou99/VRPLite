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

//#define _MAX_NUMBER_OF_NODES 10
//#define _MAX_NUMBER_OF_LINKS 40
#define _MAX_NUMBER_OF_SIMU_TIME_INTERVALS 200


#define _MAX_NUMBER_OF_VEHICLES 10
extern FILE* g_pFileDebugLog;

#define _MAX_NUMBER_OF_PATH_LINKS 100

// input parameter
int Veh_link_matrix[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PATH_LINKS];
float LinkKjam[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_SIMU_TIME_INTERVALS];

//  input parameters 
extern int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
extern float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];  // read from the network file
extern float g_link_jam_density[_MAX_NUMBER_OF_LINKS];
extern float g_link_link_length[_MAX_NUMBER_OF_LINKS];


extern int g_number_of_links;
extern int g_number_of_vehicles;

extern int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
extern int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES];
extern int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];

// variables  that can be changed in real time
int LinkOutFlowCapacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_SIMU_TIME_INTERVALS];
int LinkInFlowCapacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_SIMU_TIME_INTERVALS];

int Veh_current_link_index[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int Veh_TA[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS];  // arrival time
int Veh_TD[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_LINKS];  // departure time

int LinkCumulativeArrival[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_SIMU_TIME_INTERVALS];  // for kinemative wave 
int LinkCumulativeDeparture[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_SIMU_TIME_INTERVALS];

int LinkCumulativeArrivalCount[_MAX_NUMBER_OF_LINKS];
int LinkCumulativeDepartureCount[_MAX_NUMBER_OF_LINKS];

// link id.of vehicle’s ith link along the its path : no link = -1 : dynamic destination choice
// arrival time of vehicle v’s current paths to the  ith link : entrance time
// Veh_TD[v][i] departure time of vehicle v’s current paths from ith link : exit time

void g_SimulationInitialization(int SimulationStartTime, int SimulationEndTime)
{

		for (int l = 0; l < g_number_of_links; l++)
		{

			LinkCumulativeArrivalCount[l] = 0;
			LinkCumulativeDepartureCount[l] = 0;

			for (int t = SimulationStartTime; t < SimulationEndTime; t++)
			{
			LinkOutFlowCapacity[l][t] = g_link_capacity_per_time_interval[l];
			LinkInFlowCapacity[l][t] = 99999;  // point queue

			LinkCumulativeArrival[l][t] = 0;
			LinkCumulativeDeparture[l][t] = 0;
			}
		}


	// calculate shortest path for each vehicle
	// calculate Veh_link_matrix

	//Step 1: prepare input :
	//Vehicle path generation : prepare Veh_link_matrix[v][i] from shortest path or path routing algorithm, initial departure time Veh_TA[v][i = 0]

	for (int v = 0; v < g_number_of_vehicles; v++) //2ndrd loop for each vehicle in the network 
	{

		float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)  // we use shortest path to _MAX_NUMBER_OF_TIME_INTERVALS, which could be less than # of simulation time intervals
			{
				arc_cost[link][t] = g_link_free_flow_travel_time[link]
					+ g_external_link_time_dependent_toll[link][t];
			}
		}
		float path_travel_time = 0;

		float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_label_correcting(
			arc_cost,
			g_vehicle_origin_node[v],
			g_vehicle_departure_time_beginning[v],
			g_vehicle_departure_time_ending[v],
			g_vehicle_destination_node[v],
			g_vehicle_arrival_time_beginning[v],
			g_vehicle_arrival_time_ending[v],
			g_vehicle_path_number_of_nodes[v],
			g_vehicle_path_node_sequence[v],
			g_vehicle_path_link_sequence[v],
			g_vehicle_path_time_sequence[v],
			0,
			path_travel_time);

		//end for loop for each time t

		for (int l = 0; l < _MAX_NUMBER_OF_PATH_LINKS; l++)
		{
			if (l < g_vehicle_path_number_of_nodes[v])
				Veh_link_matrix[v][l] = g_vehicle_path_link_sequence[v][l];
			else
				Veh_link_matrix[v][l] = -100; // invalid default data

		}

		// set the current link index for each vehicle
		Veh_current_link_index[v] = 0;

		// initialization 
		Veh_TA[v][0] = g_vehicle_departure_time_beginning[v];


		int link_no = Veh_link_matrix[v][0];
		Veh_TD[v][0] = Veh_TA[v][0] + g_link_free_flow_travel_time[link_no];

		LinkCumulativeArrivalCount[link_no] += 1;



	}
}
void g_TrafficSimulation(int SimulationStartTime, int SimulationEndTime, int b_spatial_capacity_flag = 0)
{

	//Determine link capacity
		
	if (SimulationEndTime >= _MAX_NUMBER_OF_SIMU_TIME_INTERVALS)
		SimulationEndTime = _MAX_NUMBER_OF_SIMU_TIME_INTERVALS - 1;


		g_SimulationInitialization(SimulationStartTime, SimulationEndTime);
	
		//For vehicles on the link already, we need to determine the position of vehicle and calculate Veh_TD[v][i = 0]
		//Set LinkCumulativeArrival[l][t = 0] and LinkCumulativeDeparture[l][t] for the vehicles already on the road
	
		// Step 2: perform simulation
	for (int t = SimulationStartTime; t <SimulationEndTime; t++)  //1st loop for each simulation time 
	{
		// step 2.2. determine link capacity associated with bottleneck 

		if (b_spatial_capacity_flag)
		{
			for(int l = 0; l < g_number_of_links; l++) //2nd loop for each link in the network 
			{
				int LinkNumberOfVehicles = LinkCumulativeArrivalCount[l] - LinkCumulativeDepartureCount[l];
				LinkInFlowCapacity[l][t] = g_link_jam_density[l] * g_link_link_length[l] - LinkNumberOfVehicles;
			}

		}


			for(int v = 0; v< g_number_of_vehicles; v++) //2ndrd loop for each vehicle in the network 
		{
		//	If needed, perform shortest path : 
				
			// if t == departure time of vehicle, or if real time information is available, we only need to replace the element in the Veh_link_matrix[v][i] arrary
			int i = Veh_current_link_index[v]; // step 3.1: find the link sequence no. of vehicle v’ path
			int current_link_no = Veh_link_matrix[v][i]; // step 3.2: find current link id 
			// If(vehicle v is on link i’s exit queue)  i.e.Veh_TD[v][i] == t

				if (Veh_TD[v][i] == t)
			{  //step 3.3: tentative departure time = current time: ready to move 
					int next_link_no = Veh_link_matrix[v][i + 1]; // next link sequence no of vehicle 
				
					if (next_link_no >= 0)
					{

							if(LinkOutFlowCapacity[current_link_no][t] >= 1 && LinkInFlowCapacity[next_link_no][t] >= 1) // out and in capacity available 
						{  //step 3.4. check capacity 
							// step 3.5 : move 
								LinkOutFlowCapacity[current_link_no][t] -= 1; //reduce capacity on current link (for both point queue and spatial queue)

								if (b_spatial_capacity_flag)
								{
									LinkInFlowCapacity[next_link_no][t] -= 1; //reduce capacity on next link 
								}
								
								
									Veh_current_link_index[v] += 1;  // move to next link

							//if the vehicle completes its trip: out of loop
								
								Veh_TA[v][i + 1] = t;  // arrives at the next link
								Veh_TD[v][i + 1] = t + g_link_free_flow_travel_time[next_link_no]; // consider freeflow travel time for the "reserved or predicted departure time"
						
								LinkCumulativeDepartureCount[current_link_no] += 1;  // move out from the current link
								LinkCumulativeArrivalCount[next_link_no] += 1;  // move in to next link
							
						}
						else
						{//step 3.6: no move  wait for one more time interval 
							Veh_TD[v][i] = t + 1; // wait for one more time interval 
						}
					}
					else  // arrive at destination at time t: TD == t, no next link
					{   
						LinkCumulativeDepartureCount[current_link_no] += 1;  // move out from the current link
						// do nothing when you do not have a valid link number
					}
				}
		}//end for loop for each vehicle 

			//statistics collection 

			for (int l = 0; l < g_number_of_links; l++) //2nd loop for each link in the network 
			{
				LinkCumulativeArrival[l][t] = LinkCumulativeArrivalCount[l];
				LinkCumulativeDeparture[l][t] = LinkCumulativeDepartureCount[l];

				// from cumulative counts, we can calculate density and flow rates,
				// the speed is back calculated from the arrival and departure time of vehicles TA and TD
			}

	} //end for loop for each time t
	//

}