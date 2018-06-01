// AgentPlus.cpp : Defines the entry point for the console application.
/* Copyright (C) 2015 Xuesong Zhou - All Rights Reserved*/

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "AgentPlus.h"
#include "CSVParser.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// The one and only application object

CWinApp theApp;
using namespace std;

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;

FILE* g_pFileAgentPathLog = NULL;

// step 1: read network_trip data
//
std::map<int, int> g_internal_node_no_map;
std::map<int, int> g_external_node_id_map;

std::map<string, int> g_internal_agent_no_map;
std::map<int, string> g_external_passenger_id_map;
std::map<int, string> g_external_vehicle_id_map;


int g_passenger_carrying_state_vector[_MAX_NUMBER_OF_STATES][_MAX_NUMBER_OF_PASSENGERS];
float g_passenger_base_profit[_MAX_NUMBER_OF_PASSENGERS] = { 7 };
float g_passenger_request_travel_time_vector[_MAX_NUMBER_OF_PASSENGERS] = { 999 };
int g_accessibility_matrix[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_PASSENGERS];

int g_max_vehicle_capacity = 1;
int g_number_of_passengers = 0;

int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_link_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_state_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_vehicle_path_cost_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_id_for_pax_being_served[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_best_solution_vehicle_id_for_pax_being_served[_MAX_NUMBER_OF_PASSENGERS] = { 0 };

float** g_vehicle_origin_based_node_travel_time = NULL;
float** g_vehicle_destination_based_node_travel_time = NULL;

extern float g_UpperBoundGeneration(int LR_Iteration_no);

class CVRState  //class for vehicle scheduling states
{
public:
	int passenger_carrying_state[_MAX_NUMBER_OF_PASSENGERS];

	CVRState()
	{
		m_vehicle_capacity = 1;
		for (int p = 0; p < _MAX_NUMBER_OF_PASSENGERS; p++)
			passenger_carrying_state[p] = 0;
	
	}

	std::vector<int> m_outgoing_state_index_vector;
	std::vector<int> m_outgoing_state_change_service_code_vector;

	int m_vehicle_capacity;

	std::string generate_string_key()
	{
		std::string string_key;
		for (int p = 1; p <= g_number_of_passengers; p++)  // scan all passengers
		{

			stringstream s;

			s << "_";
			if (passenger_carrying_state[p] == 1)
			{
				s << p;
			}
			else
			{
				s << " ";
			}
		
			string converted(s.str());

			string_key += converted;


		}
	
		return string_key;  //e.g. _ _ _ or _1_2_3
	}
};

std::map<std::string, int> g_state_map;

int g_find_state_index(std::string string_key)
{
	if (g_state_map.find(string_key) != g_state_map.end())
	{
		return g_state_map[string_key];
	}
	else
	
		return -1;  // not found

}

std::vector<CVRState> g_VRStateVector;

void g_add_states(int parent_state_index, int number_of_passengers, int capacity)
{

	CVRState element = g_VRStateVector[parent_state_index];

	g_VRStateVector[parent_state_index].m_outgoing_state_index_vector.push_back(parent_state_index);  // link my own state index to the parent state
	g_VRStateVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(0);  // link no change state index to the parent state


	for (int p = 1; p <= number_of_passengers; p++)
	{
		if (element.passenger_carrying_state[p] == 0) // not carrying yet
		{ 
			// add pick up state 
			CVRState new_element;
			int pax_count = 0;
			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states
			{
				new_element.passenger_carrying_state[pp] = element.passenger_carrying_state[pp];

				if (element.passenger_carrying_state[pp] ==1)
					pax_count++;
			}


			new_element.m_vehicle_capacity = pax_count + 1;

			if (pax_count < capacity)
			{

				// test capacity 
				new_element.passenger_carrying_state[p] = 1;  // from 0 to 1

				std::string string_key = new_element.generate_string_key();
				int state_index = g_find_state_index(string_key);
				if (state_index == -1)
				{
					// add new state
					state_index = g_VRStateVector.size();
					g_VRStateVector.push_back(new_element);
					g_state_map[string_key] = state_index;
				}

				g_VRStateVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state
				g_VRStateVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(p);  // link new state index to the parent state
				
			}
			else
			{ // do  nothing
			}
		}
		else  // ==1 carried
		{
			// add delivery state 
			CVRState new_element;
			int pax_count = 0;

			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states
			{
				new_element.passenger_carrying_state[pp] = element.passenger_carrying_state[pp];

				if (element.passenger_carrying_state[pp] == 1)
					pax_count++;
			}
			new_element.passenger_carrying_state[p] = 0;  // from 1 to 0
			new_element.m_vehicle_capacity = pax_count - 1;

			std::string string_key = new_element.generate_string_key();
			int state_index = g_find_state_index(string_key);
			if (state_index == -1)
			{
				// add new state
				state_index = g_VRStateVector.size();
				g_VRStateVector.push_back(new_element);
				g_state_map[string_key] = state_index;
			}

			g_VRStateVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state
			g_VRStateVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back((-1)*p);  // link new state index to the parent state
		}

	}

	

}





class OutgoingState
{
public:
	std::vector<int> m_w2_vector;
};

OutgoingState g_outgoingStateSet[_MAX_NUMBER_OF_STATES];

int g_outbound_node_size[_MAX_NUMBER_OF_NODES] = { 0 };
int g_node_passenger_id[_MAX_NUMBER_OF_NODES] = { -1 };

int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_activity_node_flag[_MAX_NUMBER_OF_NODES] = { 0 };
int g_activity_node_ending_time[_MAX_NUMBER_OF_NODES] = { 99999 };
int g_activity_node_starting_time[_MAX_NUMBER_OF_NODES] = { 99999 };

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];


float** g_passenger_activity_node_multiplier;

int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
float g_link_free_flow_travel_time_float_value[_MAX_NUMBER_OF_LINKS];

float g_link_link_length[_MAX_NUMBER_OF_LINKS];
int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
float g_link_jam_density[_MAX_NUMBER_OF_LINKS];
int g_link_service_code[_MAX_NUMBER_OF_LINKS] = { 0 };


float g_link_speed[_MAX_NUMBER_OF_LINKS];
int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];
int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];


float g_VOIVTT_per_hour[_MAX_NUMBER_OF_VEHICLES];
float g_VOWT_per_hour[_MAX_NUMBER_OF_VEHICLES];




float g_link_resource_capacity[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];




int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 0 };
int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 120 };
int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];

int g_passenger_origin_node[_MAX_NUMBER_OF_PASSENGERS];  // traveling passengers
int g_passenger_departure_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_departure_time_ending[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_destination_node[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_PASSENGERS];
float g_passenger_origin_multiplier[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
float g_passenger_destination_multiplier[_MAX_NUMBER_OF_PASSENGERS] = { 0 };


float g_passenger_request_cancelation_cost[_MAX_NUMBER_OF_PASSENGERS] = { 0 };




int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };

int g_passenger_assigned_vehicle_id[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_path_node_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_passenger_path_link_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_passenger_path_time_sequence[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_passenger_path_number_of_nodes[_MAX_NUMBER_OF_PASSENGERS] = { 0 };

int g_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
int g_path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
int g_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
int g_path_number_of_nodes;
float g_path_travel_time = 0;


int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_physical_nodes = 0;

int g_number_of_time_intervals = 10;



int g_number_of_vehicles = 0;
int g_number_of_physical_vehicles = 0;


int g_number_of_toll_records = 0;

int g_number_of_LR_iterations = 1;
int g_minimum_subgradient_step_size = 0.01;

int g_shortest_path_debugging_flag = 1;
float g_waiting_time_ratio = 0.005;
float g_dummy_vehicle_cost_per_hour = 100;

bool g_no_changed_route_for_passengers_flag = false;
bool g_no_capacity_multiplier_flag = false;

float g_travel_time_budget = 100;
float g_idle_vehicle_benefit = -10;


void g_create_all_states(int number_of_passengers = 10, int capacity = 1)
{
	CVRState route_element; // 0000000000

	std::string string_key = route_element.generate_string_key();

	g_state_map[string_key] = 0;
	g_VRStateVector.push_back(route_element);

	int scan_state_index = 0;
	while (g_VRStateVector.size() < _MAX_NUMBER_OF_STATES && scan_state_index< g_VRStateVector.size() && scan_state_index< _MAX_NUMBER_OF_STATES)
	{
		g_add_states(scan_state_index, number_of_passengers, capacity);


		scan_state_index++;
	}

	// print out 
	for (int i = 0; i < g_VRStateVector.size(); i++)
	{
		std::string str = g_VRStateVector[i].generate_string_key();
	
		fprintf(g_pFileDebugLog, "state no. %d: %s; outgoing state list:", i, str.c_str());

		for (int w2 = 0; w2 < g_VRStateVector[i].m_outgoing_state_index_vector.size(); w2++)
		{
			fprintf(g_pFileDebugLog, "%d,", g_VRStateVector[i].m_outgoing_state_index_vector[w2]);
		}

		fprintf(g_pFileDebugLog, "\n");

	}

	fprintf(g_pFileDebugLog, "-----\n");



}
int g_add_new_node(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes+1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = passenger_id;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;

	g_number_of_nodes++;
	return g_number_of_nodes;
}

int g_add_new_link(int from_node_id, int to_node_id, int passenger_id = 0, int travel_time = 1, double link_length = 1, int number_of_lanes = 1, int mode_code = 0,
	int capacity_per_time_interval= 1, double speed = 60)
{
	int new_link_id = g_number_of_links;
	g_outbound_node_id[from_node_id][g_outbound_node_size[from_node_id]] = to_node_id;
	g_outbound_link_no[from_node_id][g_outbound_node_size[from_node_id]] = new_link_id;

	g_outbound_node_size[from_node_id]++;

	g_inbound_node_id[to_node_id][g_inbound_node_size[to_node_id]] = from_node_id;
	g_inbound_link_no[to_node_id][g_inbound_node_size[to_node_id]] = new_link_id;
	g_inbound_node_size[to_node_id]++;


	g_link_from_node_number[new_link_id] = from_node_id;
	g_link_to_node_number[new_link_id] = to_node_id;

	g_link_free_flow_travel_time[new_link_id] = max(1, travel_time);

	g_link_link_length[g_number_of_links] = link_length;
	g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
	g_link_mode_code[g_number_of_links] = mode_code;
	g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
	g_link_speed[g_number_of_links] = speed;
	g_link_service_code[g_number_of_links] = passenger_id;

	g_number_of_links++; 
	return g_number_of_links;
		
}



int g_get_link_no_based_on_from_node_to_node(int from_node, int to_node)
{
	if (from_node >= _MAX_NUMBER_OF_NODES)
		return -1;

	if (from_node == to_node)
		return -1;

	//scan outbound links from a upstream node 
	for (int i = 0; i < g_outbound_node_size[from_node]; i++)
	{
		if (g_outbound_node_id[from_node][i] == to_node)
			return g_outbound_link_no[from_node][i];
	}
		
	return -1; 

}


int g_node_status_array[_MAX_NUMBER_OF_NODES];


float g_node_label_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_node_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];
int g_time_predecessor[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_arc_travel_time[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };

float g_to_node_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_to_node_cost_used_for_upper_bound[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_vertex_waiting_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vertex_visit_count[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };




// label cost for each LC(i,t,p,c) 

float*** l_state_node_label_cost = NULL;

int*** l_state_node_predecessor = NULL;
int*** l_state_time_predecessor = NULL;
int*** l_state_carrying_predecessor = NULL;

float** g_node_to_node_shorest_travel_time = NULL;

void g_allocate_memory()
{
	int number_of_states = g_VRStateVector.size()+1;
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_time_intervals = g_number_of_time_intervals + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;

	cout << "number of states = " << g_VRStateVector.size() << endl; 

	l_state_node_label_cost = Allocate3DDynamicArray<float>(number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_node_predecessor = Allocate3DDynamicArray<int>( number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_time_predecessor = Allocate3DDynamicArray<int>(number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_carrying_predecessor = Allocate3DDynamicArray<int>( number_of_nodes, number_of_time_intervals, number_of_states);

	g_node_to_node_shorest_travel_time = AllocateDynamicArray<float>(number_of_nodes, number_of_nodes,999);

	g_passenger_activity_node_multiplier = AllocateDynamicArray<float>(number_of_nodes, number_of_time_intervals,0);

	g_vehicle_origin_based_node_travel_time = AllocateDynamicArray<float>(number_of_vehicles, number_of_nodes, 0);
	g_vehicle_destination_based_node_travel_time = AllocateDynamicArray<float>(number_of_vehicles, number_of_nodes, 0);

}

void g_free_memory()
{
	int number_of_states = g_VRStateVector.size();
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_time_intervals = g_number_of_time_intervals + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;

	Deallocate3DDynamicArray<float>(l_state_node_label_cost,  number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_node_predecessor,  number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_time_predecessor, number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_carrying_predecessor, number_of_nodes, number_of_time_intervals);
	
	DeallocateDynamicArray<float>(g_node_to_node_shorest_travel_time, number_of_nodes, number_of_nodes);
	DeallocateDynamicArray<float>(g_passenger_activity_node_multiplier, number_of_nodes, number_of_time_intervals);
	DeallocateDynamicArray<float>(g_vehicle_origin_based_node_travel_time, number_of_vehicles, number_of_nodes);
	DeallocateDynamicArray<float>(g_vehicle_destination_based_node_travel_time, number_of_vehicles, number_of_nodes);

	
}

// for non service link: one element: w2 = w1 for all possible stages
// for pick up link: one element: w2= w1 + the passenger of upstream node, p

// infeasible, p already in w1 or w1 is full at capacity

// for delivery link: one element: w2= w1 - the passenger of downstream node
//infeasible: if p is inc



float g_optimal_time_dependenet_dynamic_programming(
	int vehicle_id,
	float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS],
	float to_node_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS],
	float vertex_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS],
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
	int &path_number_of_nodes,
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], 
	int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	float path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	int vehicle_capacity,
	float &travel_time_return_value)
	// time-dependent label correcting algorithm with double queue implementation
{

	if (arrival_time_ending > g_number_of_time_intervals)
	{
		TRACE("error");
	}
	
	float total_cost = _MAX_LABEL_COST;
	if (g_outbound_node_size[origin_node] == 0)
	{
		return _MAX_LABEL_COST;
	}

	// step 1: Initialization for all nodes
	for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes
	{
		for (int t = 0; t < g_number_of_time_intervals; t++)
		{

			for (int w = 0; w < g_VRStateVector.size(); w++)
			{
				l_state_node_label_cost[i][t][w] = _MAX_LABEL_COST;
				l_state_node_predecessor[i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
				l_state_time_predecessor[i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
				l_state_carrying_predecessor[i][t][w] = -1;
			}
		}
	}

	//step 2: Initialization for origin node at the preferred departure time, at departure time

		int w0 = 0;  // start fro empty
		
		l_state_node_label_cost[origin_node][departure_time_beginning][w0] = 0;

	// step 3: //dynamic programming
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			if (t % 10 ==0)
			{
				cout << "vehicle " << vehicle_id << " is scanning time " << t << "..." << endl;
			}
			for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
			{
				int from_node = g_link_from_node_number[link];

				int to_node = g_link_to_node_number[link];


				// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
				float travel_time_3_points = g_vehicle_origin_based_node_travel_time[vehicle_id][from_node] + g_vehicle_destination_based_node_travel_time[vehicle_id][from_node];
				int time_window_length = g_vehicle_arrival_time_ending[vehicle_id] - g_vehicle_departure_time_ending[vehicle_id];

				// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
				if (travel_time_3_points >= time_window_length)
					continue;  //skip


				int upstream_p = g_node_passenger_id[from_node];
				int downsteram_p = g_node_passenger_id[to_node];

				int travel_time = g_link_free_flow_travel_time[link];

				for (int w1 = 0; w1 < g_VRStateVector.size(); w1++)
				{
				
					if (g_VRStateVector[w1].m_vehicle_capacity > vehicle_capacity) // skip all states exceeding vehicle capacity
						continue;


					if (l_state_node_label_cost[from_node][t][w1] < _MAX_LABEL_COST - 1)  // for feasible time-space point only
						{


							for (int w2_index = 0; w2_index < g_VRStateVector[w1].m_outgoing_state_index_vector.size(); w2_index++)
							{
								if (g_VRStateVector[w1].m_outgoing_state_change_service_code_vector[w2_index] != g_link_service_code[link])  //0,  +p or -p
									continue;
									
								if (g_link_service_code[link] != 0)
									TRACE("service_link!");

							
								int w2 = g_VRStateVector[w1].m_outgoing_state_index_vector[w2_index];

								if (g_VRStateVector[w2].m_vehicle_capacity > vehicle_capacity)
									continue;

							// part 1: link based update
								int new_to_node_arrival_time = min(t + travel_time, g_number_of_time_intervals-1);

							if (g_node_passenger_id[to_node] >= 1 && g_activity_node_starting_time[to_node] >= 0 && g_activity_node_ending_time[to_node] >= 0)
								// passegner activity node: origin or destination
							{
							
								if (new_to_node_arrival_time < g_activity_node_starting_time[to_node]
									|| new_to_node_arrival_time > g_activity_node_ending_time[to_node])
								{
									// skip scanning when the destination nodes arrival time is out of time window
									continue; 
								}
							}

							if (to_node_cost[to_node][new_to_node_arrival_time] <-0.1)
								TRACE("negative cost on service_link!");


									//					if (g_shortest_path_debugging_flag)
									//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
									//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
							float temporary_label_cost = l_state_node_label_cost[from_node][t][w1] + arc_cost[link][t] + to_node_cost[to_node][new_to_node_arrival_time];

							if (g_link_service_code[link] != 0 && w1 == 1 && w2 == 0 && temporary_label_cost <-0.1)
								TRACE("delivery link!");


							if (temporary_label_cost < l_state_node_label_cost[to_node][new_to_node_arrival_time][w2]) // we only compare cost at the downstream node ToID at the new arrival time t
							{

								if (g_shortest_path_debugging_flag)
								{
									fprintf(g_pFileDebugLog, "DP: updating node: %d from time %d to time %d, current cost: %.2f, from cost %.2f ->%.2f\n",
										to_node, t, new_to_node_arrival_time,
										l_state_node_label_cost[from_node][t][w2],
										l_state_node_label_cost[to_node][new_to_node_arrival_time][w2], temporary_label_cost);
								}

								// update cost label and node/time predecessor

								l_state_node_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
								l_state_node_predecessor[to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								l_state_time_predecessor[to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								l_state_carrying_predecessor[to_node][new_to_node_arrival_time][w2] = w1;
							}
							// part 2: same node based update for waiting arcs

							if (w2 == w1) // for the same state
							{
							
								new_to_node_arrival_time = min(t + 1, g_number_of_time_intervals -1);

							//					if (g_shortest_path_debugging_flag)
							//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
							//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
							 temporary_label_cost = l_state_node_label_cost[from_node][t][w1] + vertex_cost[from_node][t];


							 if (temporary_label_cost < l_state_node_label_cost[from_node][new_to_node_arrival_time][w1]) // we only compare cost at the downstream node ToID at the new arrival time t
							{

								if (g_shortest_path_debugging_flag)
								{
									fprintf(g_pFileDebugLog, "DP: updating node: %d from time %d at state %d to time %d at state %d, current cost: %.2f, from cost %.2f ->%.2f\n",
										to_node, t, w1,
										new_to_node_arrival_time,
										w2,
										l_state_node_label_cost[from_node][t][w1],
										l_state_node_label_cost[to_node][new_to_node_arrival_time][w1], temporary_label_cost);
								}

								// update cost label and node/time predecessor

								l_state_node_label_cost[from_node][new_to_node_arrival_time][w1] = temporary_label_cost;
								l_state_node_predecessor[from_node][new_to_node_arrival_time][w1] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								l_state_time_predecessor[from_node][new_to_node_arrival_time][w1] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								l_state_carrying_predecessor[from_node][new_to_node_arrival_time][w1] = w1;
							}
							}

							}
						}  // feasible vertex label cost
					}  // for all states

				} // for all link
		} // for all time t


	if (g_shortest_path_debugging_flag)
	{
		fprintf(g_pFileDebugLog, "--Node label cost matrix--\n");

		for (int i = 1; i <= g_number_of_nodes; i++)
		{

			for (int t = 0; t <= arrival_time_ending; t++)
			{
				//	if (g_node_label_cost[i][t] < _MAX_LABEL_COST - 1) // feasible cost label
				for (int w = 0; w < g_VRStateVector.size(); w++)
				{
					if (l_state_node_label_cost[i][t][w] < _MAX_LABEL_COST - 1)
					{
					fprintf(g_pFileDebugLog, "Node %d @ %d: w = %d, %4.2f, node pred = %d time pred t= %d\n",
						i, t, w, l_state_node_label_cost[i][t][w], l_state_node_predecessor[i][t][w], l_state_time_predecessor[i][t][w]);
					}

				}
			}
		}
		fprintf(g_pFileDebugLog, "--End of node label cost matrix--\n");


	}


	total_cost = _MAX_LABEL_COST;

	int min_cost_time_index = arrival_time_ending;

	int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	int reversed_path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	float reversed_path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];

	int w = 0;
	total_cost = l_state_node_label_cost[destination_node][min_cost_time_index][w];

	// step 2: backtrack to the origin (based on node and time predecessors)
	int	node_size = 0;
	reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
	reversed_path_time_sequence[node_size] = min_cost_time_index;
	reversed_path_state_sequence[node_size] = w;
	reversed_path_cost_sequence[node_size] = l_state_node_label_cost[destination_node][min_cost_time_index][w];


	node_size++;

	int pred_node = l_state_node_predecessor[destination_node][min_cost_time_index][w];
	int pred_time = l_state_time_predecessor[destination_node][min_cost_time_index][w];
	int pred_state = l_state_carrying_predecessor[destination_node][min_cost_time_index][w];

	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
	{
		reversed_path_node_sequence[node_size] = pred_node;
		reversed_path_time_sequence[node_size] = pred_time;
		reversed_path_state_sequence[node_size] = pred_state;
		reversed_path_cost_sequence[node_size] = l_state_node_label_cost[pred_node][pred_time][pred_state];

		node_size++;

		//record current values of node and time predecessors, and update PredNode and PredTime

		int pred_node_record = pred_node;
		int pred_time_record = pred_time;
		int pred_state_record = pred_state;

		pred_node = l_state_node_predecessor[pred_node_record][pred_time_record][pred_state_record];
		pred_time = l_state_time_predecessor[pred_node_record][pred_time_record][pred_state_record];
		pred_state = l_state_carrying_predecessor[pred_node_record][pred_time_record][pred_state_record];

	}

	//reverse the node sequence 

	for (int n = 0; n < node_size; n++)
	{
		path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
		path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
		path_state_sequence[n] = reversed_path_state_sequence[node_size - n - 1];
		path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
	}

	for (int i = 0; i < node_size - 1; i++)  // for each link, 
	{

		int link_no = g_get_link_no_based_on_from_node_to_node(path_node_sequence[i], path_node_sequence[i + 1]);
		path_link_sequence[i] = link_no;
	}

	travel_time_return_value = path_time_sequence[node_size - 1] - path_time_sequence[0];

	path_number_of_nodes = node_size;
	return total_cost;

}


void g_ReadInputData()
{

	// initialization
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++)
	{
		g_outbound_node_size[i] = 0;
		g_inbound_node_size[i] = 0;

	}

	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0


	int interval_node_no = 0;
	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type;
			int node_id;
			double X;
			double Y;
			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;


			if (node_id <= 0 || g_number_of_nodes >= _MAX_NUMBER_OF_NODES)
			{
				cout << "node_id " << node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			g_internal_node_no_map[node_id] = interval_node_no;

			g_external_node_id_map[interval_node_no] = node_id;
				interval_node_no++;

			parser.GetValueByFieldName("node_type", node_type);
			parser.GetValueByFieldName("x", X);
			parser.GetValueByFieldName("y", Y);

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 ==0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;

		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		g_number_of_physical_nodes = g_number_of_nodes;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		parser.CloseCSVFile();

	}

	// step 2: read link file 

	if (parser.OpenCSVFile("input_link.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			if (from_node_id <= 0 )
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (to_node_id <= 0 )
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map.find(from_node_id) == g_internal_node_no_map.end())
			{
				cout << "from_node_id " << from_node_id << " has not been defined in node block" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map.find(to_node_id) == g_internal_node_no_map.end())
			{
				cout << "to_node_id " << to_node_id << " has not been defined in node block" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map[from_node_id] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			if (g_internal_node_no_map[to_node_id] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			// add the to node id into the outbound (adjacent) node list

			int direction = 1;
			parser.GetValueByFieldName("direction", direction);

			if (direction <= -2 || direction >= 2)
			{
				cout << "direction " << direction << " is out of range" << endl;
				g_ProgramStop();
			}

			for (int link_direction = -1; link_direction <= 1; link_direction += 2)  // called twice; -1 direction , 1 direction 
			{
				if (direction == -1 && link_direction == 1)
					continue; // skip

				if (direction == 1 && link_direction == -1)
					continue; // skip

				// then if  direction == 0 or 2 then create the corresponding link




				int directional_from_node_id = g_internal_node_no_map[from_node_id];
				int directional_to_node_id = g_internal_node_no_map[to_node_id];


				if (link_direction == -1) // reverse direction;
				{
					directional_from_node_id = g_internal_node_no_map[to_node_id];
					directional_to_node_id = g_internal_node_no_map[from_node_id];
				}

				g_outbound_node_id[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = directional_to_node_id;
				g_outbound_link_no[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = g_number_of_links;

				g_outbound_node_size[directional_from_node_id]++;
				g_inbound_node_id[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = directional_from_node_id;
				g_inbound_link_no[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = g_number_of_links;
				g_inbound_node_size[directional_to_node_id]++;

				float link_length = 1;
				int number_of_lanes = 1;
				int mode_code = 0;
				float capacity_per_time_interval = 1;
				float travel_time = 1.0;
				float speed = 1;
				float jam_density = 200;


				parser.GetValueByFieldName("length", link_length);
				parser.GetValueByFieldName("number_of_lanes", number_of_lanes);
				parser.GetValueByFieldName("mode_code", mode_code);
				parser.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity_per_time_interval);

				parser.GetValueByFieldName("speed_limit_in_mph", speed);
				if (speed >= 70)
					speed = 70;

				if (speed <= 25)
					speed = 25;


				travel_time = link_length * 60 / max(1, speed);
				parser.GetValueByFieldName("jam_density", jam_density);


				if (travel_time > 100)
				{
					cout << "travel_time > 100"; 
					g_ProgramStop();
				}

				g_link_from_node_number[g_number_of_links] = directional_from_node_id;
				g_link_to_node_number[g_number_of_links] = directional_to_node_id;

				g_link_free_flow_travel_time[g_number_of_links] = max(1, travel_time+0.5);   // at least 1 min, round to nearest integers
				g_link_free_flow_travel_time_float_value[g_number_of_links] = travel_time;

				g_link_link_length[g_number_of_links] = link_length;
				g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
				g_link_mode_code[g_number_of_links] = mode_code;
				g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
				g_link_speed[g_number_of_links] = speed;

				// increase the link counter by 1
				g_number_of_links++;

				if (g_number_of_links % 1000==0)
					cout << "reading " << g_number_of_links << " links.. " << endl;

			}

		}
		 
		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

		parser.CloseCSVFile();
	}


	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string agent_id;
			parser.GetValueByFieldName("agent_id", agent_id);

			int agent_type = 0;
			parser.GetValueByFieldName("agent_type", agent_type);
			int external_from_node_id;
			int external_to_node_id;
			int from_node_id;
			int to_node_id;


			if (agent_type == 0) //passenger
			{
				int pax_no =  g_number_of_passengers + 1;

				g_internal_agent_no_map[agent_id] = pax_no;
				g_external_passenger_id_map[pax_no + 1] = agent_id;
					

				if (pax_no >= _MAX_NUMBER_OF_PASSENGERS)
				{
					cout << "Agent+ can handle  " << _MAX_NUMBER_OF_PASSENGERS << "passengers" << endl;
					g_ProgramStop();
				}


				parser.GetValueByFieldName("from_node_id", external_from_node_id);
				parser.GetValueByFieldName("to_node_id", external_to_node_id);
			
				from_node_id = g_internal_node_no_map[external_from_node_id];
				to_node_id = g_internal_node_no_map[external_to_node_id];

				g_passenger_origin_node[pax_no] = from_node_id;
				g_passenger_destination_node[pax_no] = to_node_id;



				parser.GetValueByFieldName("departure_time_start", g_passenger_departure_time_beginning[pax_no]);
				int departure_time_window = 0;
				parser.GetValueByFieldName("departure_time_window", departure_time_window);
				g_passenger_departure_time_ending[pax_no] = max(0, departure_time_window);


				g_passenger_departure_time_ending[pax_no] = max(g_passenger_departure_time_ending[pax_no], g_passenger_departure_time_beginning[pax_no]);

				parser.GetValueByFieldName("arrival_time_start", g_passenger_arrival_time_beginning[pax_no]);

				int arrival_time_window = 0;
				parser.GetValueByFieldName("arrival_time_window", arrival_time_window);

				g_passenger_arrival_time_ending[pax_no] = g_passenger_arrival_time_beginning[pax_no] + arrival_time_window;

				g_passenger_arrival_time_ending[pax_no] = max(g_passenger_arrival_time_ending[pax_no], g_passenger_arrival_time_beginning[pax_no]);
				g_number_of_time_intervals = max(g_passenger_arrival_time_ending[pax_no] + 10, g_number_of_time_intervals);

				int new_artifical_pasenger_origin_id = g_add_new_node(pax_no, g_passenger_departure_time_beginning[pax_no], g_passenger_departure_time_ending[pax_no]);
				g_add_new_link(g_passenger_origin_node[pax_no], new_artifical_pasenger_origin_id, pax_no);  // pick up link
				g_add_new_link(new_artifical_pasenger_origin_id, g_passenger_origin_node[pax_no]);

				int new_artifical_pasenger_destination_id = g_add_new_node(pax_no);
				g_add_new_link(g_passenger_destination_node[pax_no], new_artifical_pasenger_destination_id, pax_no*(-1));  // delivery link
				g_add_new_link(new_artifical_pasenger_destination_id, g_passenger_destination_node[pax_no]);

				parser.GetValueByFieldName("base_profit", g_passenger_base_profit[pax_no]);

				

				g_number_of_passengers++;
			}
			else
			{  // vehicle


				int vehicle_no = g_number_of_vehicles + 1;
				g_external_vehicle_id_map[vehicle_no] = agent_id;


				parser.GetValueByFieldName("from_node_id", external_from_node_id);
				parser.GetValueByFieldName("to_node_id", external_to_node_id);

				from_node_id = g_internal_node_no_map[external_from_node_id];
				to_node_id = g_internal_node_no_map[external_to_node_id];

				g_vehicle_origin_node[vehicle_no] = from_node_id;
				g_vehicle_destination_node[vehicle_no] = to_node_id;

				parser.GetValueByFieldName("departure_time_start", g_vehicle_departure_time_beginning[vehicle_no]);
				int departure_time_window = 0;
				parser.GetValueByFieldName("departure_time_window", departure_time_window);
				g_vehicle_departure_time_ending[vehicle_no] = g_vehicle_departure_time_beginning[vehicle_no] + max(1, departure_time_window);
				g_vehicle_arrival_time_beginning[vehicle_no] = -1;
				parser.GetValueByFieldName("arrival_time_start", g_vehicle_arrival_time_beginning[vehicle_no]);

				if (g_vehicle_arrival_time_beginning[vehicle_no] < 0)
				{
					cout << "Vehicle data must have values in field arrival_time_start in file input_agent.csv!" << endl;
					g_ProgramStop();
				}
				int arrival_time_window = -1;
				parser.GetValueByFieldName("arrival_time_window", arrival_time_window);

				if (arrival_time_window < 0)
				{
					cout << "Vehicle data must have values in field arrival_time_window in file input_agent.csv!" << endl;
					g_ProgramStop();
				}
				g_vehicle_arrival_time_ending[vehicle_no] = g_vehicle_arrival_time_beginning[vehicle_no] + max(1, arrival_time_window);


				g_number_of_time_intervals = max(g_vehicle_arrival_time_ending[vehicle_no] + 10, g_number_of_time_intervals);

				if (g_vehicle_arrival_time_ending[vehicle_no] < g_vehicle_departure_time_beginning[vehicle_no] + 60)  // we should use a shortest path travel time to check. 
				{
					cout << "warning: Arrival time for vehicle " << vehicle_no << " should be " << g_vehicle_departure_time_beginning[vehicle_no] + 120 << endl;
					g_vehicle_arrival_time_ending[vehicle_no] = g_vehicle_departure_time_beginning[vehicle_no] + 60;
	//				g_ProgramStop();
				}

				g_activity_node_flag[g_vehicle_origin_node[vehicle_no]] = 1;
				g_activity_node_flag[g_vehicle_destination_node[vehicle_no]] = 1;
				g_activity_node_ending_time[g_vehicle_origin_node[vehicle_no]] = g_vehicle_departure_time_ending[vehicle_no];
				g_activity_node_ending_time[g_vehicle_destination_node[vehicle_no]] = g_vehicle_arrival_time_ending[vehicle_no];
				
				
				g_vehicle_capacity[vehicle_no] = -1;



				parser.GetValueByFieldName("capacity", g_vehicle_capacity[vehicle_no]);
				if (g_vehicle_capacity[vehicle_no] < 0)
				{
					cout << "Vehicle data must have values in field capacity in file input_agent.csv!" << endl;
					g_ProgramStop();
				}
				parser.GetValueByFieldName("VOIVTT_per_hour", g_VOIVTT_per_hour[vehicle_no]);
				parser.GetValueByFieldName("VOWT_per_hour", g_VOWT_per_hour[vehicle_no]);
				

				if (g_max_vehicle_capacity < g_vehicle_capacity[vehicle_no])
					g_max_vehicle_capacity = g_vehicle_capacity[vehicle_no];

				g_number_of_vehicles++;
			}


		}
		parser.CloseCSVFile();
	}

	fprintf(g_pFileOutputLog, "number of passengers=, %d\n", g_number_of_passengers);

	fprintf(g_pFileOutputLog, "number of vehicles =,%d\n", g_number_of_vehicles);

	//beginning for addig virtual vehicles
	g_number_of_physical_vehicles = g_number_of_vehicles;

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		int	v = (g_number_of_vehicles+1); //new vehicle id

			g_vehicle_origin_node[v] = g_passenger_origin_node[p];
			g_vehicle_destination_node[v] = g_passenger_origin_node[p];
			g_vehicle_departure_time_beginning[v] = max(0, g_passenger_departure_time_ending[p] - 10);
			g_vehicle_departure_time_ending[v] = g_vehicle_departure_time_beginning[v];
			g_vehicle_arrival_time_beginning[v] = g_number_of_time_intervals - 1;
			g_vehicle_arrival_time_ending[v] = g_number_of_time_intervals - 1;
			g_vehicle_capacity[v] = 1;
			g_VOIVTT_per_hour[v] = g_dummy_vehicle_cost_per_hour;
			g_VOWT_per_hour[v] = 0;
			g_number_of_vehicles++;
	}

	//end of adding dummy vehicles for each passenger

	cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_passengers << " passengers, " << g_number_of_vehicles << "vehicles" << endl;
	fprintf(g_pFileDebugLog, "network has %d nodes, %d links, %d toll records, %d  passengers, %d vehicles\n",
		g_number_of_nodes, g_number_of_links, g_number_of_toll_records, g_number_of_passengers, g_number_of_vehicles);


	for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
	{
		int from_node = g_link_from_node_number[link];

		int to_node = g_link_to_node_number[link];

		if (g_link_service_code[link] != 0)
		{

		fprintf(g_pFileDebugLog, "link no.%d,  %d->%d, service code %d\n",
			link + 1,
			from_node,
			to_node,
			g_link_service_code[link]);
		}
	}
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileDebugLog, "passenger %d starts from node %d to node %d, start time %d->%d, ending time %d->%d\n",
			p,
			g_external_node_id_map[g_passenger_origin_node[p]],
			g_external_node_id_map[g_passenger_destination_node[p]],
			g_passenger_departure_time_beginning[p],
			g_passenger_departure_time_ending[p],
			g_passenger_arrival_time_beginning[p],
			g_passenger_arrival_time_ending[p]);


		fprintf(g_pFileAgentPathLog, "0,p%s,p,pax,%d;%d,%d;%d\n",
			g_external_passenger_id_map[p].c_str(),
			g_external_node_id_map[g_passenger_origin_node[p]],
			g_external_node_id_map[g_passenger_destination_node[p]],
			g_passenger_departure_time_beginning[p],
			g_passenger_departure_time_beginning[p]+10
			);

	}



	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		fprintf(g_pFileDebugLog, "vehicle %d starts from node %d to node %d, start time %d->%d, ending time %d->%d\n",
			v,
			g_external_node_id_map[g_vehicle_origin_node[v]],
			g_external_node_id_map[g_vehicle_destination_node[v]],
			g_vehicle_departure_time_beginning[v],
			g_vehicle_departure_time_ending[v],
			g_vehicle_arrival_time_beginning[v],
			g_vehicle_arrival_time_ending[v]);

	}
}


void g_Finding_Shorest_Path_For_All_Passengers(float VOIVTT_per_hour)
{

	for (int p = 1; p <= g_number_of_passengers; p++)
	{

		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_arc_cost[link][t] = g_arc_travel_time[link][t] / 60.0 * VOIVTT_per_hour;  // 60 min pur hour
			}
		}

		// setup waiting cost
		for (int node = 0; node < g_number_of_nodes; node++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_vertex_waiting_cost[node][t] = 0;
			}
		}




	}

}


bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables()  // with varaible y only
{

	fprintf(g_pFileOutputLog, "\nIteration,Lower Bound,Upper Bound,Gap,Relative_gap,");
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "pax %d,",p);
	}
	fprintf(g_pFileOutputLog, "# of pax not served,");


	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "best_p%d,", p);
	}
	fprintf(g_pFileOutputLog, "best_# of pax not served,");

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "price_p%d,", p);
	}
	fprintf(g_pFileOutputLog, "total_price,");

	fprintf(g_pFileOutputLog, "\n");



	cout << "Preparation......" << endl;
	int VOIVTT_per_hour = 50;
	g_Finding_Shorest_Path_For_All_Passengers(VOIVTT_per_hour);

	//step 0: initialization 
	fprintf(g_pFileDebugLog, "step 0: initialization \n");

	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < g_number_of_time_intervals; t++)
		{
			g_arc_travel_time[link][t] = g_link_free_flow_travel_time[link];  //transportation cost

		}
	}

	// setup waiting cost
	for (int node = 0; node < g_number_of_nodes; node++)
	{
		for (int t = 0; t < g_number_of_time_intervals; t++)
		{
			g_vertex_waiting_cost[node][t] = 1;
			g_vertex_visit_count[node][t] = 0;

		}
	}




	//cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;
	//g_SolutionStartTime = CTime::GetCurrentTime();


	//loop for each LR iteration

	float g_best_upper_bound = 99999;
	float g_best_lower_bound = -99999;

	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{

		// reset the vertex visit count
		for (int node = 0; node < g_number_of_nodes; node++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_vertex_visit_count[node][t] = 0;

			}
		}

		double global_lower_bound = 0;
		double global_upper_bound = 99999;

		cout << "Lagrangian Iteration " << LR_iteration << "/" << g_number_of_LR_iterations << endl;
		fprintf(g_pFileDebugLog, "----------Lagrangian Iteration: %d ----------------------------------------\n", LR_iteration + 1);

		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;


		//step 2: shortest path for vehicle

		for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here
		{
			// set arc cost, to_node_cost and waiting_cost for vehicles

			for (int link = 0; link < g_number_of_links; link++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_arc_cost[link][t] = g_arc_travel_time[link][t] / 60.0 * g_VOIVTT_per_hour[v];  // 60 min pur hour
				}
			}

			// setup waiting cost
			for (int node = 0; node < g_number_of_nodes; node++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_vertex_waiting_cost[node][t] = 1 / 60.0* g_VOWT_per_hour[v];;
				}
			}
			// special case: no waiting cost at vehicle returning depot

			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				int vehicle_destination_node = g_vehicle_destination_node[v];
				g_vertex_waiting_cost[vehicle_destination_node][t] = 0;
			}

			float path_cost_by_vehicle_v =
				g_optimal_time_dependenet_dynamic_programming(v,
				g_arc_cost,
				g_to_node_cost,
				g_vertex_waiting_cost,
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
				g_vehicle_path_state_sequence[v],
				g_vehicle_path_cost_sequence[v],
				0,
				g_vehicle_capacity[v],
				g_path_travel_time);


			fprintf(g_pFileDebugLog, "\Vehicle %d'  path has %d nodes with a transportation cost of %f and travel time of %d: ",
				v,
				g_vehicle_path_number_of_nodes[v],
				path_cost_by_vehicle_v,
				g_path_travel_time);
			fprintf(g_pFileDebugLog, "\n");


			fprintf(g_pFileDebugLog, "[[time, node, state, service code, travel cost, to_node_cost,cumulative_cost ($), travel time(interval), travel time(min)");
			fprintf(g_pFileDebugLog, "\n");





			float cumulative_cost = 0;
			float DP_cumulative_cost = 0;
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
			{
				fprintf(g_pFileDebugLog, "veh:%d,%5d,%5d,",
					v,
					g_vehicle_path_time_sequence[v][i],
					g_vehicle_path_node_sequence[v][i]
					);

				DP_cumulative_cost = g_vehicle_path_cost_sequence[v][i];


				int state_index = g_vehicle_path_state_sequence[v][i];

				std::string state_str;

				if (state_index < g_VRStateVector.size())
				{

					state_str = g_VRStateVector[state_index].generate_string_key();
				}

				fprintf(g_pFileDebugLog, "[%s],",
					state_str.c_str()
					);

				int link_id = -1;
				int service_code = 0;
				float travel_cost = 0;
				float to_node_cost = 0;
				int travel_time = 0;
				float float_travel_time = 0;

				if (i < g_vehicle_path_number_of_nodes[v] - 1)
				{
					link_id = g_get_link_no_based_on_from_node_to_node(g_vehicle_path_node_sequence[v][i], g_vehicle_path_node_sequence[v][i + 1]);
					if (link_id >= 0)
					{

						travel_cost = g_arc_cost[link_id][g_vehicle_path_time_sequence[v][i]];
						to_node_cost = g_to_node_cost[g_vehicle_path_node_sequence[v][i + 1]][g_vehicle_path_time_sequence[v][i + 1]];
						cumulative_cost += (travel_cost + to_node_cost);
						service_code = g_link_service_code[link_id];
						travel_time = g_link_free_flow_travel_time[link_id];
						float_travel_time = g_link_free_flow_travel_time_float_value[link_id];
					}
				}


				if (link_id >= 0)  // physical transportation link
				{

					std::string service_str;

					if (service_code == 0)
						service_str = "moving..";
					if (service_code > 0)
						service_str = "!picking up ";
					if (service_code < 0)
						service_str = "!dropping off ";


					if (service_code == 0)
					{
						fprintf(g_pFileDebugLog, "%s,$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2f,",
							service_str.c_str(),
							travel_cost,
							to_node_cost,
							cumulative_cost,
							DP_cumulative_cost,
							travel_time,
							float_travel_time);


					}
					else
					{
						fprintf(g_pFileDebugLog, "%s(%2d),$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2f,",
							service_str.c_str(),
							service_code,
							travel_cost,
							to_node_cost,
							cumulative_cost,
							DP_cumulative_cost,
							travel_time,
							float_travel_time);
					}

				}
				else
				{
					float waiting_cost = g_vertex_waiting_cost[g_vehicle_path_node_sequence[v][i]][g_vehicle_path_time_sequence[v][i]];
					cumulative_cost += waiting_cost;
					fprintf(g_pFileDebugLog, "--waiting, $%.4f,$%.4f,$*%.4f,-",
						waiting_cost,
						cumulative_cost,
						DP_cumulative_cost
						);
				}
				fprintf(g_pFileDebugLog, "\n");

			}

			fprintf(g_pFileDebugLog, "]]\n");

			//if (path_cost_by_vehicle_v > g_idle_vehicle_benefit)  // no enough profit
			//{
			//	path_cost_by_vehicle_v = g_idle_vehicle_benefit;
			//	//global_upper_bound += g_idle_vehicle_benefit;
			//}  // otherwise, do something today. 

			global_lower_bound += path_cost_by_vehicle_v;
			fprintf(g_pFileDebugLog, "global_lower_bound for vehicle %d += %f = %f\n", v, path_cost_by_vehicle_v, global_lower_bound);

		}






		// step 3: scan all vehicles to mark the useage of the corresponding vertex 
		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v] - 1; i++)  // for each link along the path
			{
				int node = g_vehicle_path_node_sequence[v][i];
				int time_index = g_vehicle_path_time_sequence[v][i];

				if (node >= 0 && g_node_passenger_id[node] >= 1) // feasible from node i
				{
					g_vertex_visit_count[node][time_index] += 1;  // vehicle uses the link at time t
				}
			}

		}


		// step 4: scan all passengers and update multipliers and to_node_cost

		double total_multiplier_price_with_constant = 0;
		// scan multipliers 
		for (int node = 1; node <= g_number_of_nodes; node++)
		{
			if (g_node_passenger_id[node] >= 1 && g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
			{

				for (int t = g_activity_node_starting_time[node]; t <= g_activity_node_ending_time[node]; t++)  // for all vertex in the time window
				{
	
					total_multiplier_price_with_constant += g_passenger_activity_node_multiplier[node][t];

		
				}

			}


		}

		global_lower_bound += total_multiplier_price_with_constant*(-1);

		// update multipliers using subgradient
		double total_multiplier_price = 0;
		for (int node = 1; node <= g_number_of_nodes; node++)
		{
			if (g_node_passenger_id[node] >= 1 && g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
			{
				int number_of_visits = 0;
				for (int t = g_activity_node_starting_time[node]; t <= g_activity_node_ending_time[node]; t++)
				{
					number_of_visits += g_vertex_visit_count[node][t];
				}


				float StepSize = g_passenger_base_profit[g_node_passenger_id[node]] / (LR_iteration + 1.0f);
				if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
				{
					StepSize = g_minimum_subgradient_step_size;
				}

				for (int t = g_activity_node_starting_time[node]; t <= g_activity_node_ending_time[node]; t++)  // for all vertex in the time window
				{
					g_passenger_activity_node_multiplier[node][t] += StepSize * (number_of_visits - 1);  // decrease the value and create profit
					g_to_node_cost[node][t] = g_passenger_activity_node_multiplier[node][t];

					//for pax's origin with the fixed departure time only for now. need to consider destination's price and flexible time window later
					g_passenger_origin_multiplier[g_node_passenger_id[node]] = g_to_node_cost[node][t];

					total_multiplier_price += g_passenger_activity_node_multiplier[node][t];

					if (number_of_visits == 1)
					{

						fprintf(g_pFileDebugLog, "\>>> Passenger %d  iteration no. %d at node %d and time %d visit = %d, the multiplier keeps as  %f",

							g_node_passenger_id[node],
							LR_iteration,
							node,
							t,
							number_of_visits,
							g_to_node_cost[node][t]);
						fprintf(g_pFileDebugLog, "\n");
					}
					else
					{
						fprintf(g_pFileDebugLog, "\>>> Passenger  %d iteration no. %d  at node %d and time %d visit = %d, the multiplier changes to  %f",

							g_node_passenger_id[node],
							LR_iteration,
							node,
							t,
							number_of_visits,
							g_to_node_cost[node][t]);
						fprintf(g_pFileDebugLog, "\n");
					}
				}

			}


		}


		int number_of_pax_not_served = 0;
		global_upper_bound = g_UpperBoundGeneration(LR_iteration);
		// perform upper bound solutoin based on LR heuristics
		// for each vehicle
		// perform DP
		// mark vehicles served
		// reset to_node cost
		//end




		//CTimeSpan ctime = CTime::GetCurrentTime() - g_SolutionStartTime;
		//		g_LogFile << "Computational time:," << ctime.GetTotalSeconds() << ",Iterationvehicle_id:, " << LR_iteration + 1 << ",Lower bound in Minute:," << globallowerbound / g_MinuteDivision << ",Upper bound in Minute:," << globalupperbound / g_MinuteDivision << ",Lower bound:," << globallowerbound << ",Upper bound:," << globalupperbound << ",Total Trip Price:," << TotalTripPrice << ",Total Resource Price (Resource Cost):," << TotalResourcePrice << ",Total Travel Time:," << TotalTravelTime << ",Optimality gap:," << (globalupperbound - globallowerbound) / globalupperbound << endl;


		g_best_lower_bound = max(g_best_lower_bound, global_lower_bound);

		if (global_upper_bound < g_best_upper_bound)
		{  // update upper bound value
			g_best_upper_bound = global_upper_bound;
			//update best upper bound soluti
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				g_best_solution_vehicle_id_for_pax_being_served[p] = g_vehicle_id_for_pax_being_served[p];
			}

		}


		fprintf(g_pFileDebugLog, "Summary: Iteration %d: Lower Bound = %f, upper Bound = %f, gap = %f, relative_gap = %.3f%%, # of pax not served = %d\n",
			LR_iteration+1,
			g_best_lower_bound,
			g_best_upper_bound,
			(g_best_upper_bound - g_best_lower_bound),
			(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0,
			number_of_pax_not_served
			);

		fprintf(g_pFileOutputLog, "%d,%f,%f,%f,%.3f%%,",
			LR_iteration+1,
			g_best_lower_bound,
			g_best_upper_bound,
			(g_best_upper_bound - g_best_lower_bound),
			(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
			);
		//current solution
		int count_not_served = 0;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			if (g_vehicle_id_for_pax_being_served[p] > g_number_of_physical_vehicles)
			{
	
					fprintf(g_pFileOutputLog, "%d*,", g_vehicle_id_for_pax_being_served[p]);
					count_not_served++;
			}
			else
			{
				fprintf(g_pFileOutputLog, "%d,", g_vehicle_id_for_pax_being_served[p]);
			}

		}

		fprintf(g_pFileOutputLog, "%d,",count_not_served);

		// best solution 
		count_not_served = 0;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			if (g_best_solution_vehicle_id_for_pax_being_served[p] > g_number_of_physical_vehicles)
			{

				fprintf(g_pFileOutputLog, "%d*,", g_best_solution_vehicle_id_for_pax_being_served[p]);
				count_not_served++;
			}
			else
			{
				fprintf(g_pFileOutputLog, "%d,", g_best_solution_vehicle_id_for_pax_being_served[p]);
			}

		}

		fprintf(g_pFileOutputLog, "%d,", count_not_served);

		float total_price = 0;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{

			fprintf(g_pFileOutputLog, "%.2f,", g_passenger_origin_multiplier[p]*(-1.0));
			total_price += g_passenger_origin_multiplier[p];
	

		}
		
		fprintf(g_pFileOutputLog, "%.2f,", total_price*(-1.0));


		fprintf(g_pFileOutputLog, "\n");
	}


	//for each lagrangian relaxation iteration

	cout << "End of Lagrangian Iteration Process " << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;



	return true;
}


float g_UpperBoundGeneration(int LR_Iteration_no)
{
	float global_upper_bound = 0;
	int number_of_pax_not_served = 0;

	for (int p = 1; p <= g_number_of_passengers; p++)//note that the scheduling sequence does not matter  here
	{
		g_vehicle_id_for_pax_being_served[p] = 0;
	}
	for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does  matter  here
	{
		// setup 1: set arc cost, to_node_cost and waiting_cost for vehicles

		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_arc_cost[link][t] = g_arc_travel_time[link][t] / 60.0 * g_VOIVTT_per_hour[v];  // 60 min pur hour
			}
		}

		// setup waiting cost
		for (int node = 0; node < g_number_of_nodes; node++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_vertex_waiting_cost[node][t] = 1 / 60.0 * 15; // g_VOWT_per_hour[v];
				g_vertex_visit_count[node][t] = 0;
				g_to_node_cost_used_for_upper_bound[node][t] = 0;
			}
		}
		// special case: no waiting cost at vehicle returning depot

		for (int t = 0; t < g_number_of_time_intervals; t++)
		{
			int vehicle_destination_node = g_vehicle_destination_node[v];
			g_vertex_waiting_cost[vehicle_destination_node][t] = 0;
		}

		// setup to node cost for each vehicle in finding its own upper bound
		for (int node = 1; node <= g_number_of_nodes; node++)
		{
			
			if (g_node_passenger_id[node] >= 1 && g_vehicle_id_for_pax_being_served[g_node_passenger_id[node]] == 0 /*not served*/ && g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
			{
				for (int t = g_activity_node_starting_time[node]; t <= g_activity_node_ending_time[node]; t++)  // for all vertex in the time window
				{
					if (v<= g_number_of_physical_vehicles)  // physical vehicle
						g_to_node_cost_used_for_upper_bound[node][t] = g_to_node_cost[node][t];  // for existing lower bound solution
					else  // virtual vehicle
					{
						if (v - g_number_of_physical_vehicles == g_node_passenger_id[node])  // for your assigned pax id only
						{ 
							g_to_node_cost_used_for_upper_bound[node][t] = -100;  // for existing lower bound solution
						}
					}
				
				}
			}
		}
					
		//step 2: shortest path for vehicle with to node cost for passengers not being served

		float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_dynamic_programming(v,
			g_arc_cost,
			g_to_node_cost_used_for_upper_bound,
			g_vertex_waiting_cost,
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
			g_vehicle_path_state_sequence[v],
			g_vehicle_path_cost_sequence[v],
			0,
			g_vehicle_capacity[v],
			g_path_travel_time);


		fprintf(g_pFileDebugLog, "\Upper bound: Vehicle %d'  path has %d nodes with a transportation cost of %f and travel time of %d: ",
			v,
			g_vehicle_path_number_of_nodes[v],
			path_cost_by_vehicle_v,
			g_path_travel_time);
		fprintf(g_pFileDebugLog, "\n");


		fprintf(g_pFileDebugLog, "UB: [[time, node, state, service code, travel cost, to_node_cost,cumulative_cost ($), travel time(interval), travel time(min)");
		fprintf(g_pFileDebugLog, "\n");

		float cumulative_upper_bound_cost = 0;
		float DP_cumulative_cost = 0;

		//		day_no, , agent_type, path_node_sequence, path_time_sequence, path_state_sequence,

		if (LR_Iteration_no == g_number_of_LR_iterations - 1)
		{

		fprintf(g_pFileAgentPathLog, "%d,v%s,v,",
			LR_Iteration_no+1,
			g_external_vehicle_id_map[v].c_str());


		if (v <= g_number_of_physical_vehicles)
			fprintf(g_pFileAgentPathLog, "physical,");
		else
			fprintf(g_pFileAgentPathLog, "virtual,");


		for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
		{
			if (g_vehicle_path_node_sequence[v][i] < g_number_of_physical_nodes)  // output only physical nodes
			{

				if (i < g_vehicle_path_number_of_nodes[v] - 1)
				{
					if (g_vehicle_path_node_sequence[v][i] == g_vehicle_path_node_sequence[v][i]
						&& g_vehicle_path_node_sequence[v][i] == g_vehicle_path_node_sequence[v][i + 1])

						continue;  // dulicated nodes

				}


			fprintf(g_pFileAgentPathLog, "%d;",
				g_external_node_id_map[g_vehicle_path_node_sequence[v][i]]
				);
			}

		}
		fprintf(g_pFileAgentPathLog, ","); //end of field for node sequence

		for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
		{

			if (g_vehicle_path_node_sequence[v][i] < g_number_of_physical_nodes)  // output only physical nodes
			{
				if (i < g_vehicle_path_number_of_nodes[v] - 1)
				{
					if (g_vehicle_path_node_sequence[v][i] == g_vehicle_path_node_sequence[v][i]
						&& g_vehicle_path_node_sequence[v][i] == g_vehicle_path_node_sequence[v][i + 1])

						continue;  // dulicated nodes

				}
			
			fprintf(g_pFileAgentPathLog, "%d;",
				g_vehicle_path_time_sequence[v][i]
				);
			}
		}
		fprintf(g_pFileAgentPathLog, ","); //end of field for time sequence

		fprintf(g_pFileAgentPathLog, "\n"); //end of line
		}

		for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
		{

			if (g_external_node_id_map.find(g_vehicle_path_node_sequence[v][i]) != g_external_node_id_map.end())
			{
				fprintf(g_pFileDebugLog, "UB: veh:%d,%5d,%5d,",
					v,
					g_vehicle_path_time_sequence[v][i],

					g_external_node_id_map[g_vehicle_path_node_sequence[v][i]]);

			}

			else
			{
				fprintf(g_pFileDebugLog, "UB: veh:%d,%5d,*%5d,",
					v,
					g_vehicle_path_time_sequence[v][i],

					g_vehicle_path_node_sequence[v][i]);


			}

			DP_cumulative_cost = g_vehicle_path_cost_sequence[v][i];


			int state_index = g_vehicle_path_state_sequence[v][i];

			std::string state_str;

			if (state_index < g_VRStateVector.size())
			{

				state_str = g_VRStateVector[state_index].generate_string_key();
			}

			fprintf(g_pFileDebugLog, "[%s],",
				state_str.c_str()
				);

			int link_id = -1;
			int service_code = 0;
			float travel_cost = 0;
			float to_node_cost = 0;
			int travel_time = 0;
			float float_travel_time = 0;

			if (i < g_vehicle_path_number_of_nodes[v] - 1)
			{
				link_id = g_get_link_no_based_on_from_node_to_node(g_vehicle_path_node_sequence[v][i], g_vehicle_path_node_sequence[v][i + 1]);
				if (link_id >= 0)
				{

					travel_cost = g_arc_cost[link_id][g_vehicle_path_time_sequence[v][i]];
					to_node_cost = g_to_node_cost[g_vehicle_path_node_sequence[v][i + 1]][g_vehicle_path_time_sequence[v][i + 1]];
					cumulative_upper_bound_cost += (travel_cost);  // no to node cost
					service_code = g_link_service_code[link_id];
					travel_time = g_link_free_flow_travel_time[link_id];
					float_travel_time = g_link_free_flow_travel_time_float_value[link_id];
				}
			}


			if (link_id >= 0)  // physical transportation link
			{

				std::string service_str;

				if (service_code == 0)
					service_str = "moving..";
				if (service_code > 0)
					service_str = "!picking up ";
				if (service_code < 0)
					service_str = "!dropping off ";


				if (service_code == 0)
				{
					fprintf(g_pFileDebugLog, "UB: %s,$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2f,",
						service_str.c_str(),
						travel_cost,
						to_node_cost,
						cumulative_upper_bound_cost,
						DP_cumulative_cost,
						travel_time,
						float_travel_time);


				}
				else
				{
					fprintf(g_pFileDebugLog, "UB: %s(%2d),$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2f,",
						service_str.c_str(),
						service_code,
						travel_cost,
						to_node_cost,
						cumulative_upper_bound_cost,
						DP_cumulative_cost,
						travel_time,
						float_travel_time);
				}

			}
			else
			{
				float waiting_cost = g_vertex_waiting_cost[g_vehicle_path_node_sequence[v][i]][g_vehicle_path_time_sequence[v][i]];
				cumulative_upper_bound_cost += waiting_cost;
				fprintf(g_pFileDebugLog, "UB: --waiting, $%.4f,$%.4f,$*%.4f,-",
					waiting_cost,
					cumulative_upper_bound_cost,
					DP_cumulative_cost
					);
			}
			fprintf(g_pFileDebugLog, "\n");

		}

		fprintf(g_pFileDebugLog, "]]\n");


		global_upper_bound += cumulative_upper_bound_cost;
		fprintf(g_pFileDebugLog, "global_upper_bound for vehicle %d += %f = %f\n", v, cumulative_upper_bound_cost, global_upper_bound);

	
	// step 3: scan all passengers  not being served, mark the passengers just served by vehicle v
		for (int i = 0; i < g_vehicle_path_number_of_nodes[v] - 1; i++)  // for each link along the path
		{
			int node = g_vehicle_path_node_sequence[v][i];
			int time_index = g_vehicle_path_time_sequence[v][i];

			if (node >= 0 && g_node_passenger_id[node] >= 1) // feasible from node i
			{
				g_vertex_visit_count[node][time_index] += 1;  // vehicle uses the link at time t
			}
		}

		for (int node = 1; node <= g_number_of_nodes; node++)
		{
			if (g_node_passenger_id[node] >= 1 && g_vehicle_id_for_pax_being_served[g_node_passenger_id[node]] == 0 /*not served*/ && g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
			{
				int number_of_visits = 0;
				for (int t = g_activity_node_starting_time[node]; t <= g_activity_node_ending_time[node]; t++)
				{
					number_of_visits += g_vertex_visit_count[node][t];
				}

				if (number_of_visits >= 1)
				{
					g_vehicle_id_for_pax_being_served[g_node_passenger_id[node]] = v;
					fprintf(g_pFileDebugLog, "\!!! LR Iteration %d: Passenger %d being served by vehicle %d\n",
						LR_Iteration_no,
	g_node_passenger_id[node],
						v);
				}
			}
		}


	}

	// after running all physical vehicles, we default to the virutal or dummy vehicles
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		if (g_vehicle_id_for_pax_being_served[p] >= g_number_of_physical_vehicles)
			number_of_pax_not_served++;

		if (g_vehicle_id_for_pax_being_served[p] == 0) // not served
		{
			fprintf(g_pFileDebugLog, "Warning: pax %d has not been served by neither physical and virtual vehicles!\n", p);
		}
	}
	fprintf(g_pFileDebugLog, "Summary: Iteration %d: total operating cost =  %f\n",
		LR_Iteration_no, global_upper_bound);

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
			if (g_vehicle_id_for_pax_being_served[p] <= g_number_of_physical_vehicles)
			{
				fprintf(g_pFileDebugLog, "Summary: Iteration %d, pax %d -> veh %d\n",
					LR_Iteration_no, p, g_vehicle_id_for_pax_being_served[p]);
			}
			else
			{
				fprintf(g_pFileDebugLog, "Summary: Iteration %d, pax %d -> ** virtual veh %d\n",
					LR_Iteration_no, p, g_vehicle_id_for_pax_being_served[p]);


			}
	}

	return  global_upper_bound;
}
bool g_Optimization_Lagrangian_Method_Link_Capacity_Problem()
{
	



	return true;
}

void g_ReadConfiguration()
{
	CCSVParser parser;
	if (parser.OpenCSVFile("input_configuration.csv", true))
	{

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type;
			int node_id;
			double X;
			double Y;
			parser.GetValueByFieldName("number_of_iterations", g_number_of_LR_iterations);
			parser.GetValueByFieldName("shortest_path_debugging_details", g_shortest_path_debugging_flag);
			parser.GetValueByFieldName("dummy_vehicle_cost_per_hour", g_dummy_vehicle_cost_per_hour);
			


				
			

			break;  // only the first line
		}
		parser.CloseCSVFile();
	}
}



int g_ListFront;
int g_ListTail;
int g_SENodeList[_MAX_NUMBER_OF_NODES];


// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
void SEList_clear()
{
	g_ListFront = -1;
	g_ListTail = -1;
}

void SEList_push_front(int node)
{
	if (g_ListFront == -1)  // start from empty
	{
		g_SENodeList[node] = -1;
		g_ListFront = node;
		g_ListTail = node;
	}
	else
	{
		g_SENodeList[node] = g_ListFront;
		g_ListFront = node;
	}

}
void SEList_push_back(int node)
{
	if (g_ListFront == -1)  // start from empty
	{
		g_ListFront = node;
		g_ListTail = node;
		g_SENodeList[node] = -1;
	}
	else
	{
		g_SENodeList[g_ListTail] = node;
		g_SENodeList[node] = -1;
		g_ListTail = node;
	}
}

bool SEList_empty()
{
	return(g_ListFront == -1);
}

int SEList_front()
{
	return g_ListFront;
}

void SEList_pop_front()
{
	int tempFront = g_ListFront;
	g_ListFront = g_SENodeList[g_ListFront];
	g_SENodeList[tempFront] = -1;
}

int g_node_status_static_array[_MAX_NUMBER_OF_NODES];

float g_node_label_earliest_arrival_time[_MAX_NUMBER_OF_NODES];
int g_node_static_predecessor[_MAX_NUMBER_OF_NODES];


float g_optimal_label_correcting(int origin_node, int departure_time)
	// time-dependent label correcting algorithm with double queue implementation
{
	int internal_debug_flag = 0;

	float total_cost = _MAX_LABEL_COST;
	if (g_outbound_node_size[origin_node] == 0)
	{
		return _MAX_LABEL_COST;
	}

	for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes
	{
		g_node_status_static_array[i] = 0;  // not scanned
		g_node_label_earliest_arrival_time[i] = _MAX_LABEL_COST;
		g_node_static_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
	}

	//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node

	g_node_label_earliest_arrival_time[origin_node] = departure_time;


	SEList_clear();
	SEList_push_back(origin_node);


	while (!SEList_empty())
	{
		int from_node = SEList_front();//pop a node FromID for scanning

		SEList_pop_front();  // remove current node FromID from the SE list
		g_node_status_static_array[from_node] = 0;


		if (g_shortest_path_debugging_flag)
			fprintf(g_pFileDebugLog, "SP: SE node: %d\n", from_node);

		//scan all outbound nodes of the current node
		for (int i = 0; i < g_outbound_node_size[from_node]; i++)  // for each link (i,j) belong A(i)
		{
			int link_no = g_outbound_link_no[from_node][i];
			int to_node = g_outbound_node_id[from_node][i];


			bool  b_node_updated = false;


			int new_to_node_arrival_time = g_node_label_earliest_arrival_time[from_node] + g_link_free_flow_travel_time[link_no];

			//					if (g_shortest_path_debugging_flag)
			//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
			//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);



			if (new_to_node_arrival_time < g_node_label_earliest_arrival_time[to_node]) // we only compare cost at the downstream node ToID at the new arrival time t
			{

				//if (g_shortest_path_debugging_flag)
				//	fprintf(g_pFileDebugLog, "SP: updating node: %d from time %d to time %d, current cost: %.2f, from cost %.2f ->%.2f\n",
				//	to_node, t, new_to_node_arrival_time,
				//	g_node_label_cost[from_node][t],
				//	g_node_label_cost[to_node][new_to_node_arrival_time], temporary_label_cost);

				// update cost label and node/time predecessor

				g_node_label_earliest_arrival_time[to_node] = new_to_node_arrival_time;
				g_node_static_predecessor[to_node] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time

				b_node_updated = true;


				if (g_node_status_static_array[to_node] != 1)
				{
					if (g_shortest_path_debugging_flag)
						fprintf(g_pFileDebugLog, "SP: add node %d into SE List\n",
						to_node);


					SEList_push_back(to_node);
					g_node_status_static_array[to_node] = 1;
				}
			}

		}
	}

	for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes
	{
		g_node_to_node_shorest_travel_time[origin_node][i] = g_node_label_earliest_arrival_time[i];

	}
	return total_cost;

}

void g_generate_travel_time_matrix()
{
	for (int p1 = 1; p1 <= g_number_of_passengers; p1++)
	for (int p2 = 1; p2 <= g_number_of_passengers; p2++)
	{
		g_accessibility_matrix[p1][p2] = 1;
	}

	fprintf(g_pFileDebugLog, "--- travel time and cancelation cost ($)----\n");

	// for each pax
	for (int p = 1; p <= g_number_of_passengers; p++)
	{ 
		cout << ">>find shortest path tree for pax p = " << p << "..." << endl; 
		g_optimal_label_correcting(g_passenger_origin_node[p], g_passenger_departure_time_beginning[p]);
		

		float earliest_arrival_time = g_node_to_node_shorest_travel_time[g_passenger_origin_node[p]][g_passenger_destination_node[p]];
		g_passenger_request_travel_time_vector[p] = max(5, earliest_arrival_time - g_passenger_departure_time_beginning[p]);

		g_passenger_request_cancelation_cost[p] = g_passenger_request_travel_time_vector[p] / 60.0 * 30;
		g_optimal_label_correcting(g_passenger_destination_node[p], earliest_arrival_time);
		// from the pax's destination to all the other nodes starting from the earliest arrival time at the d
		fprintf(g_pFileDebugLog, "pax no.%d, Departure Time = %d (min), Travel Time = %.2f (min), Earliest Arrival Time = %.2f, cost = $%.2f\n",
			p, g_passenger_departure_time_beginning[p],
			g_passenger_request_travel_time_vector[p],
			earliest_arrival_time,
			g_passenger_request_cancelation_cost[p]);

	}
	//check rule 1:
	for (int p1 = 1; p1 <= g_number_of_passengers; p1++)
	for (int p2 = 1; p2 <= g_number_of_passengers; p2++)
	{
		float departure_time_difference = abs(g_passenger_departure_time_beginning[p1] - g_passenger_departure_time_beginning[p2]);
		float minimum_travel_time_p1_p2 = g_node_to_node_shorest_travel_time[g_passenger_origin_node[p1]][g_passenger_origin_node[p2]];
		float minimum_travel_time_p2_p1 = g_node_to_node_shorest_travel_time[g_passenger_origin_node[p2]][g_passenger_origin_node[p1]];

		if (departure_time_difference < min(minimum_travel_time_p1_p2, minimum_travel_time_p2_p1))
		{
			g_accessibility_matrix[p1][p2] = 0;
		}



	}

	// we now have g_node_to_node_shorest_travel_time from all activity node
	fprintf(g_pFileOutputLog, "Least Travel Time for Pax,");
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "Pax %d,", p);

	}
	fprintf(g_pFileOutputLog, "\n,");

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "%.2f,", g_passenger_request_travel_time_vector[p]);

	}
	fprintf(g_pFileOutputLog, "\n");
	fprintf(g_pFileOutputLog, "Modified Vehilce Time,begining, endinging\n");

	// modify the dummy vehicles' arrival time based on the least travel time for corresponding pax

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		int	v = (g_number_of_physical_vehicles + p); //dummy vehicle id

		g_external_vehicle_id_map[v] = std::to_string(v);

		g_vehicle_arrival_time_beginning[v] = min(g_number_of_time_intervals-1, g_passenger_departure_time_beginning[p] + g_passenger_request_travel_time_vector[p]+10); // +10 min as buffer
		g_vehicle_arrival_time_ending[v] = g_vehicle_arrival_time_beginning[v];
	/*	fprintf(g_pFileOutputLog, "%d,%d,%d\n",
			
		v,
		g_vehicle_departure_time_beginning[v],
		g_vehicle_arrival_time_ending[v]
		
		);*/


	}

	// calculate shortest path from each vehicle's origin to all nodes

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		cout << ">>find shortest path tree for vehicle v = " << v << "..." << endl;
		g_optimal_label_correcting(g_vehicle_origin_node[v], g_vehicle_departure_time_beginning[v]);

		for (int i = 1; i <= g_number_of_nodes; i++) //Initialization for all nodes
		{
			g_vehicle_origin_based_node_travel_time[v][i] = max(1, g_node_label_earliest_arrival_time[i] - g_vehicle_departure_time_beginning[v]);
		
		//	fprintf(g_pFileOutputLog, "V %d DN %d =,%f\n", v, i, g_vehicle_origin_based_node_travel_time[v][i]);

		
		}
		
	}
	// calculate shortest path from each vehicle's destination to all nodes

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		cout << ">>find shortest path tree for vehicle v = " << v << "..." << endl;
		g_optimal_label_correcting(g_vehicle_destination_node[v], 0);

		for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes
		{
			g_vehicle_destination_based_node_travel_time[v][i] = max(1, g_node_label_earliest_arrival_time[i]);
		//	fprintf(g_pFileOutputLog, "V %d ON %d =,%f\n", v, i, g_vehicle_destination_based_node_travel_time[v][i]);

		}

	}

	fprintf(g_pFileOutputLog, "Reduced Search Space Perc for Veh,");
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		fprintf(g_pFileOutputLog, "Veh %d,", v);

	}
	fprintf(g_pFileOutputLog, "\n,");


	// calculate the % of nodes can be skipped 
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		int number_of_nodes_to_be_skipped = 0;
		int time_window_length = max(10, g_vehicle_arrival_time_ending[v] - g_vehicle_departure_time_ending[v]);

		for (int i = 1; i <= g_number_of_nodes; i++)  // for each node
		{

			float travel_time_3_points = g_vehicle_origin_based_node_travel_time[v][i] + g_vehicle_destination_based_node_travel_time[v][i];

			// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
			if (travel_time_3_points >= time_window_length)
				number_of_nodes_to_be_skipped++;

		}
		fprintf(g_pFileOutputLog, "%.2f%%,", number_of_nodes_to_be_skipped*100.0 / max(1, g_number_of_nodes));

	}
	fprintf(g_pFileOutputLog, "\n,");

}

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// initialize MFC and print and error on failure
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: change error code to suit your needs
			_tprintf(_T("Fatal Error: MFC initialization failed\n"));
			nRetCode = 1;
		}
		else
		{
			// TODO: code your application's behavior here.
		}
	}
	else
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: GetModuleHandle failed\n"));
		nRetCode = 1;
	}

	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{ 
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}
	 g_pFileOutputLog = fopen("output_solution.csv", "w");
	 if (g_pFileOutputLog == NULL)
	 {
		 cout << "File output_solution.csv cannot be opened." << endl;
		 g_ProgramStop();
	 }
	 g_pFileAgentPathLog = fopen("agent_path.csv", "w");
	 if (g_pFileAgentPathLog == NULL)
	 {
		 cout << "File agent_path.csv cannot be opened." << endl;
		 g_ProgramStop();
	 }

	 fprintf(g_pFileAgentPathLog, "iteration_no,agent_id,agent_type,virtual_vehicle,path_node_sequence,path_time_sequence,path_state_sequence,\n"); // header
	 
	g_ReadConfiguration();
	g_ReadInputData();
	g_create_all_states(g_number_of_passengers, g_max_vehicle_capacity);

	g_allocate_memory();
	g_generate_travel_time_matrix();

	// definte timestamps
	clock_t start_t, end_t, total_t;
	int i;

	start_t = clock();
	g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables();

	end_t = clock();

	total_t = (end_t - start_t);


	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %ld milliseconds\n", total_t);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%ld, milliseconds\n", total_t);
	

	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);
	fclose(g_pFileAgentPathLog);
	

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	g_free_memory();
	cout << "done." << endl;



	return nRetCode;
}
