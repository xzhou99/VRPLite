// AgentPlus.cpp : Defines the entry point for the console application.
/* Copyright (C) 2014 Xuesong Zhou - All Rights Reserved*/
/* Copyright (C) 2015 Xuesong Zhou and Monirehalsadat Mahmoudi - All Rights Reserved*/
/*Contact Info: xzhou99@gmail.com, mahmoudi.monireh@gmail.com*/
/* this code is developed as part of research paper
Finding Optimal Solutions for Vehicle Routing Problem with Pickup and Delivery Services with Time Windows: A Dynamic Programming Approach Based on State-space-time Network Representations
Monirehalsadat Mahmoudi, Xuesong Zhou
http://arxiv.org/abs/1507.02731
*/
/*
AgentPlus Commercial License for OEMs, ISVs and VARs
The source code authors (Xuesong Zhou and Monirehalsadat Mahmoudi ) provides its AgentPlus Libraries under a dual license model designed 
to meet the development and distribution needs of both commercial distributors (such as OEMs, ISVs and VARs) and open source projects.

For OEMs, ISVs, VARs and Other Distributors of Commercial Applications:
OEMs (Original Equipment Manufacturers), ISVs (Independent Software Vendors), VARs (Value Added Resellers) and other distributors that
combine and distribute commercially licensed software with AgentPlus software and do not wish to distribute the source code for the commercially licensed software 
under version 2 of the GNU General Public License (the "GPL") must enter into a commercial license agreement with the source code authors.

For Open Source Projects and Other Developers of Open Source Applications:
For developers of Free Open Source Software ("FOSS") applications under the GPL that want to combine and distribute those FOSS applications with 
AgentPlus software, our AgentPlus open source software licensed under the GPL is the best option.

For developers and distributors of open source software under a FOSS license other than the GPL, we make the GPL-licensed AgentPlus Libraries available
under a FOSS Exception that enables use of the those AgentPlus Libraries under certain conditions without causing the entire derivative work to be subject to the GPL.
*/

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// The one and only application object
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "AgentPlus.h"
#include "CSVParser.h"

CWinApp theApp;
using namespace std;

FILE* g_pFileDebugLog = NULL;

FILE* g_pFileOutputLog = NULL;

FILE* g_pFileAgentPathLog = NULL;

int g_number_of_threads = 6;
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
int g_passenger_capacity[_MAX_NUMBER_OF_PASSENGERS] = { 1 };

int g_max_vehicle_capacity = 1;
int g_number_of_passengers = 0;

int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_link_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_path_state_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
float g_vehicle_path_cost_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
int g_vehicle_id_for_pax_being_served[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_best_solution_vehicle_id_for_pax_being_served[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_vehicle_serving_passenger_matrix[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };
float g_vehicle_serving_passenger_matrix_multiplier[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_y_vehicle_serving_passenger_matrix[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 1 };

float** g_vehicle_origin_based_node_travel_time = NULL;
float** g_vehicle_destination_based_node_travel_time = NULL;

extern float g_UpperBoundGeneration(int LR_Iteration_no);

// int -->string
void int2str(const int &int_temp, string &string_temp)
{
	stringstream stream;
	stream << int_temp;
	string_temp = stream.str();
}
// string --> int
int str2int(const string &string_temp)
{
	int a;
	stringstream stream(string_temp);
	stream >> a;
	return a;
}

class CVR_Carrying_State  //class for vehicle scheduling states
{
public:
	int passenger_carrying_state[_MAX_NUMBER_OF_PASSENGERS];
	int m_vehicle_capacity;//vehicle capacity already used

	CVR_Carrying_State()
	{
		m_vehicle_capacity = 1;
		for (int p = 0; p < _MAX_NUMBER_OF_PASSENGERS; p++)
			passenger_carrying_state[p] = 0;
	
	}

	std::vector<int> m_outgoing_state_index_vector;
	std::vector<int> m_outgoing_state_change_service_code_vector;//not known yet Sep 4 2015 by meng

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

std::map<std::string, int> g_carrying_state_map;  // hash table for mapping unique string key to the numerical state index w

int g_find_carrying_state_index(std::string string_key)
{
	if (g_carrying_state_map.find(string_key) != g_carrying_state_map.end())
	{
		return g_carrying_state_map[string_key];
	}
	else
	
		return -1;  // not found

}

std::vector<CVR_Carrying_State> g_VRStateCarryingVector;

void g_add_carrying_states(int parent_state_index, int number_of_passengers, int max_vehicle_capacity)
{

	CVR_Carrying_State element = g_VRStateCarryingVector[parent_state_index];

	g_VRStateCarryingVector[parent_state_index].m_outgoing_state_index_vector.push_back(parent_state_index);  // link my own state index to the parent state
	g_VRStateCarryingVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(0);  // link no change state index to the parent state


	for (int p = 1; p <= number_of_passengers; p++)
	{
		if (element.passenger_carrying_state[p] == 0) // not carrying p yet
		{ 
			// add pick up state 
			CVR_Carrying_State new_element;
			int pax_capacity_demand = 0;
			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states to create a new element 
			{
				new_element.passenger_carrying_state[pp] = element.passenger_carrying_state[pp];

				if (element.passenger_carrying_state[pp] == 1)
					pax_capacity_demand += g_passenger_capacity[pp];
			}


			new_element.m_vehicle_capacity = pax_capacity_demand + g_passenger_capacity[p];  // add pax p

			if (new_element.m_vehicle_capacity <= max_vehicle_capacity)
			{

				// test capacity 
				new_element.passenger_carrying_state[p] = 1;  // from 0 to 1

				std::string string_key = new_element.generate_string_key();
				int state_index = g_find_carrying_state_index(string_key);  // find if the newly generated state node has been defined already
				if (state_index == -1) // no defined yet
				{
					// add new state
					state_index = g_VRStateCarryingVector.size();
					g_VRStateCarryingVector.push_back(new_element);
					g_carrying_state_map[string_key] = state_index;
				}// otherwise do nother

				g_VRStateCarryingVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state in the state transtion graph
				g_VRStateCarryingVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(p);  // identify the new element is generated due to passenger p
				
			}
			else
			{ // do  nothing, so we do not add this node into our state vector
				//cout << " capacity for new node =  " << new_element.m_vehicle_capacity << endl ;
			}
		}
		else  // ==1 carried
		{
			// add delivery state
			CVR_Carrying_State new_element;
			int pax_capacity_demand = 0;

			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states
			{
				new_element.passenger_carrying_state[pp] = element.passenger_carrying_state[pp];

				if (element.passenger_carrying_state[pp] == 1)
					pax_capacity_demand += g_passenger_capacity[pp];
			}
			new_element.passenger_carrying_state[p] = 0;  // from 1 to 0
			new_element.m_vehicle_capacity = pax_capacity_demand - g_passenger_capacity[p];

			std::string string_key = new_element.generate_string_key();
			int state_index = g_find_carrying_state_index(string_key);  // find if the newly generated state node has been defined already
			if (state_index == -1)  // no
			{
				// add new state
				state_index = g_VRStateCarryingVector.size();
				g_VRStateCarryingVector.push_back(new_element);
				g_carrying_state_map[string_key] = state_index;
			}  //otherwise, do nothing

			g_VRStateCarryingVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state
			g_VRStateCarryingVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back((-1)*p);  // link new state index to the parent state
		}

	}	
}


class OutgoingCarryingState
{
public:
	std::vector<int> m_w2_vector;
};

OutgoingCarryingState g_outgoingCarryingStateSet[_MAX_NUMBER_OF_STATES];

// defintion of passenger service states
class CVR_Service_State  //class for vehicle scheduling service states
{
public:
	int passenger_service_state[_MAX_NUMBER_OF_PASSENGERS]; // code is 0, 1 or 2
	int m_passenger_occupancy;//total number of passengers in a vehicle
	int m_boundary_state_flag;

	CVR_Service_State()
	{
		m_boundary_state_flag = 1;  // true
		m_passenger_occupancy = 0;
		for (int p = 0; p < _MAX_NUMBER_OF_PASSENGERS; p++)
			passenger_service_state[p] = 0;

	}

	void SetAllServedStateNode()
	{
		m_boundary_state_flag = 1;  // true
		for (int p = 1; p <= g_number_of_passengers; p++)
			passenger_service_state[p] = 2;  // all served

		m_passenger_occupancy = 0;

		generate_string_key();
	
	}
	std::vector<int> m_outgoing_state_index_vector;
	std::vector<int> m_outgoing_state_change_service_code_vector;//not known yet Sep 4 2015 by meng

	bool IsBoundaryState()
	{
	
		if (m_boundary_state_flag == 1)
			return true;
		else
			return false;
	}
	std::string generate_string_key()
	{
		std::string string_key;
		for (int p = 1; p <= g_number_of_passengers; p++)  // scan all passengers
		{

			stringstream s;

			s << "_";
			if (passenger_service_state[p] == 1)
			{
				m_boundary_state_flag = 0;  // if any of passenger service code is 1, then this state is not a boundary state
				s << p;
			}
			else if (passenger_service_state[p] == 2)  //complete the trip for this passenger p
			{
				s << p;
				s << "*";
			}
			else
			{
				s << " ";
			}

			string converted(s.str());

			string_key += converted;

		}
		return string_key;  //e.g. _ _ _ or _1_2_3 or or _1*_2*_3* or _1_2_3*
	}
};

std::map<std::string, int> g_service_state_map;  // hash table for mapping unique string key to the numerical state index s

int g_find_service_state_index(std::string string_key)
{
	if (g_service_state_map.find(string_key) != g_service_state_map.end())
	{
		return g_service_state_map[string_key];
	}
	else

		return -1;  // not found

}

std::vector<CVR_Service_State> g_VRStateServiceVector;

void g_add_service_states(int parent_state_index, int number_of_passengers)
{

	CVR_Service_State element = g_VRStateServiceVector[parent_state_index];

	g_VRStateServiceVector[parent_state_index].m_outgoing_state_index_vector.push_back(parent_state_index);  // link my own state index to the parent state
	g_VRStateServiceVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(0);  // link no change state index to the parent state


	for (int p = 1; p <= number_of_passengers; p++)
	{
		if (element.passenger_service_state[p] == 0) // not carrying p yet
		{
			// add pick up state 
			CVR_Service_State new_element;
			int pax_capacity_demand = 0;
			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states to create a new element 
			{
				new_element.passenger_service_state[pp] = element.passenger_service_state[pp];

				if (element.passenger_service_state[pp] == 1)
					pax_capacity_demand += g_passenger_capacity[pp];
			}


			new_element.m_passenger_occupancy = pax_capacity_demand + g_passenger_capacity[p];  // add pax p


				// test capacity 
			new_element.passenger_service_state[p] = 1;  // from 0 to 1

				std::string string_key = new_element.generate_string_key();
				int state_index = g_find_service_state_index(string_key);  // find if the newly generated state node has been defined already
				if (state_index == -1) // no defined yet
				{
					// add new state
					state_index = g_VRStateServiceVector.size();
					g_VRStateServiceVector.push_back(new_element);
					g_service_state_map[string_key] = state_index;
				}// otherwise do nother

				g_VRStateServiceVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state in the state transtion graph
				g_VRStateServiceVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back(p);  // identify the new element is generated due to passenger p

		}
		else  if (element.passenger_service_state[p] == 1) //  ==1 carried
		{
			// add delivery and completition state
			CVR_Service_State new_element;
			int pax_capacity_demand = 0;

			for (int pp = 1; pp <= number_of_passengers; pp++)  // copy vector states
			{
				new_element.passenger_service_state[pp] = element.passenger_service_state[pp];

				if (element.passenger_service_state[pp] == 1)
					pax_capacity_demand += g_passenger_capacity[pp];
			}
			new_element.passenger_service_state[p] = 2;  // from 1 to 2
			new_element.m_passenger_occupancy = pax_capacity_demand - g_passenger_capacity[p];

			std::string string_key = new_element.generate_string_key();
			int state_index = g_find_service_state_index(string_key);  // find if the newly generated state node has been defined already
			if (state_index == -1)  // no
			{
				// add new state
				state_index = g_VRStateServiceVector.size();
				g_VRStateServiceVector.push_back(new_element);
				g_service_state_map[string_key] = state_index;
			}  //otherwise, do nothing

			g_VRStateServiceVector[parent_state_index].m_outgoing_state_index_vector.push_back(state_index);  // link new state index to the parent state
			g_VRStateServiceVector[parent_state_index].m_outgoing_state_change_service_code_vector.push_back((-1)*p);  // link new state index to the parent state
		}

	}
}


class OutgoingServiceState
{
public:
	std::vector<int> m_w2_vector;
};

OutgoingServiceState g_outgoingServiceStateSet[_MAX_NUMBER_OF_STATES];

// end of definition for service states


int g_outbound_node_size[_MAX_NUMBER_OF_NODES] = { 0 };
int g_node_passenger_id[_MAX_NUMBER_OF_NODES] = { -1 };
int g_node_passenger_pickup_flag[_MAX_NUMBER_OF_NODES] = { 0 };

int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_activity_node_flag[_MAX_NUMBER_OF_NODES] = { 0 };
int g_activity_node_ending_time[_MAX_NUMBER_OF_NODES] = { 99999 };
int g_activity_node_starting_time[_MAX_NUMBER_OF_NODES] = { 99999 };

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_INBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_INBOUND_NODES];


int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
float g_link_free_flow_travel_time_float_value[_MAX_NUMBER_OF_LINKS];

float g_link_link_length[_MAX_NUMBER_OF_LINKS];
int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
float g_link_jam_density[_MAX_NUMBER_OF_LINKS];
int g_link_service_code[_MAX_NUMBER_OF_LINKS] = { 0 };


float g_link_speed[_MAX_NUMBER_OF_LINKS];
int g_link_from_node_id[_MAX_NUMBER_OF_LINKS];
int g_link_to_node_id[_MAX_NUMBER_OF_LINKS];


float g_VOIVTT_per_hour[_MAX_NUMBER_OF_VEHICLES];
float g_VOWT_per_hour[_MAX_NUMBER_OF_VEHICLES];






int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

float g_vehicle_path_cost[_MAX_NUMBER_OF_VEHICLES] = { 0 };  // for vehcile routings

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
int g_passenger_dummy_destination_node[_MAX_NUMBER_OF_PASSENGERS] = { -1 };


int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_PASSENGERS];
float g_lamda_vehicle_passenger_assignment_multiplier[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };


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
float g_path_travel_time[_MAX_NUMBER_OF_VEHICLES] = { 0 };


int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_physical_nodes = 0;
int g_maximum_number_name_of_nodes = 0;

int g_number_of_time_intervals = 10;



int g_number_of_vehicles = 0;
int g_number_of_physical_vehicles = 0;


int g_number_of_toll_records = 0;

int g_number_of_LR_iterations = 1;
int g_minimum_subgradient_step_size = 1;

int g_shortest_path_debugging_flag = 1;
float g_waiting_time_ratio = 0.005;
float g_dummy_vehicle_cost_per_hour = 100;

bool g_no_changed_route_for_passengers_flag = false;
bool g_no_capacity_multiplier_flag = false;

float g_travel_time_budget = 100;
float g_idle_vehicle_benefit = -10;


void g_create_all_carrying_states(int number_of_passengers = 10, int max_vehicle_capacity = 1)
{
	CVR_Carrying_State route_element; // 0000000000 restricted in the construction function by setting passenger_carrying_state = 0 for each passenger

	std::string string_key = route_element.generate_string_key();

	g_carrying_state_map[string_key] = 0;// 0 is the root node index
	g_VRStateCarryingVector.push_back(route_element);

	int scan_state_index = 0;
	while (g_VRStateCarryingVector.size() < _MAX_NUMBER_OF_STATES && scan_state_index< g_VRStateCarryingVector.size() && scan_state_index< _MAX_NUMBER_OF_STATES)
	{
		g_add_carrying_states(scan_state_index, number_of_passengers, max_vehicle_capacity);
		scan_state_index++;
	}

	// print out 
	for (int i = 0; i < g_VRStateCarryingVector.size(); i++)
	{
		std::string str = g_VRStateCarryingVector[i].generate_string_key();
	
		fprintf(g_pFileDebugLog, "carrying state no. %d:(%d) %s; outgoing state list:", i, g_VRStateCarryingVector[i].m_vehicle_capacity, str.c_str());

		for (int w2 = 0; w2 < g_VRStateCarryingVector[i].m_outgoing_state_index_vector.size(); w2++)
		{
			fprintf(g_pFileDebugLog, "%d,", g_VRStateCarryingVector[i].m_outgoing_state_index_vector[w2]);
		}

		fprintf(g_pFileDebugLog, "\n");

	}

	fprintf(g_pFileDebugLog, "-----\n");



}

void g_create_all_service_states(int number_of_passengers = 2)
{
	CVR_Service_State route_element; // 0000000000 restricted in the construction function by setting passenger_service_state = 0 for each passenger

	std::string string_key = route_element.generate_string_key();

	g_service_state_map[string_key] = 0;// 0 is the root node index
	g_VRStateServiceVector.push_back(route_element);


	CVR_Service_State route_element1; // 2222222 restricted in the construction function by setting passenger_service_state = 2 for each passenger
	route_element1.SetAllServedStateNode();
	string_key = route_element1.generate_string_key();

	g_service_state_map[string_key] = 1;// 1 is the second node index
	g_VRStateServiceVector.push_back(route_element1);

	

	int scan_state_index = 0;
	while (g_VRStateServiceVector.size() < _MAX_NUMBER_OF_STATES && scan_state_index< g_VRStateServiceVector.size() && scan_state_index< _MAX_NUMBER_OF_STATES)
	{
		g_add_service_states(scan_state_index, number_of_passengers);
		scan_state_index++;
	}

	// print out 
	for (int i = 0; i < g_VRStateServiceVector.size(); i++)
	{
		std::string str = g_VRStateServiceVector[i].generate_string_key();

		fprintf(g_pFileDebugLog, "service state no. %d:(%d) %s; outgoing state list:", i, g_VRStateServiceVector[i].m_passenger_occupancy, str.c_str());

		for (int w2 = 0; w2 < g_VRStateServiceVector[i].m_outgoing_state_index_vector.size(); w2++)
		{
			fprintf(g_pFileDebugLog, "%d,", g_VRStateServiceVector[i].m_outgoing_state_index_vector[w2]);
		}

		fprintf(g_pFileDebugLog, "\n");

	}

	fprintf(g_pFileDebugLog, "-----\n");



}
int g_add_new_node(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_internal_id = g_number_of_nodes;
	int new_node_external_number = g_maximum_number_name_of_nodes + 1;

	g_outbound_node_size[new_node_internal_id] = 0;
	g_inbound_node_size[new_node_internal_id] = 0;
	g_node_passenger_id[new_node_internal_id] = passenger_id;
	g_activity_node_flag[new_node_internal_id] = 1;
	g_activity_node_starting_time[new_node_internal_id] = beginning_time;
	g_activity_node_ending_time[new_node_internal_id] = end_time;
	
	g_internal_node_no_map[new_node_external_number] = new_node_internal_id;
	g_external_node_id_map[new_node_internal_id] = new_node_external_number;

	g_number_of_nodes++;
	g_maximum_number_name_of_nodes++;


	return new_node_internal_id;
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


	g_link_from_node_id[new_link_id] = from_node_id;
	g_link_to_node_id[new_link_id] = to_node_id;

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

float g_arc_travel_time[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };

float g_to_node_cost[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };  /// this is lamda

//float g_arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
//float g_to_node_cost_used_for_upper_bound[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };
//float g_vertex_waiting_cost[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };


int g_vertex_visit_count_for_lower_bound[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };  // used for lower bound

int g_vertex_visit_count_for_upper_bound[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS] = { 0 };  // used for upper bound




// label cost for each LC(i,t,p,c) 

float*** l_state_node_label_cost = NULL;
int*** l_state_node_predecessor = NULL;
int*** l_state_time_predecessor = NULL;
int*** l_state_carrying_predecessor = NULL;

//parallel computing 
float**** lp_state_node_label_cost = NULL;
int**** lp_state_node_predecessor = NULL;
int**** lp_state_time_predecessor = NULL;
int**** lp_state_carrying_predecessor = NULL;

//state vertex label cost: la as local assignment variables 
float**** la_state_node_label_cost = NULL;

int**** la_state_node_predecessor = NULL;
int**** la_state_time_predecessor = NULL;
int**** la_state_service_predecessor = NULL;
int**** la_state_vehicle_predecessor = NULL;


float** g_node_to_node_shorest_travel_time = NULL;


float*** g_v_arc_cost = NULL;
float*** g_v_to_node_cost_used_for_upper_bound = NULL;
float*** g_v_vertex_waiting_cost = NULL;


void g_allocate_memory_DP(int number_of_processors)
{

	int number_of_states = g_VRStateCarryingVector.size() + 1;
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_links = g_number_of_links + 1;
	int number_of_time_intervals = g_number_of_time_intervals + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;


	g_v_arc_cost = Allocate3DDynamicArray<float>(number_of_vehicles, number_of_links, number_of_time_intervals);
	g_v_to_node_cost_used_for_upper_bound = Allocate3DDynamicArray<float>(number_of_vehicles, number_of_nodes, number_of_time_intervals);
	g_v_vertex_waiting_cost = Allocate3DDynamicArray<float>(number_of_vehicles, number_of_nodes, number_of_time_intervals);



	if (g_number_of_time_intervals >= _MAX_NUMBER_OF_TIME_INTERVALS)
	{
		cout << "Program configuration issue: _MAX_NUMBER_OF_TIME_INTERVALS= " << _MAX_NUMBER_OF_TIME_INTERVALS << "is less than g_number_of_time_intervals = " << g_number_of_time_intervals << endl;
		cout << "Please contact developer." << endl;
		g_ProgramStop();
	}

	cout << "number of states = " << g_VRStateCarryingVector.size() << endl; 


	l_state_node_label_cost = Allocate3DDynamicArray<float>(number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_node_predecessor = Allocate3DDynamicArray<int>( number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_time_predecessor = Allocate3DDynamicArray<int>(number_of_nodes, number_of_time_intervals, number_of_states);
	l_state_carrying_predecessor = Allocate3DDynamicArray<int>( number_of_nodes, number_of_time_intervals, number_of_states);

	number_of_processors = g_number_of_threads;
	
	lp_state_node_label_cost = Allocate4DDynamicArray<float>(number_of_processors,number_of_nodes, number_of_time_intervals, number_of_states);
	lp_state_node_predecessor = Allocate4DDynamicArray<int>(number_of_processors, number_of_nodes, number_of_time_intervals, number_of_states);
	lp_state_time_predecessor = Allocate4DDynamicArray<int>(number_of_processors, number_of_nodes, number_of_time_intervals, number_of_states);
	lp_state_carrying_predecessor = Allocate4DDynamicArray<int>(number_of_processors, number_of_nodes, number_of_time_intervals, number_of_states);

	g_node_to_node_shorest_travel_time = AllocateDynamicArray<float>(number_of_nodes, number_of_nodes, 999);

	number_of_vehicles = g_number_of_vehicles+1;
	number_of_states = g_VRStateServiceVector.size() + 1;

	la_state_node_label_cost = Allocate4DDynamicArray<float>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);
	la_state_node_predecessor = Allocate4DDynamicArray<int>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);
	la_state_time_predecessor = Allocate4DDynamicArray<int>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);
	la_state_service_predecessor = Allocate4DDynamicArray<int>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);
	la_state_vehicle_predecessor = Allocate4DDynamicArray<int>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);


}
void g_allocate_memory_travel_time(int number_of_processors)
{
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;

	g_node_to_node_shorest_travel_time = AllocateDynamicArray<float>(number_of_nodes, number_of_nodes, 999);
	g_vehicle_origin_based_node_travel_time = AllocateDynamicArray<float>(number_of_vehicles, number_of_nodes, 0);
	g_vehicle_destination_based_node_travel_time = AllocateDynamicArray<float>(number_of_vehicles, number_of_nodes, 0);

}

void g_free_memory_travel_time(int number_of_processors)
{
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;

	exit(0);  // replying on OS to release memory

	DeallocateDynamicArray<float>(g_node_to_node_shorest_travel_time, number_of_nodes, number_of_nodes);
	DeallocateDynamicArray<float>(g_vehicle_origin_based_node_travel_time, number_of_vehicles, number_of_nodes);
	DeallocateDynamicArray<float>(g_vehicle_destination_based_node_travel_time, number_of_vehicles, number_of_nodes);

}

void g_free_memory_DP(int number_of_processors)
{
	int number_of_states = g_VRStateCarryingVector.size();
	int number_of_nodes = g_number_of_nodes + 1;
	int number_of_links = g_number_of_links + 1;
	int number_of_time_intervals = g_number_of_time_intervals + 1;
	int number_of_vehicles = g_number_of_vehicles + 1;

	Deallocate3DDynamicArray<float>(l_state_node_label_cost,  number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_node_predecessor,  number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_time_predecessor, number_of_nodes, number_of_time_intervals);
	Deallocate3DDynamicArray<int>(l_state_carrying_predecessor, number_of_nodes, number_of_time_intervals);
	
	Deallocate3DDynamicArray<float>(g_v_arc_cost, number_of_vehicles, number_of_links);
	Deallocate3DDynamicArray<float>(g_v_to_node_cost_used_for_upper_bound, number_of_vehicles, number_of_nodes);
	Deallocate3DDynamicArray<float>(g_v_vertex_waiting_cost, number_of_vehicles, number_of_nodes);

	number_of_processors = g_number_of_threads;

	Deallocate4DDynamicArray<float>(lp_state_node_label_cost, number_of_processors, number_of_nodes, number_of_time_intervals);
	Deallocate4DDynamicArray<int>(lp_state_node_predecessor, number_of_processors, number_of_nodes, number_of_time_intervals);
	Deallocate4DDynamicArray<int>(lp_state_time_predecessor, number_of_processors,  number_of_nodes, number_of_time_intervals);
	Deallocate4DDynamicArray<int>(lp_state_carrying_predecessor, number_of_processors, number_of_nodes, number_of_time_intervals);


	DeallocateDynamicArray<float>(g_node_to_node_shorest_travel_time, number_of_nodes, number_of_nodes);
	

	number_of_vehicles = g_number_of_vehicles + 1;
	number_of_states = g_VRStateServiceVector.size() + 1;

	la_state_node_label_cost = Allocate4DDynamicArray<float>(number_of_vehicles, number_of_time_intervals, number_of_states, number_of_nodes);

	Deallocate4DDynamicArray<float>(la_state_node_label_cost, number_of_vehicles, number_of_time_intervals, number_of_states);
	Deallocate4DDynamicArray<int>(la_state_node_predecessor, number_of_vehicles, number_of_time_intervals, number_of_states);
	Deallocate4DDynamicArray<int>(la_state_time_predecessor, number_of_vehicles, number_of_time_intervals, number_of_states);
	Deallocate4DDynamicArray<int>(la_state_service_predecessor, number_of_vehicles, number_of_time_intervals, number_of_states);
	Deallocate4DDynamicArray<int>(la_state_vehicle_predecessor, number_of_vehicles, number_of_time_intervals, number_of_states);


}

// for non service link: one element: w2 = w1 for all possible stages
// for pick up link: one element: w2= w1 + the passenger of upstream node, p

// infeasible, p already in w1 or w1 is full at capacity

// for delivery link: one element: w2= w1 - the passenger of downstream node
//infeasible: if p is inc



//parallel computing version
float g_pc_optimal_time_dependenet_dynamic_programming(
	int p,
	int vehicle_id,
	int origin_node, 
	int departure_time_beginning, 
	int departure_time_ending, 
	int destination_node, 
	int arrival_time_beginning, 
	int arrival_time_ending,
	int &path_number_of_nodes,
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	float path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	int vehicle_capacity,
	float &travel_time_return_value,
	bool bUpperBoundFlag)
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

			for (int w = 0; w < g_VRStateCarryingVector.size(); w++)
			{
				lp_state_node_label_cost[p][i][t][w] = _MAX_LABEL_COST;
				lp_state_node_predecessor[p][i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
				lp_state_time_predecessor[p][i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
				lp_state_carrying_predecessor[p][i][t][w] = -1;
			}
		}
	}

	//step 2: Initialization for origin node at the preferred departure time, at departure time

	int w0 = 0;  // start fro empty

	lp_state_node_label_cost[p][origin_node][departure_time_beginning][w0] = 0;

	// step 3: //dynamic programming
	for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
	{
		if (t % 100== 0)
		{
			if (bUpperBoundFlag ==true)
			cout << "CPU " << p << ": UB: vehicle " << vehicle_id << " is scanning time " << t << "..." << endl;
			else
				cout << "CPU " << p << ": LB: vehicle " << vehicle_id << " is scanning time " << t << "..." << endl;

		}
		for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
		{
			int from_node = g_link_from_node_id[link];

			int to_node = g_link_to_node_id[link];


			//// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
			//float travel_time_3_points = g_vehicle_origin_based_node_travel_time[vehicle_id][from_node] + g_vehicle_destination_based_node_travel_time[vehicle_id][from_node];
			//int time_window_length = g_vehicle_arrival_time_ending[vehicle_id] - g_vehicle_departure_time_ending[vehicle_id];

			//// if the total travel time from origin to node i then back to destination is greater than the time window, then skip this node/link to scan
			//if (travel_time_3_points >= time_window_length)
			//	continue;  //skip


			int upstream_p = g_node_passenger_id[from_node];
			int downsteram_p = g_node_passenger_id[to_node];

			int travel_time = g_link_free_flow_travel_time[link];

			for (int w1 = 0; w1 < g_VRStateCarryingVector.size(); w1++)
			{

				if (g_VRStateCarryingVector[w1].m_vehicle_capacity > vehicle_capacity) // skip all states exceeding vehicle capacity
					continue;

				if (vehicle_id > g_number_of_physical_vehicles) // virtual vehicle
				{
					int pax_id = vehicle_id - g_number_of_physical_vehicles;  // the corresponding pax for virtual vehicle
					bool bFeasibleStateFlag = false;
					if (w1 == 0 || w1 == pax_id)
						bFeasibleStateFlag = true;
					//skip this state

					if (bFeasibleStateFlag == false)
						continue; //skip this state
					else
					{
						// continue the search process
					}
				}

				if (lp_state_node_label_cost[p][from_node][t][w1] < _MAX_LABEL_COST - 1)  // for feasible time-space point only
				{


					for (int w2_index = 0; w2_index < g_VRStateCarryingVector[w1].m_outgoing_state_index_vector.size(); w2_index++)
					{
						if (g_VRStateCarryingVector[w1].m_outgoing_state_change_service_code_vector[w2_index] != g_link_service_code[link])  //0,  +p or -p
							continue;

						if (g_link_service_code[link] != 0)
							TRACE("service_link!");


						int w2 = g_VRStateCarryingVector[w1].m_outgoing_state_index_vector[w2_index];

						if (g_VRStateCarryingVector[w2].m_vehicle_capacity > vehicle_capacity)
							continue;

						if (vehicle_id > g_number_of_physical_vehicles) // virtual vehicle
						{
							int pax_id = vehicle_id - g_number_of_physical_vehicles;  // the corresponding pax for virtual vehicle
							bool bFeasibleStateFlag = false;
							if (w2 == 0 || w2 == pax_id)
								bFeasibleStateFlag = true;
							//skip this state

							if (bFeasibleStateFlag == false)
								continue; //skip this state
							else
							{
								// continue the search process
							}
						}
						// part 1: link based update
						int new_to_node_arrival_time = min(t + travel_time, g_number_of_time_intervals - 1);

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



						float to_node_cost = 0;

						if (bUpperBoundFlag == false)
							to_node_cost = g_to_node_cost[vehicle_id][to_node][new_to_node_arrival_time];
						else // when we consider upper bound and virtural vehicle
							to_node_cost = g_v_to_node_cost_used_for_upper_bound[vehicle_id][to_node][new_to_node_arrival_time];



						//					if (g_shortest_path_debugging_flag)
						//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
						//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
						float temporary_label_cost = lp_state_node_label_cost[p][from_node][t][w1] + g_v_arc_cost[vehicle_id][link][t] + to_node_cost;

						if (g_link_service_code[link] != 0 && w1 == 1 && w2 == 0 && temporary_label_cost <-0.1)
							TRACE("delivery link!");


						if (temporary_label_cost < lp_state_node_label_cost[p][to_node][new_to_node_arrival_time][w2]) // we only compare cost at the downstream node ToID at the new arrival time t
						{

							if (g_shortest_path_debugging_flag)
							{
								fprintf(g_pFileDebugLog, "DP: updating node: %d from time %d to time %d, current cost: %.2f, from cost %.2f ->%.2f\n",
									to_node, t, new_to_node_arrival_time,
									lp_state_node_label_cost[p][from_node][t][w2],
									lp_state_node_label_cost[p][to_node][new_to_node_arrival_time][w2], temporary_label_cost);
							}

							// update cost label and node/time predecessor

							lp_state_node_label_cost[p][to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
							lp_state_node_predecessor[p][to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
							lp_state_time_predecessor[p][to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
							lp_state_carrying_predecessor[p][to_node][new_to_node_arrival_time][w2] = w1;
						}
						// part 2: same node based update for waiting arcs

						if (w2 == w1) // for the same state
						{

							new_to_node_arrival_time = min(t + 1, g_number_of_time_intervals - 1);

							//					if (g_shortest_path_debugging_flag)
							//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
							//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
							temporary_label_cost = lp_state_node_label_cost[p][from_node][t][w1] + g_v_vertex_waiting_cost[vehicle_id][from_node][t];

							if (vehicle_id == 1 && bUpperBoundFlag && from_node == 673 && t >=64)
							{
								TRACE("waiting cost: %f, ", g_v_vertex_waiting_cost[vehicle_id][673][64]);
							}
								
							if (temporary_label_cost < lp_state_node_label_cost[p][from_node][new_to_node_arrival_time][w1]) // we only compare cost at the downstream node ToID at the new arrival time t
							{

	

								// update cost label and node/time predecessor

								lp_state_node_label_cost[p][from_node][new_to_node_arrival_time][w1] = temporary_label_cost;
								lp_state_node_predecessor[p][from_node][new_to_node_arrival_time][w1] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								lp_state_time_predecessor[p][from_node][new_to_node_arrival_time][w1] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								lp_state_carrying_predecessor[p][from_node][new_to_node_arrival_time][w1] = w1;
							}
						}

					}
				}  // feasible vertex label cost
			}  // for all states

		} // for all link
	} // for all time t


	
	total_cost = _MAX_LABEL_COST;

	int min_cost_time_index = arrival_time_ending;

	int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	int reversed_path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];
	float reversed_path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS];

	int w = 0;
	total_cost = lp_state_node_label_cost[p][destination_node][min_cost_time_index][w];

	// step 2: backtrack to the origin (based on node and time predecessors)
	int	node_size = 0;
	reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
	reversed_path_time_sequence[node_size] = min_cost_time_index;
	reversed_path_state_sequence[node_size] = w;
	reversed_path_cost_sequence[node_size] = lp_state_node_label_cost[p][destination_node][min_cost_time_index][w];


	node_size++;

	int pred_node = lp_state_node_predecessor[p][destination_node][min_cost_time_index][w];
	int pred_time = lp_state_time_predecessor[p][destination_node][min_cost_time_index][w];
	int pred_state = lp_state_carrying_predecessor[p][destination_node][min_cost_time_index][w];

	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
	{
		reversed_path_node_sequence[node_size] = pred_node;
		reversed_path_time_sequence[node_size] = pred_time;
		reversed_path_state_sequence[node_size] = pred_state;
		reversed_path_cost_sequence[node_size] = lp_state_node_label_cost[p][pred_node][pred_time][pred_state];

		node_size++;

		//record current values of node and time predecessors, and update PredNode and PredTime

		int pred_node_record = pred_node;
		int pred_time_record = pred_time;
		int pred_state_record = pred_state;

		pred_node = lp_state_node_predecessor[p][pred_node_record][pred_time_record][pred_state_record];
		pred_time = lp_state_time_predecessor[p][pred_node_record][pred_time_record][pred_state_record];
		pred_state = lp_state_carrying_predecessor[p][pred_node_record][pred_time_record][pred_state_record];

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


//parallel computing version
float g_integrated_assignment_routing_dynamic_programming()
{

	// step 1: Initialization for all nodes

	for (int v = 0; v <= g_number_of_vehicles; v++) //Initialization for all nodes
	{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				for (int s = 0; s < g_VRStateServiceVector.size(); s++)
				{
					for (int i = 0; i <= g_number_of_nodes; i++) //Initialization for all nodes


					{
						la_state_node_label_cost[v][t][s][i] = _MAX_LABEL_COST;
						la_state_node_predecessor[v][t][s][i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
						la_state_time_predecessor[v][t][s][i] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
						la_state_service_predecessor[v][t][s][i] = -1;
						la_state_vehicle_predecessor[v][t][s][i] = -1;
					}
				}
			}
		
	}

	//step 2: Initialization for origin node at the preferred departure time, at departure time



	int s0 = 0;  // start from empty
	int origin_node = g_vehicle_origin_node[1];
	int departure_time_beginning = g_vehicle_departure_time_beginning[1];

	la_state_node_label_cost[1][departure_time_beginning][s0][origin_node] = 0;

	for (int v = 1; v <= g_number_of_physical_vehicles; v++) // 1st loop for vehicle
	{
		int arrival_time_ending = g_vehicle_arrival_time_ending[v];
		departure_time_beginning = g_vehicle_departure_time_beginning[v];
		int vehicle_capacity = g_vehicle_capacity[v];


		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //2nd loop: time
		{
			if (t % 100 == 0)
			{
					cout << " vehicle " << v << " is scanning time " << t << "..." << endl;

			}

			for (int s1 = 0; s1 < g_VRStateServiceVector.size(); s1++)  // 3rd loop, service state
			{

				if (g_VRStateServiceVector[s1].m_passenger_occupancy > vehicle_capacity) // skip all states exceeding vehicle capacity
					continue;


			for (int link = 0; link < g_number_of_links; link++)  // 4th loop for each link (i,j)
			{
				int from_node = g_link_from_node_id[link];  //i

				int to_node = g_link_to_node_id[link];  // j


				int upstream_p = g_node_passenger_id[from_node];
				int downsteram_p = g_node_passenger_id[to_node];

				int travel_time = g_link_free_flow_travel_time[link];


				if (la_state_node_label_cost[v][t][s1][from_node] < _MAX_LABEL_COST - 1)  // for feasible time-space point only
					{


						for (int s2_index = 0; s2_index < g_VRStateServiceVector[s1].m_outgoing_state_index_vector.size(); s2_index++)
						{
							if (g_VRStateServiceVector[s1].m_outgoing_state_change_service_code_vector[s2_index] != g_link_service_code[link])  //0,  +p or -p
								continue;

							if (g_link_service_code[link] != 0)
								TRACE("service_link!");


							int s2 = g_VRStateServiceVector[s1].m_outgoing_state_index_vector[s2_index];

							if (g_VRStateServiceVector[s2].m_passenger_occupancy > vehicle_capacity)
								continue;

							// part 1: link based update
							int new_to_node_arrival_time = min(t + travel_time, g_number_of_time_intervals - 1);

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




							//					if (g_shortest_path_debugging_flag)
							//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
							//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
							float temporary_label_cost = la_state_node_label_cost[v][t][s1][from_node] + g_v_arc_cost[v][link][t];

							if (temporary_label_cost < la_state_node_label_cost[v][new_to_node_arrival_time][s2][to_node]) // we only compare cost at the downstream node ToID at the new arrival time t
							{

								//if (g_shortest_path_debugging_flag)
								//{
								//	fprintf(g_pFileDebugLog, "DP: updating node: %d from time %d to time %d, current cost: %.2f, from cost %.2f ->%.2f\n",
								//		to_node, t, new_to_node_arrival_time,
								//		lp_state_node_label_cost[p][from_node][t][w2],
								//		lp_state_node_label_cost[p][to_node][new_to_node_arrival_time][w2], temporary_label_cost);
								//}

								// update cost label and node/time predecessor

								la_state_node_label_cost[v][new_to_node_arrival_time][s2][to_node] = temporary_label_cost;
								la_state_node_predecessor[v][new_to_node_arrival_time][s2][to_node] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								la_state_time_predecessor[v][new_to_node_arrival_time][s2][to_node] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								la_state_service_predecessor[v][new_to_node_arrival_time][s2][to_node] = s1;
								la_state_vehicle_predecessor[v][new_to_node_arrival_time][s2][to_node] = v;
							}
							// part 2: same node based update for waiting arcs

							if (s2 == s1) // for the same state
							{

								new_to_node_arrival_time = min(t + 1, g_number_of_time_intervals - 1);

								//					if (g_shortest_path_debugging_flag)
								//						fprintf(g_pFileDebugLog, "SP: checking from node %d, to node %d at time %d, FFTT = %d\n",
								//					from_node, to_node, new_to_node_arrival_time,  g_link_free_flow_travel_time[link_no]);
								 temporary_label_cost = la_state_node_label_cost[v][t][s1][from_node] + +g_v_vertex_waiting_cost[v][from_node][t];

	
								 if (temporary_label_cost < la_state_node_label_cost[v][new_to_node_arrival_time][s1][from_node]) // we only compare cost at the downstream node ToID at the new arrival time t
								{



									// update cost label and node/time predecessor

									 la_state_node_label_cost[v][new_to_node_arrival_time][s1][from_node] = temporary_label_cost;
									 la_state_node_predecessor[v][new_to_node_arrival_time][s1][from_node] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
									 la_state_time_predecessor[v][new_to_node_arrival_time][s1][from_node] = t;  // pointer to previous TIME INDEX from the current label at current node and time
									 la_state_service_predecessor[v][new_to_node_arrival_time][s1][from_node] = s1;
									 la_state_vehicle_predecessor[v][new_to_node_arrival_time][s1][from_node] = v;
								 }
							}

						}
					}  // feasible vertex label cost
				}  // for all links

			} // for all states
			} // for all time t

			// RACE relay section from v to v+1

		if (v < g_number_of_vehicles)
		{
			for (int s = 0; s < g_VRStateServiceVector.size(); s++)  //  service state
			{
			// boundary states only
				if (g_VRStateServiceVector[s].IsBoundaryState() == true)
				{ // relay the state-dependent label cost from vehicle v to vehicle v+1
					// from time T to 0 // from destination node of v to origin node of v+1

					std::string str = g_VRStateServiceVector[s].generate_string_key();


					int t1 = g_vehicle_arrival_time_ending[v];
					int t2 = g_vehicle_departure_time_beginning[v + 1];
					int node1 = g_vehicle_destination_node[v];
					int node2 = g_vehicle_origin_node[v+1] ;

					if (la_state_node_label_cost[v][t1][s][node1] < _MAX_LABEL_COST)
					{
					
					fprintf(g_pFileDebugLog, "vehicle %d: relaying state no. %d:(%d) %s, with label cost %f\n",
						v,
						s,
						g_VRStateServiceVector[s].m_passenger_occupancy, 
						str.c_str(), 
						la_state_node_label_cost[v][t1][s][node1]);

					la_state_node_label_cost[v + 1][t2][s][node2] = la_state_node_label_cost[v][t1][s][node1];
					la_state_node_predecessor[v + 1][t2][s][node2] = la_state_node_predecessor[v][t1][s][node1];
					la_state_time_predecessor[v + 1][t2][s][node2] = la_state_time_predecessor[v][t1][s][node1];
					la_state_service_predecessor[v + 1][t2][s][node2] = la_state_service_predecessor[v][t1][s][node1];
					la_state_vehicle_predecessor[v + 1][t2][s][node2] = v;
					}
				}

			}
		}
	}  // for different vehicles


	double total_cost = _MAX_LABEL_COST;


	int reversed_path_vehicle_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int reversed_path_time_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int reversed_path_state_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int reversed_path_node_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	float reversed_path_cost_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];

	int last_s = 1;  // 1 for all serviced state node
	int last_v = g_number_of_physical_vehicles;
	int destination_node = g_vehicle_destination_node[last_v];
	int arrival_time = g_vehicle_arrival_time_ending[last_v];
	total_cost = la_state_node_label_cost[last_v][arrival_time][last_s][destination_node];

	// step 2: backtrack to the origin (based on node and time predecessors)
	int	node_size = 0;
	reversed_path_node_sequence[node_size] = destination_node;//record the first node backward, destination node
	reversed_path_time_sequence[node_size] = arrival_time;
	reversed_path_state_sequence[node_size] = last_s;
	reversed_path_vehicle_sequence[node_size] = last_v;
	reversed_path_cost_sequence[node_size] = la_state_node_label_cost[last_v][arrival_time][last_s][destination_node];


	node_size++;

	int pred_node = la_state_node_predecessor[last_v][arrival_time][last_s][destination_node];
	int pred_time = la_state_time_predecessor[last_v][arrival_time][last_s][destination_node];
	int pred_state = la_state_service_predecessor[last_v][arrival_time][last_s][destination_node];
	int pred_vehicle = la_state_vehicle_predecessor[last_v][arrival_time][last_s][destination_node];

	while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
	{
		reversed_path_node_sequence[node_size] = pred_node;
		reversed_path_time_sequence[node_size] = pred_time;
		reversed_path_state_sequence[node_size] = pred_state;
		reversed_path_vehicle_sequence[node_size] = pred_vehicle;
		reversed_path_cost_sequence[node_size] = la_state_node_label_cost[pred_vehicle][pred_time][pred_state][pred_node];

		node_size++;

		//record current values of node and time predecessors, and update PredNode and PredTime

		int pred_node_record = pred_node;
		int pred_time_record = pred_time;
		int pred_state_record = pred_state;
		int pred_vehicle_record = pred_vehicle;

		pred_node = la_state_node_predecessor[pred_vehicle_record][pred_time_record][pred_state_record][pred_node_record];
		pred_time = la_state_time_predecessor[pred_vehicle_record][pred_time_record][pred_state_record][pred_node_record];
		pred_state = la_state_service_predecessor[pred_vehicle_record][pred_time_record][pred_state_record][pred_node_record];
		pred_vehicle = la_state_vehicle_predecessor[pred_vehicle_record][pred_time_record][pred_state_record][pred_node_record];

	}

	//reverse the node sequence 
	int l_path_vehicle_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int l_path_time_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int l_path_state_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	int l_path_node_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];
	float l_path_cost_sequence[_MAX_NUMBER_OF_STEP_INTERVALS];

	double prev_cost = -1;
	for (int n = 0; n < node_size; n++)
	{
		l_path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
		l_path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
		l_path_state_sequence[n] = reversed_path_state_sequence[node_size - n - 1];
		l_path_vehicle_sequence[n] = reversed_path_vehicle_sequence[node_size - n - 1];
		l_path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];

		std::string str = g_VRStateServiceVector[l_path_state_sequence[n]].generate_string_key();

		float cost = l_path_cost_sequence[n]; 

		if (fabs(cost - prev_cost) > 0.0001)
		{
		fprintf(g_pFileDebugLog, "\index %d, Vehicle %d, time = %d, node = %d, state = %s, cost = %f\n",
			n,
			l_path_vehicle_sequence[n],
			l_path_time_sequence[n],
			g_external_node_id_map[l_path_node_sequence[n]],
			str.c_str(),
			cost);
		}

		prev_cost = cost;
	}


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

	g_number_of_nodes = 0; // initialize  the counter to 0
	g_number_of_links = 0; // initialize  the counter to 0


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

			g_maximum_number_name_of_nodes = max(g_maximum_number_name_of_nodes, node_id);

			parser.GetValueByFieldName("node_type", node_type);
			parser.GetValueByFieldName("x", X);
			parser.GetValueByFieldName("y", Y);

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 ==0)
				cout << "reading " << g_number_of_nodes << " physical nodes.. " << endl;

		}

		cout << "number of physical nodes = " << g_number_of_nodes << endl;

		g_number_of_physical_nodes = g_number_of_nodes;

		fprintf(g_pFileOutputLog, "number of physical nodes =,%d\n", g_number_of_nodes);
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
				//cout << " warning: from_node_id " << from_node_id << " has not been defined in node input file" << endl;
				continue;
			}

			if (g_internal_node_no_map.find(to_node_id) == g_internal_node_no_map.end())
			{
				// cout << "warning  " << to_node_id << " has not been defined in node input file" << endl;
				continue;
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

				parser.GetValueByFieldName("speed_limit", speed);
				if (speed >= 70)
					speed = 70;

				if (speed <= 25)
					speed = 25;


				travel_time = max(1, link_length * 60 / max(1, speed));
				parser.GetValueByFieldName("jam_density", jam_density);


				if (travel_time > 1000)
				{
					cout << "travel time of link " << from_node_id << " -> " << to_node_id << " > 1000";
					g_ProgramStop();
				}

				g_link_from_node_id[g_number_of_links] = directional_from_node_id;
				g_link_to_node_id[g_number_of_links] = directional_to_node_id;

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
					cout << "reading " << g_number_of_links << "physical links.. " << endl;

			}

		}
		 
		cout << "number of physical links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of physical links =,%d\n", g_number_of_links);

		parser.CloseCSVFile();
	}


	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		int current_agent_id = -1;
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string agent_id;
			
			parser.GetValueByFieldName("agent_id", agent_id);

			if (agent_id.length() == 0)  // break for empty line
				break;

			//check correctness of input agent_id
			if (str2int(agent_id)>current_agent_id)
			{
				current_agent_id = str2int(agent_id);
			}
			else
			{
				cout << "Illegal agent id sequence. Program stops." << endl;
				g_ProgramStop();
			}

			TRACE("agent_id = %s\n", agent_id.c_str());
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


				parser.GetValueByFieldName("capacity", g_passenger_capacity[pax_no]);
				if (g_passenger_capacity[pax_no] < 0)
				{
					cout << "Passenger data must have values in field capacity in file input_agent.csv!" << endl;
					g_ProgramStop();
				}


				parser.GetValueByFieldName("departure_time_start", g_passenger_departure_time_beginning[pax_no]);
				if (g_passenger_departure_time_beginning[pax_no]<0)
				{
					cout << "Illegal departure time input. Program stops." << endl;
					g_ProgramStop();
				}

				int departure_time_window = 0;
				parser.GetValueByFieldName("departure_time_window", departure_time_window);
				if (departure_time_window<0)
				{
					cout << "Illegal departure time window input. Program stops." << endl;
					g_ProgramStop();
				}
				//g_passenger_departure_time_ending[pax_no] = max(0, departure_time_window);
				//g_passenger_departure_time_ending[pax_no] = max(g_passenger_departure_time_ending[pax_no], c);
				g_passenger_departure_time_ending[pax_no] = g_passenger_departure_time_beginning[pax_no]+departure_time_window;

				parser.GetValueByFieldName("arrival_time_start", g_passenger_arrival_time_beginning[pax_no]);
				if (g_passenger_arrival_time_beginning[pax_no]<0)
				{
					cout << "Illegal arrival time input. Program stops." << endl;
					g_ProgramStop();
				}

					int arrival_time_window = 0;
				parser.GetValueByFieldName("arrival_time_window", arrival_time_window);
				if (arrival_time_window<0)
				{
					cout << "Illegal arrival time window input. Program stops." << endl;
					g_ProgramStop();
				}

				g_passenger_arrival_time_ending[pax_no] = g_passenger_arrival_time_beginning[pax_no] + arrival_time_window;

				//g_passenger_arrival_time_ending[pax_no] = max(g_passenger_arrival_time_ending[pax_no], g_passenger_arrival_time_beginning[pax_no]);

				g_number_of_time_intervals = max(g_passenger_arrival_time_ending[pax_no] + 10, g_number_of_time_intervals);

				//adding artificial passenger origin/pickup node id
				int new_artifical_pasenger_origin_id = g_add_new_node(pax_no, g_passenger_departure_time_beginning[pax_no], g_passenger_departure_time_ending[pax_no]);
				g_node_passenger_pickup_flag[new_artifical_pasenger_origin_id] = 1;  // mark this node is a pick-up node

				g_add_new_link(g_passenger_origin_node[pax_no], new_artifical_pasenger_origin_id, pax_no);  // pick up link
				g_add_new_link(new_artifical_pasenger_origin_id, g_passenger_origin_node[pax_no]);

				int new_artifical_pasenger_destination_id = g_add_new_node(pax_no, g_passenger_arrival_time_beginning[pax_no], g_passenger_arrival_time_ending[pax_no]);

				g_passenger_dummy_destination_node[pax_no] = new_artifical_pasenger_destination_id;
				g_add_new_link(g_passenger_destination_node[pax_no], new_artifical_pasenger_destination_id, pax_no *(-1));  // delivery link
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
			g_vehicle_departure_time_beginning[v] = max(0, g_vehicle_departure_time_beginning[p] - 10);
			g_vehicle_departure_time_ending[v] = g_vehicle_departure_time_beginning[v];
			g_vehicle_arrival_time_beginning[v] = g_number_of_time_intervals - 1;
			g_vehicle_arrival_time_ending[v] = g_number_of_time_intervals - 1;
			g_vehicle_capacity[v] = g_passenger_capacity[p];
			g_VOIVTT_per_hour[v] = g_dummy_vehicle_cost_per_hour;
			g_VOWT_per_hour[v] = 0;
			g_number_of_vehicles++;
	}

	//end of adding dummy vehicles for each passenger


	//
	for (int i = 0; i < g_number_of_nodes; i++)
	{
		if (g_outbound_node_size[i] > _MAX_NUMBER_OF_OUTBOUND_NODES-5)
		{
			cout << "Node number " << g_external_node_id_map[i] << " 's outbound node size is larger than predetermined value. Program stops" << endl;
			g_ProgramStop();
		}
	}
	//

	cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_passengers << " passengers, " << g_number_of_vehicles << "vehicles" << endl;
	fprintf(g_pFileDebugLog, "network has %d nodes, %d links, %d toll records, %d  passengers, %d vehicles\n",
		g_number_of_nodes, g_number_of_links, g_number_of_toll_records, g_number_of_passengers, g_number_of_vehicles);


	//for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
	//{
	//	int from_node = g_link_from_node_id[link];

	//	int to_node = g_link_to_node_id[link];

	//	fprintf(g_pFileDebugLog, "link no.%d,  %d->%d, service code %d\n",
	//		link + 1,
	//		g_external_node_id_map[from_node],
	//		g_external_node_id_map[to_node],
	//		g_link_service_code[link]);
	//}
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


void g_FulfillArcTravelAndNodeWaitingCost(float VOIVTT_per_hour)
{

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		// setup arc travelling cost
		for (int link = 0; link < g_number_of_links; link++)
		{
			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				g_v_arc_cost[v][link][t] = g_arc_travel_time[link][t] / 60.0 * VOIVTT_per_hour;  // 60 min per hour
				if (g_arc_travel_time[link][t]>0)
				{
					int q = 0;
				}
			}
		}

		// setup waiting cost
		for (int node = 0; node <= g_number_of_nodes; node++)
		{
			for (int t = 0; t <= g_number_of_time_intervals; t++)
			{
				g_v_vertex_waiting_cost[v][node][t] = 0;
			}
		}


	}

}


bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables()  // with varaible y only
{

	fprintf(g_pFileOutputLog, "\nIteration,Lower Bound,Upper Bound,Gap,Relative_gap,");
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "pax %d,", p);
	}
	fprintf(g_pFileOutputLog, "# of pax not served,");

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileOutputLog, "y_veh_for_pax%d,", p);
	}

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			fprintf(g_pFileOutputLog, "price_v%dp%d,", v, p);
		}
	}


	fprintf(g_pFileOutputLog, "\n");


	cout << "Preparation......" << endl;
	int VOIVTT_per_hour = 50;
	g_FulfillArcTravelAndNodeWaitingCost(VOIVTT_per_hour);//convert link travel time to arc travelling cost, and set all node waiting cost to be 0

	//step 0: initialization 
	fprintf(g_pFileDebugLog, "step 0: initialization \n");


	if (_MAX_NUMBER_OF_LINKS < g_number_of_links)
	{

		cout << "Number of links = " << g_number_of_links << ", which is greater then the max threshold of " << _MAX_NUMBER_OF_LINKS;
		g_ProgramStop();
	}
	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < g_number_of_time_intervals; t++)
		{
			g_arc_travel_time[link][t] = g_link_free_flow_travel_time[link];  //transportation cost
		}
	}

	// setup waiting cost
	for (int node = 0; node <= g_number_of_nodes; node++)
	{
		for (int t = 0; t <= g_number_of_time_intervals; t++)
		{
			for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here
			{
				g_v_vertex_waiting_cost[v][node][t] = 1;
				g_vertex_visit_count_for_lower_bound[v][node][t] = 0;
			}


		}

	}

	for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here
	for (int p = 1; p <= g_number_of_passengers; p++)
	{

		g_lamda_vehicle_passenger_assignment_multiplier[v][p] = 0;
	}
	//cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;
	//g_SolutionStartTime = CTime::GetCurrentTime();


	float g_best_upper_bound = 99999;
	float g_best_lower_bound = -99999;

	int first_stage_lamda_p_mode_flag = 1;

	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{
		if (LR_iteration >= 0)
			first_stage_lamda_p_mode_flag = 0;

		double global_lower_bound = 0;
		double global_upper_bound = 99999;

		cout << "Lagrangian Iteration " << LR_iteration << "/" << g_number_of_LR_iterations << endl;
		fprintf(g_pFileDebugLog, "----------Lagrangian Iteration: %d ----------------------------------------\n", LR_iteration + 1);


		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			for (int node = 1; node <= g_number_of_nodes; node++)
			{


				if (g_node_passenger_id[node] >= 1 && g_node_passenger_pickup_flag[node] >= 1 /*pick up node only*/ && g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
				{
					int passenger_no = g_node_passenger_id[node];
					int number_of_visits = 0;
					for (int t = g_activity_node_starting_time[node]; t < g_activity_node_ending_time[node]; t++)
					{
						number_of_visits += g_vertex_visit_count_for_lower_bound[v][node][t];
					}

					float StepSize = 1;

					if (LR_iteration == 0)
					{
						StepSize = g_passenger_base_profit[g_node_passenger_id[node]]; // initial value from base profit
					}
					else
					{
						StepSize = g_passenger_base_profit[g_node_passenger_id[node]] / (LR_iteration + 1.0f);
					}

					if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
					{
						StepSize = g_minimum_subgradient_step_size;
					}


					int p = g_node_passenger_id[node];
					for (int t = g_activity_node_starting_time[node]; t < g_activity_node_ending_time[node]; t++)  // for all vertex in the time window
					{
						int ref_assignment_visit_flag = 1;

						if (first_stage_lamda_p_mode_flag == 0)
						{
							ref_assignment_visit_flag = g_y_vehicle_serving_passenger_matrix[v][passenger_no];
						}

						if (LR_iteration == 0)
						{
							g_lamda_vehicle_passenger_assignment_multiplier[v][p] = g_passenger_base_profit[g_node_passenger_id[node]] * (-1);
						}
						else
						{

							g_lamda_vehicle_passenger_assignment_multiplier[v][p] += StepSize * (number_of_visits - ref_assignment_visit_flag);  // decrease the value and create profit

							if (g_lamda_vehicle_passenger_assignment_multiplier[v][p] > -1)  // prevent postive price and over statisfaction
								g_lamda_vehicle_passenger_assignment_multiplier[v][p] = -1;
						}
						g_to_node_cost[v][node][t] = g_lamda_vehicle_passenger_assignment_multiplier[v][p];
						//for pax's origin with the fixed departure time only for now. need to consider destination's price and flexible time window later

						if (number_of_visits == 1)
						{

							fprintf(g_pFileDebugLog, "\>>> Vehicle %d: Passenger %d  iteration no. %d at node %d and time %d visit = %d, the multiplier keeps as  %f",
								v,
								g_node_passenger_id[node],
								LR_iteration,
								node,
								t,
								number_of_visits,
								g_lamda_vehicle_passenger_assignment_multiplier[v][p]);
							fprintf(g_pFileDebugLog, "\n");
						}
						else
						{
							fprintf(g_pFileDebugLog, "\>>> Vehicle %d, Passenger  %d iteration no. %d  at node %d and time %d visit = %d, the multiplier changes to  %f",
								v,
								g_node_passenger_id[node],
								LR_iteration,
								node,
								t,
								number_of_visits,
								g_lamda_vehicle_passenger_assignment_multiplier[v][p]);
							fprintf(g_pFileDebugLog, "\n");
						}
					}

				}


			}
		} // for each vehicle

		// Lower bound 

		// reset the vertex visit count
		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			for (int node = 0; node < g_number_of_nodes; node++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_vertex_visit_count_for_lower_bound[v][node][t] = 0;

				}
			}
		}
		for (int v = 1; v <= g_number_of_vehicles; v++)
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_vehicle_serving_passenger_matrix[v][p] = 0;

		}

		if (first_stage_lamda_p_mode_flag == 1)
		{
			fprintf(g_pFileDebugLog, "--- first stage: vehicle to passenger assignment vector matrix ----\n");
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				fprintf(g_pFileDebugLog, "p=%d: %4.0f\n", p, g_lamda_vehicle_passenger_assignment_multiplier[1][p]);
			}
		}
		else  //second stage
		{

			fprintf(g_pFileDebugLog, "--- second stage: vehicle to passenger assignment price matrix ----\n       ");

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				fprintf(g_pFileDebugLog, "  p%2d: ", p);  //header
			}

			for (int v = 1; v <= g_number_of_physical_vehicles; v++)
			{
				fprintf(g_pFileDebugLog, "\nveh %2d: ", v);

				for (int p = 1; p <= g_number_of_passengers; p++)
				{

					fprintf(g_pFileDebugLog, "%4.0f,  ", g_lamda_vehicle_passenger_assignment_multiplier[v][p]);

				}
			}
			fprintf(g_pFileDebugLog, "\n");  //last line

			// semi assignment

			fprintf(g_pFileDebugLog, "--- vehicle to passenger semi_assignment matrix as subproblem for each passenger----\n       ");
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				float least_cost = 999999;
				int vehicle_no_for_least_cost = 0;
				for (int v = 1; v <= g_number_of_vehicles; v++)
				{
					g_y_vehicle_serving_passenger_matrix[v][p] = 0;

					if (g_lamda_vehicle_passenger_assignment_multiplier[v][p] * (-1) < least_cost)
					{

						least_cost = g_lamda_vehicle_passenger_assignment_multiplier[v][p] * (-1);
						vehicle_no_for_least_cost = v;

					}
				}
				g_y_vehicle_serving_passenger_matrix[vehicle_no_for_least_cost][p] = 1;
				fprintf(g_pFileDebugLog, "--- y of vehicle %d is assigned to pax %d = ----\n,", vehicle_no_for_least_cost, p);
			}

		}
#pragma omp parallel for
		for (int ProcessID = 0; ProcessID < g_number_of_threads; ProcessID++)
		{
			//step 2: shortest path for vehicle

			for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here  // include both physical and virtual vehicles
			{
				if (v%g_number_of_threads != ProcessID)  // if vehicle id does not belong processor p, skip // we only perform calculation for a cluster vehicles belong for p 
					continue;
				int	id = omp_get_thread_num();  // starting from 0

				cout << "#Iteration: " << LR_iteration + 1 << " Processor " << id << " is calculating the LB shortest paths for vehicle " << v << "..." << endl;


				// set arc cost, to_node_cost and waiting_cost for vehicles

				for (int link = 0; link < g_number_of_links; link++)
				{
					for (int t = 0; t < g_number_of_time_intervals; t++)
					{
						g_v_arc_cost[v][link][t] = g_arc_travel_time[link][t] / 60.0 * g_VOIVTT_per_hour[v];  // 60 min pur hour
					}
				}

				// setup waiting cost
				for (int node = 0; node <= g_number_of_nodes; node++)
				{
					for (int t = 0; t <= g_number_of_time_intervals; t++)
					{

						g_v_vertex_waiting_cost[v][node][t] = 1 / 60.0* g_VOWT_per_hour[v];
					}
				}
				// special case: no waiting cost at vehicle returning depot

				for (int t = 0; t <= g_number_of_time_intervals; t++)
				{
					int vehicle_destination_node = g_vehicle_destination_node[v];
					g_v_vertex_waiting_cost[v][vehicle_destination_node][t] = 0;
				}


				if (g_number_of_threads == 1)  //single thread
				{

					fprintf(g_pFileDebugLog, "\Debug: LB iteration %d, Vehicle %d performing DP: origin %d -> destination %d\n ",
						LR_iteration,
						v,
						g_vehicle_origin_node[v],
						g_vehicle_destination_node[v]);
				}


				//fprintf(g_pFileDebugLog, "\n");
				float path_cost_by_vehicle_v =
					g_pc_optimal_time_dependenet_dynamic_programming
					(ProcessID,
					v,
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
					g_path_travel_time[v],
					false);

				g_vehicle_path_cost[v] = path_cost_by_vehicle_v;
#pragma omp critical
				{
					global_lower_bound += path_cost_by_vehicle_v;
				}

			}  //for each v
		} //for each p
		for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here  // include both physical and virtual vehicles
		{
			fprintf(g_pFileDebugLog, "Vehicle %d'  path has %d nodes with a transportation cost of %f and travel time of %d: ",
				v,
				g_vehicle_path_number_of_nodes[v],
				g_vehicle_path_cost[v],
				g_path_travel_time[v]);
			fprintf(g_pFileDebugLog, "\n");


			fprintf(g_pFileDebugLog, "[[time, node, state, service code, travel cost, to_node_cost,cumulative_cost ($), travel time(interval), travel time(min)");
			fprintf(g_pFileDebugLog, "\n");

			float cumulative_cost = 0;
			float DP_cumulative_cost = 0;
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
			{
				int state_index = g_vehicle_path_state_sequence[v][i];

				if (state_index>0)
				{

					fprintf(g_pFileDebugLog, "veh:%d,%5d,%5d,",
						v,
						g_vehicle_path_time_sequence[v][i],
						g_external_node_id_map[g_vehicle_path_node_sequence[v][i]]
						);
				}

				DP_cumulative_cost = g_vehicle_path_cost_sequence[v][i];



				std::string state_str;

				if (state_index < g_VRStateCarryingVector.size())
				{

					state_str = g_VRStateCarryingVector[state_index].generate_string_key();
				}

				if (state_index>0)
				{

					fprintf(g_pFileDebugLog, "[%s],",
						state_str.c_str()
						);
				}

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

						travel_cost = g_v_arc_cost[v][link_id][g_vehicle_path_time_sequence[v][i]];
						to_node_cost = g_to_node_cost[v][g_vehicle_path_node_sequence[v][i + 1]][g_vehicle_path_time_sequence[v][i + 1]];
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
						fprintf(g_pFileDebugLog, "%s,$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2fmin,",
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

						if (fabsf(to_node_cost) > 0.1)
						{
							fprintf(g_pFileDebugLog, "%s(%2d),$%.4f,{@ %.1f},$$%.4f,$*%.4f,%d,%.2fmin,",
								service_str.c_str(),
								service_code,
								travel_cost,
								to_node_cost,
								cumulative_cost,
								DP_cumulative_cost,
								travel_time,
								float_travel_time);

						}
						else
						{
							fprintf(g_pFileDebugLog, "%s(%2d),$%.4f,%.1f,$%.4f,$*%.4f,%d,%.2fmin,",
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

				}
				else
				{
					//float waiting_cost = g_v_vertex_waiting_cost[v][g_vehicle_path_node_sequence[v][i]][g_vehicle_path_time_sequence[v][i]];
					//cumulative_cost += waiting_cost;
					//fprintf(g_pFileDebugLog, "--waiting, $%.4f,$%.4f,$*%.4f,-",
					//	waiting_cost,
					//	cumulative_cost,
					//	DP_cumulative_cost
					//	);
				}
				if (state_index > 0)
				{
					fprintf(g_pFileDebugLog, "\n");
				}

			}

			fprintf(g_pFileDebugLog, "]]\n");


			fprintf(g_pFileDebugLog, "global_lower_bound for vehicle %d += %f = %f\n", v, g_vehicle_path_cost[v], global_lower_bound);

		} // for vehicle v loop including physical and virtual vehicles

		// step 3: scan all vehicles to mark the useage of the corresponding vertex 
		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v] - 1; i++)  // for each link along the path
			{
				int node = g_vehicle_path_node_sequence[v][i];
				int time_index = g_vehicle_path_time_sequence[v][i];

				if (node >= 0 && g_node_passenger_id[node] >= 1) // feasible from node i
				{
					g_vertex_visit_count_for_lower_bound[v][node][time_index] += 1;  // vehicle uses the link at time t
					int p = g_node_passenger_id[node];
					g_vehicle_serving_passenger_matrix[v][p] = 1;
				}
			}

		}


		// step 4: scan all passengers and update multipliers and to_node_cost

		double total_multiplier_price_with_constant = 0;
		// scan multipliers 
		if (first_stage_lamda_p_mode_flag == 1)
		{
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				int v = 1;  // use the price for the first vehicle to represent the price vector
				total_multiplier_price_with_constant += (g_y_vehicle_serving_passenger_matrix[v][p] * g_lamda_vehicle_passenger_assignment_multiplier[v][p]);
			}
		}
		else  //second stage
		{

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				for (int v = 1; v <= g_number_of_vehicles; v++)
				{
					total_multiplier_price_with_constant += (g_y_vehicle_serving_passenger_matrix[v][p] * g_lamda_vehicle_passenger_assignment_multiplier[v][p]);

				}
			}
		}

		float old_global_lower_bound = global_lower_bound;
		global_lower_bound += total_multiplier_price_with_constant*(-1);
		fprintf(g_pFileDebugLog, "\nupdate lower bound by += total price for all pax: total (positive) price of %f \nglobal_lower_bound = %f + %f = %f  \n",
			total_multiplier_price_with_constant*(-1),
			old_global_lower_bound,
			total_multiplier_price_with_constant*(-1),
			global_lower_bound);




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


		g_best_lower_bound = max(g_best_lower_bound, global_lower_bound);  // keep the best lower bound till current iteration

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
			LR_iteration + 1,
			g_best_lower_bound,
			g_best_upper_bound,
			(g_best_upper_bound - g_best_lower_bound),
			(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0,
			number_of_pax_not_served
			);

		fprintf(g_pFileOutputLog, "%d,%f,%f,%f,%.3f%%,",
			LR_iteration + 1,
			g_best_lower_bound,
			g_best_upper_bound,
			(g_best_upper_bound - g_best_lower_bound),
			(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
			);
		//current solution
		int count_not_served = 0;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			if (g_vehicle_id_for_pax_being_served[p] == 0)
			{

				fprintf(g_pFileOutputLog, "*,");
				count_not_served++;
			}
			else
			{
				fprintf(g_pFileOutputLog, "%d,", g_vehicle_id_for_pax_being_served[p]);
			}

		}

		fprintf(g_pFileOutputLog, "%d,", count_not_served);

		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			int vehicle_no_assigned = 0;
			for (int v = 1; v <= g_number_of_vehicles; v++)
			{
				if (g_y_vehicle_serving_passenger_matrix[v][p] == 1)
				{
					vehicle_no_assigned = v;
					break;
				}
			}
			fprintf(g_pFileOutputLog, "%d,", vehicle_no_assigned);
		}

		int number_vehicle_price_columns = g_number_of_vehicles;
		if (first_stage_lamda_p_mode_flag == 1)
		{
			number_vehicle_price_columns = 1;

		}

		for (int v = 1; v <= number_vehicle_price_columns; v++)
		{
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				fprintf(g_pFileOutputLog, "%4.1f,", g_lamda_vehicle_passenger_assignment_multiplier[v][p]);
			}
		}
		fprintf(g_pFileOutputLog, "\n");
	}


	//for each lagrangian relaxation iteration

	cout << "End of Lagrangian Iteration Process " << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;



	return true;
}

bool g_Optimization_Integrated_Assignment_Routing_Method_Vehicle_Routing_Problem()  
{

	fprintf(g_pFileOutputLog, "\n");


	cout << "Preparation......" << endl;
	int VOIVTT_per_hour = 50;
	g_FulfillArcTravelAndNodeWaitingCost(VOIVTT_per_hour);//convert link travel time to arc travelling cost, and set all node waiting cost to be 0

	//step 0: initialization 
	fprintf(g_pFileDebugLog, "step 0: initialization \n");


	if (_MAX_NUMBER_OF_LINKS < g_number_of_links)
	{

		cout << "Number of links = " << g_number_of_links << ", which is greater then the max threshold of " << _MAX_NUMBER_OF_LINKS;
		g_ProgramStop();
	}
	for (int link = 0; link < g_number_of_links; link++)
	{
		for (int t = 0; t < g_number_of_time_intervals; t++)
		{
			g_arc_travel_time[link][t] = g_link_free_flow_travel_time[link];  //transportation cost
		}
	}

	// setup waiting cost
		for (int node = 0; node <= g_number_of_nodes; node++)
		{
			for (int t = 0; t <= g_number_of_time_intervals; t++)
			{
				for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here
				{
					g_v_vertex_waiting_cost[v][node][t] = 1;
					g_vertex_visit_count_for_lower_bound[v][node][t] = 0;
				}


			}

		}

	
	//cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	//cout << "Running Time:" << g_GetAppRunningTime() << endl;
	//g_SolutionStartTime = CTime::GetCurrentTime();

		for (int v = 1; v <= g_number_of_vehicles; v++)
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_vehicle_serving_passenger_matrix[v][p] = 0;

		}

	

			//step 2: shortest path for vehicle

		for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here  // include both physical and virtual vehicles
		{

			// set arc cost, to_node_cost and waiting_cost for vehicles

			for (int link = 0; link < g_number_of_links; link++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_v_arc_cost[v][link][t] = g_arc_travel_time[link][t] / 60.0 * g_VOIVTT_per_hour[v];  // 60 min pur hour
				}
			}

			// setup waiting cost
			for (int node = 0; node <= g_number_of_nodes; node++)
			{
				for (int t = 0; t <= g_number_of_time_intervals; t++)
				{

					g_v_vertex_waiting_cost[v][node][t] = 1 / 60.0* g_VOWT_per_hour[v];
				}
			}
			// special case: no waiting cost at vehicle returning depot

			for (int t = 0; t <= g_number_of_time_intervals; t++)
			{
				int vehicle_destination_node = g_vehicle_destination_node[v];
				g_v_vertex_waiting_cost[v][vehicle_destination_node][t] = 0;
			}

		}

				//fprintf(g_pFileDebugLog, "\n");
				float path_cost_by_vehicle_v =
					g_integrated_assignment_routing_dynamic_programming ();



		//back trace
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

	fprintf(g_pFileDebugLog, "--- vehicle to passenger assignment matrix from LB (before updating) ----\n       ");

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileDebugLog, "  p%2d: ", p);  //header
	}

	for (int v = 1; v <= g_number_of_physical_vehicles; v++)
	{
		fprintf(g_pFileDebugLog, "\nveh %2d: ", v);

		for (int p = 1; p <= g_number_of_passengers; p++)
		{

			fprintf(g_pFileDebugLog, "  %2d,  ", g_vehicle_serving_passenger_matrix[v][p]);  //header

		}
	}
	fprintf(g_pFileDebugLog, "\n");  //last line

	// update the values
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		int being_served_flag = 0;

		for (int v = 1; v <= g_number_of_physical_vehicles; v++)
		{

			if (being_served_flag == 1)
				g_vehicle_serving_passenger_matrix[v][p] = 0; // reset;

			if (being_served_flag == 0 && g_vehicle_serving_passenger_matrix[v][p] == 1)  //first time served by a vehicle
			{

				being_served_flag = 1;
				g_vehicle_id_for_pax_being_served[p] = v;
			}


		}

	}

		fprintf(g_pFileDebugLog, "--- vehicle to passenger assignment matrix for UB (after updating) ----\n       ");

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		fprintf(g_pFileDebugLog, "  p%2d: ", p);  //header
	}

	for (int v = 1; v <= g_number_of_physical_vehicles; v++)
	{
		fprintf(g_pFileDebugLog, "\nveh %2d: ", v);

		for (int p = 1; p <= g_number_of_passengers; p++)
		{

			fprintf(g_pFileDebugLog, "  %2d,  ", g_vehicle_serving_passenger_matrix[v][p]);  //header

		}
	}
	fprintf(g_pFileDebugLog, "\n");  //last line


		//step 2: shortest path for vehicle
#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < g_number_of_threads; ProcessID++)
	{

		for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here  // include both physical and virtual vehicles
		{
			if (v%g_number_of_threads != ProcessID)  // if vehicle id does not belong processor p, skip // we only perform calculation for a cluster vehicles belong for p 
				continue;

			// setup 1: set arc cost, to_node_cost and waiting_cost for vehicles
			if (v > g_number_of_physical_vehicles)
			{
				int assigned_pax_id = v - g_number_of_physical_vehicles;

				if (g_vehicle_id_for_pax_being_served[assigned_pax_id] >= 1)
					continue;  // no need to serve this passenger using virtual vehicle 
			}

			int	id = omp_get_thread_num();  // starting from 0

			cout << "#Iteration: " << LR_Iteration_no + 1 << " Processor " << id << " is calculating the UB shortest paths using vehicle " << v << "..." << endl;


			for (int link = 0; link < g_number_of_links; link++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_v_arc_cost[v][link][t] = g_arc_travel_time[link][t] / 60.0 * g_VOIVTT_per_hour[v];  // 60 min pur hour
				}
			}

			// setup waiting cost
			for (int node = 0; node <= g_number_of_nodes; node++)
			{
				for (int t = 0; t < g_number_of_time_intervals; t++)
				{
					g_v_vertex_waiting_cost[v][node][t] = 1 / 60.0* g_VOWT_per_hour[v];
					g_v_to_node_cost_used_for_upper_bound[v][node][t] = 0;
				}
			}
			// special case: no waiting cost at vehicle returning depot

			for (int t = 0; t < g_number_of_time_intervals; t++)
			{
				int vehicle_destination_node = g_vehicle_destination_node[v];
				g_v_vertex_waiting_cost[v][vehicle_destination_node][t] = 0;
			}

			// setup to node cost for each vehicle in finding its own upper bound/ feasible solution

			for (int node = 1; node <= g_number_of_nodes; node++)
			{

				if (g_node_passenger_id[node] >= 1 && g_node_passenger_pickup_flag[node] >= 1 /*pick up node only*/
					&& g_activity_node_starting_time[node] >= 0 && g_activity_node_ending_time[node] >= 0)
				{
					int p = g_node_passenger_id[node];
					for (int t = g_activity_node_starting_time[node]; t < g_activity_node_ending_time[node]; t++)  // for all vertex in the time window
					{

						if (v <= g_number_of_physical_vehicles)
						{


							if (g_vehicle_serving_passenger_matrix[v][p] == 1)  // physical vehicle assigned to this passenger
								g_v_to_node_cost_used_for_upper_bound[v][node][t] = g_to_node_cost[v][node][t] -1000;  // from existing lower bound solution
							else
								g_v_to_node_cost_used_for_upper_bound[v][node][t] = 999999;  // do not serve this pas!

						}
						else  // virtual vehicle
						{
							if (v - g_number_of_physical_vehicles == p && g_vehicle_id_for_pax_being_served[p] == 0)  // for your assigned and not being pax id only
							{
								float dummy_cost = (-1)*_MAX_NUMBER_OF_TIME_INTERVALS*g_dummy_vehicle_cost_per_hour;
								g_v_to_node_cost_used_for_upper_bound[v][node][t] = dummy_cost;  // artificial cost for virtual vehicles
								//							fprintf(g_pFileDebugLog, "\Place 1000 $ bill for picking up non-served and dedicated pax id %d for virtual vehicle %d \n ", g_node_passenger_id[node], v);

							}
						}

					}
				}
			}


			//step 2: shortest path for vehicle with to node cost for passengers not being served


			float path_cost_by_vehicle_v =
				g_pc_optimal_time_dependenet_dynamic_programming
				(ProcessID,
				v,
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
				g_path_travel_time[v],
				true);
			g_vehicle_path_cost[v] = path_cost_by_vehicle_v;

		}
	}
	// print out
	for (int v = 1; v <= g_number_of_vehicles; v++)//note that the scheduling sequence does not matter  here  // include both physical and virtual vehicles
	{
		if (v > g_number_of_physical_vehicles)
		{
			int assigned_pax_id = v - g_number_of_physical_vehicles;

			if (g_vehicle_id_for_pax_being_served[assigned_pax_id] >= 1)
				continue;  // no need to serve this passenger using virtual vehicle 
		}

			fprintf(g_pFileDebugLog, "\Upper bound: Vehicle %d'  path has %d nodes with a transportation cost of %f and travel time of %d: ",
				v,
				g_vehicle_path_number_of_nodes[v],
				g_vehicle_path_cost[v],
				g_path_travel_time[v]);
			fprintf(g_pFileDebugLog, "\n");


			fprintf(g_pFileDebugLog, "UB: [[time, node, state, service code, travel cost, to_node_cost,cumulative_cost ($), travel time(interval), travel time(min)");
			fprintf(g_pFileDebugLog, "\n");

			float cumulative_upper_bound_cost = 0;
			float DP_cumulative_cost = 0;

			//		day_no, , agent_type, path_node_sequence, path_time_sequence, path_state_sequence,

			if (LR_Iteration_no == g_number_of_LR_iterations - 1)
			{

				fprintf(g_pFileAgentPathLog, "%d,v%s,v,",
					LR_Iteration_no + 1,
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
			//start logging 
			for (int i = 0; i < g_vehicle_path_number_of_nodes[v]; i++)
			{
				int state_index = g_vehicle_path_state_sequence[v][i];

				if (state_index > 0)
				{
					if (g_external_node_id_map.find(g_vehicle_path_node_sequence[v][i]) != g_external_node_id_map.end())
					{


						fprintf(g_pFileDebugLog, "UB: veh:%d,%5d,%5d,",
							v,
							g_vehicle_path_time_sequence[v][i],

							g_vehicle_path_node_sequence[v][i]);


					}

					else
					{


						fprintf(g_pFileDebugLog, "UB: veh:%d,%5d,*%5d,",
							v,
							g_vehicle_path_time_sequence[v][i],

							g_vehicle_path_node_sequence[v][i]);



					}
				}

				DP_cumulative_cost = g_vehicle_path_cost_sequence[v][i];
				

				std::string state_str;

				if (state_index < g_VRStateCarryingVector.size())
				{

					state_str = g_VRStateCarryingVector[state_index].generate_string_key();
				}

				if (state_index>0)
				{
				
				fprintf(g_pFileDebugLog, "[%s],",
					state_str.c_str()
					);
				}

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


						travel_cost = g_v_arc_cost[v][link_id][g_vehicle_path_time_sequence[v][i]];
						to_node_cost = g_to_node_cost[v][g_vehicle_path_node_sequence[v][i + 1]][g_vehicle_path_time_sequence[v][i + 1]];
						cumulative_upper_bound_cost += (travel_cost);  // we do not have "to node cost"
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
					//float waiting_cost = g_v_vertex_waiting_cost[v][g_vehicle_path_node_sequence[v][i]][g_vehicle_path_time_sequence[v][i]];
					//cumulative_upper_bound_cost += waiting_cost;
					//fprintf(g_pFileDebugLog, "UB: --waiting, $%.4f,$%.4f,$*%.4f,-",
					//	waiting_cost,
					//	cumulative_upper_bound_cost,
					//	DP_cumulative_cost
					//	);
				}
				if (state_index > 0)
				{

				fprintf(g_pFileDebugLog, "\n");
				}

			}

			fprintf(g_pFileDebugLog, "]]\n");


			global_upper_bound += cumulative_upper_bound_cost;

			fprintf(g_pFileDebugLog, "iteration %d, global_upper_bound for vehicle %d += %f = %f\n", 
				LR_Iteration_no+1, v, cumulative_upper_bound_cost, global_upper_bound);


	}  //end for vehicle

	
	fprintf(g_pFileDebugLog, "Summary: Iteration %d: total operating cost =  %f\n",
		LR_Iteration_no, global_upper_bound);

	for (int p = 1; p <= g_number_of_passengers; p++)
	{
			if (g_vehicle_id_for_pax_being_served[p] >= 1)
			{
				fprintf(g_pFileDebugLog, "Summary: Iteration %d, pax %d -> veh %d\n",
					LR_Iteration_no, p, g_vehicle_id_for_pax_being_served[p]);
			}
			else
			{
				fprintf(g_pFileDebugLog, "Summary: Iteration %d, pax %d -> ** virtual veh \n",
					LR_Iteration_no, p);


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

			int number_of_threads = 1;
			double X;
			double Y;
			parser.GetValueByFieldName("number_of_iterations", g_number_of_LR_iterations);
			parser.GetValueByFieldName("shortest_path_debugging_details", g_shortest_path_debugging_flag);
			parser.GetValueByFieldName("dummy_vehicle_cost_per_hour", g_dummy_vehicle_cost_per_hour);
			parser.GetValueByFieldName("minimum_subgradient_step_size", g_minimum_subgradient_step_size);
			parser.GetValueByFieldName("max_number_of_threads_to_be_used", number_of_threads);
			
			
			
			if (number_of_threads <= 0)
				number_of_threads = 1;

			if (number_of_threads > omp_get_max_threads())
				number_of_threads = omp_get_max_threads();
			int a = omp_get_max_threads();
			g_number_of_threads = number_of_threads;
			


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

	for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
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

			if (to_node==3)
			{
				int q = 0;
			}

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
					/*if (g_shortest_path_debugging_flag)
						fprintf(g_pFileDebugLog, "SP: add node %d into SE List\n",
						to_node);*/

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
	
		g_optimal_label_correcting(g_passenger_origin_node[p], g_passenger_departure_time_beginning[p]);//fulfill 
		
		float earliest_arrival_time = g_node_to_node_shorest_travel_time[g_passenger_origin_node[p]][g_passenger_destination_node[p]];
		
		if (earliest_arrival_time>1440)
		{
			cout << "Abnormal earliest arrival time. Program stops." << endl;
			g_ProgramStop();
		}

		g_passenger_request_travel_time_vector[p] = max(5, earliest_arrival_time - g_passenger_departure_time_beginning[p]);

		g_passenger_request_cancelation_cost[p] = g_passenger_request_travel_time_vector[p] / 60.0 * 30;
		// from the pax's destination to all the other nodes starting from the earliest arrival time at the d
		g_optimal_label_correcting(g_passenger_destination_node[p], earliest_arrival_time);		
		fprintf(g_pFileDebugLog, "pax no.%d, Departure Time = %d (min), Travel Time = %.2f (min), Earliest Arrival Time = %.2f, cost = $%.2f\n",
			p, g_passenger_departure_time_beginning[p],
			g_passenger_request_travel_time_vector[p],
			earliest_arrival_time,
			g_passenger_request_cancelation_cost[p]);

		if (g_passenger_arrival_time_beginning[p] < earliest_arrival_time + 2)
		{
			int existing_arrival_time_window = g_passenger_arrival_time_ending[p] - g_passenger_arrival_time_beginning[p];
			
			fprintf(g_pFileDebugLog, "modified arrival time window for pax no.%d [%d,%d]  \n",
				p, earliest_arrival_time + 2, earliest_arrival_time + 2 + existing_arrival_time_window);

			g_passenger_arrival_time_beginning[p] = earliest_arrival_time + 2;
			g_passenger_arrival_time_ending[p] = g_passenger_arrival_time_beginning[p] + existing_arrival_time_window; 

			float ideal_profit = g_passenger_request_travel_time_vector[p] / 60 * 22;  // 22 dolloar per 60 min 

			if (g_passenger_base_profit[p] < ideal_profit)
			{
				fprintf(g_pFileDebugLog, "modified base profit for pax no.%d from %f to %f  \n",
					p, g_passenger_base_profit[p], ideal_profit);

				g_passenger_base_profit[p] = ideal_profit;
			}


		}

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

		g_external_vehicle_id_map[v] = "virtual"+ std::to_string(v);

		int required_time_for_virtual_vehicle_to_serve_pax = g_passenger_departure_time_beginning[p] + 2 * g_passenger_request_travel_time_vector[p] + 60;

		if (g_number_of_time_intervals - 1 < required_time_for_virtual_vehicle_to_serve_pax)
			g_number_of_time_intervals = required_time_for_virtual_vehicle_to_serve_pax + 1;  //increase planning horizon to complete all jobs

		g_vehicle_arrival_time_beginning[v] = min(g_number_of_time_intervals - 1, required_time_for_virtual_vehicle_to_serve_pax); // ad-hoc upper bound: warning: need to be carefule when there are congestions:  +60 min as buffer // 2* as forward and return trips
   //		g_vehicle_arrival_time_beginning[v] = min(g_number_of_time_intervals - 1, required_time_for_virtual_vehicle_to_serve_pax); // ad-hoc upper bound: warning: need to be carefule when there are congestions:  +60 min as buffer // 2* as forward and return trips
		g_vehicle_arrival_time_ending[v] = g_vehicle_arrival_time_beginning[v];

		fprintf(g_pFileOutputLog, "%s%d,%d\n",
			
		g_external_vehicle_id_map[v].c_str(),
		g_vehicle_departure_time_beginning[v],
		g_vehicle_arrival_time_ending[v]
		
		);


	}

	// calculate shortest path from each vehicle's origin to all nodes

	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		cout << ">>find shortest path tree for vehicle v = " << v << "..." << endl;
		g_optimal_label_correcting(g_vehicle_origin_node[v], g_vehicle_departure_time_beginning[v]);

		for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
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

		for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
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
void g_ProgramStop()
{

	cout << "Agent+ Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};

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

	int multi_vehicle_mode_flag = 0;

	if (multi_vehicle_mode_flag==0)  // carrying statse based formulation
	g_create_all_carrying_states(g_number_of_passengers, g_max_vehicle_capacity);
	else  // ==1 service state based formulation
		g_create_all_service_states(g_number_of_passengers);

	g_allocate_memory_travel_time(0);
	g_generate_travel_time_matrix();
	
	//DP
	g_allocate_memory_DP(0);

	// define timestamps
	clock_t start_t, end_t, total_t;
	int i;

	start_t = clock();
	
	if (multi_vehicle_mode_flag == 0)  // carrying state based formulation
		g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables();
	else  // ==1 service state based formulation
	{
		g_Optimization_Integrated_Assignment_Routing_Method_Vehicle_Routing_Problem();
	}
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
	
	g_free_memory_DP(0);
	g_free_memory_travel_time(0);
	
	cout << "done." << endl;

	return nRetCode;
}


