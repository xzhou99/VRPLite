// AgentPlus.cpp : Defines the entry point for the console application.
/* Copyright (C) 2015 Xuesong Zhou - All Rights Reserved*/

//add your names here

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

FILE* g_pFile_OutputAgentLog = NULL;
FILE* g_pFile_OutputNodeLog = NULL;
FILE* g_pFile_OutputLinkLog = NULL;

FILE* g_pFileOutputLog = NULL;
//FILE* g_pFileAgentPathLog = NULL;

FILE* g_pFile_Output_paxprofitLog = NULL;
FILE* g_pFile_PathLog = NULL;

FILE* g_pFile_Vehicle_Path_PassengerLog = NULL;

std::map<int, int> g_internal_node_no_map;
std::map<int, int> g_external_node_id_map;

std::map<int, int> g_internal_agent_no_map;
std::map<int, string> g_external_passenger_id_map;
std::map<int, string> g_external_vehicle_id_map;

int g_number_of_threads = 4;
// passenger info in Node as pickup and dropoff node are all passengers
int g_node_passenger_id[_MAX_NUMBER_OF_NODES];
int g_node_type[_MAX_NUMBER_OF_NODES];  //key nodeid; 1: pick up, 2: drop off
int g_node_timestamp[_MAX_NUMBER_OF_NODES];  //key nodeid;, values type 1: pick up:ready time, 2: order time
float g_node_baseprofit[_MAX_NUMBER_OF_NODES];  // type 2; = 0
//the number of outbound nodes. key nodeid;, values type 1: pick up:ready time, 2: order time
int g_outbound_node_size[_MAX_NUMBER_OF_NODES];
std::map<int, int> pax_id_to_seq_no_map;


float g_passenger_base_profit[_MAX_NUMBER_OF_PASSENGERS] = { -7 };
float local_vehicle_passenger_additional_profit[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

float g_passenger_order_time[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
//used in profit(LR multipliers) update: summation of variable y
int g_passenger_number_of_visits[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_vehicle_visit_flag[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_VEHICLES] = { 0 };
//whether vehicle v can take passenger p; 1 ok 0 no; in prohibit links
//prohibt_visit and passernger can only be carried once control this one
int g_vehicle_passenger_visit_allowed_flag[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

int g_max_vehicle_capacity = 1;
int g_number_of_passengers = 0;

int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_activity_node_flag[_MAX_NUMBER_OF_NODES] = { 0 };
int g_activity_node_ending_time[_MAX_NUMBER_OF_NODES] = { 99999 };
int g_activity_node_starting_time[_MAX_NUMBER_OF_NODES] = { 99999 };


int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

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

int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_depot_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_depot_destination_node[_MAX_NUMBER_OF_VEHICLES];

int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 0 };
int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 120 };
int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];


int g_passenger_origin_node[_MAX_NUMBER_OF_PASSENGERS];  // traveling passengers
int g_passenger_destination_node[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_dummy_destination_node[_MAX_NUMBER_OF_PASSENGERS] = { -1 };

int g_passenger_departure_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_departure_time_ending[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_PASSENGERS];

int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_physical_nodes = 0;
int g_number_of_time_intervals = 10;

int g_updated_number_of_time_intervals = 0;

int g_number_of_vehicles = 0;

int g_number_of_LR_iterations = 1;
double g_minimum_subgradient_step_size = 0.01;

int g_shortest_path_debugging_flag = 1;
float g_waiting_time_ratio = 0.005;
float g_dummy_vehicle_cost_per_hour = 100;

float g_travel_time_budget = 100;
float g_idle_vehicle_benefit = -10;

CTime g_SolutionStartTime;

int g_add_new_node_origin(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = passenger_id;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 1;

	g_number_of_nodes++;
	return g_number_of_nodes;
}

int g_add_new_node(int vehicle_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = 0;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 3;

	g_number_of_nodes++;
	return g_number_of_nodes;
}

int g_add_new_node_destination(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = passenger_id;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 2;

	g_number_of_nodes++;
	return g_number_of_nodes;
}

int g_add_new_link(int from_node_id, int to_node_id, int passenger_id = 0, int travel_time = 1, double link_length = 1, int number_of_lanes = 1, int mode_code = 0,
	int capacity_per_time_interval = 1, double speed = 60)
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

vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);

	char Delimiter = ';';

	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}


void g_ProgramStop()
{

	cout << "Agent+ Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};

//class for vehicle scheduling states
class CVSState
{
public:
	int current_node_id;  // space dimension
						  //passengerID, nodeType
	std::map<int, int> passenger_service_state;
	std::map<int, int> passenger_service_time;
	std::map<int, int> passenger_service_begin_time;

	std::map<int, int> passenger_carrying_state;
	//visit nodes
	std::vector<int> m_visit_sequence;  // store nodes f
	std::vector<int> m_visit_time_sequence;  // store passing nodes times

	int m_vehicle_capacity;
	float LabelCost;  // with LR price
	float PrimalLabelCost;  // without LR price

	int m_final_arrival_time;   // for ending states


	CVSState()
	{
		m_final_arrival_time = 0;
		LabelCost = _MAX_LABEL_COST;
		m_vehicle_capacity = 0;
	}

	void Copy(CVSState* pSource)
	{
		current_node_id = pSource->current_node_id;
		passenger_service_state.clear();
		passenger_service_state = pSource->passenger_service_state;

		passenger_service_time.clear();
		passenger_service_time = pSource->passenger_service_time;

		passenger_service_begin_time.clear();
		passenger_service_begin_time = pSource->passenger_service_begin_time;

		passenger_carrying_state.clear();
		passenger_carrying_state = pSource->passenger_carrying_state;

		m_visit_sequence.clear();
		m_visit_sequence = pSource->m_visit_sequence;
		m_visit_time_sequence.clear();
		m_visit_time_sequence = pSource->m_visit_time_sequence;
		m_vehicle_capacity = pSource->m_vehicle_capacity;
		LabelCost = pSource->LabelCost;
	}
	int GetPassengerServiceState(int passenger_id)
	{
		if (passenger_service_state.find(passenger_id) != passenger_service_state.end())
			return passenger_service_state[passenger_id];  // 1 or 2
		else
			return 0;
	}


	void StartCarryingService(int passenger_id, int service_time)
	{
		passenger_carrying_state[passenger_id] = 1;
		m_vehicle_capacity += 1;

		passenger_service_begin_time[passenger_id] = service_time;

	}

	void CompleteCarryingService(int passenger_id, int service_time)
	{
		map<int, int>::iterator iter = passenger_carrying_state.find(passenger_id);
		if (iter != passenger_carrying_state.end())
		{
			passenger_carrying_state.erase(iter);
			m_vehicle_capacity -= 1;
		}

		passenger_service_time[passenger_id] = service_time;

	}

	//Start or Complete service
	void MarkCarryingService(int passenger_id, int node_type, int ServiceTime)
	{
		if (node_type == 1)
			StartCarryingService(passenger_id, ServiceTime);
		if (node_type == 2)
			CompleteCarryingService(passenger_id, ServiceTime);

	}

	bool IsAllServiceComplete()
	{
		if (passenger_carrying_state.size() == 0)
			return true;
		else
			return false;
	}

	//state actually node in SP. label cost
	void CalculateLabelCost(int vehicle_id)
	{
		LabelCost = 0;
		PrimalLabelCost = 0;
		int i = 0;
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			if (it->second == 2)  // complete
			{
				i++;
				int passenger_id = it->first;
				LabelCost -= g_passenger_base_profit[passenger_id];//LBd

				 // the waiting cost of passenger is 0.3
				LabelCost += 0.3*max(0, (passenger_service_begin_time[passenger_id] - g_passenger_order_time[passenger_id]));
				PrimalLabelCost += 0.3*max(0, (passenger_service_begin_time[passenger_id] - g_passenger_order_time[passenger_id]));
			}
		}

		// total travel time
		LabelCost += m_visit_time_sequence[m_visit_time_sequence.size() - 1] - g_vehicle_departure_time_beginning[1];
		PrimalLabelCost += m_visit_time_sequence[m_visit_time_sequence.size() - 1] - g_vehicle_departure_time_beginning[1];


		// the difference between waiting cost and transportation cost

		for (int it = 1; it < m_visit_sequence.size(); it++)
		{
			if (m_visit_sequence[it - 1] == m_visit_sequence[it]) //waiting arc
			{
				LabelCost -= (1 - 0.5)*(m_visit_time_sequence[it] - m_visit_time_sequence[it - 1]); //cost of waiting arc is 0.5
				PrimalLabelCost -= (1 - 0.5)*(m_visit_time_sequence[it] - m_visit_time_sequence[it - 1]);
			}

		}

	}


	//class CVSState  record pax and vehicle's completed service
	void CountPassengerNumberOfVisits(int vehicle_id)
	{

		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			if (it->second == 2)  // complete
			{
				int passenger_id = it->first;
#pragma omp critical
				{
					g_passenger_number_of_visits[passenger_id] += 1;
					g_passenger_vehicle_visit_flag[passenger_id][vehicle_id] = 1;

				}

			}
		}

	}

	std::string generate_string_key()
	{

		stringstream s;

		s << "n";
		s << current_node_id;  // space key
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			s << "_";

			s << it->first << "[" << it->second << "]";

		}
		string converted(s.str());
		return converted;

	}

	bool operator<(const CVSState &other) const
	{
		return LabelCost < other.LabelCost;
	}

};

class C_time_indexed_state_vector
{
public:
	int current_time;


	std::vector<CVSState> m_VSStateVector;
	//state string 1_1_1,state index
	std::map<std::string, int> m_state_map;

	void Reset()
	{
		current_time = 0;
		m_VSStateVector.clear();
		m_state_map.clear();
	}

	int m_find_state_index(std::string string_key)
	{

		if (m_state_map.find(string_key) != m_state_map.end())
		{
			return m_state_map[string_key];
		}
		else
			return -1;  // not found

	}

	void update_state(CVSState new_element)
	{
		std::string string_key = new_element.generate_string_key();//if it is new, string is n100, no state index
		int state_index = m_find_state_index(string_key);

		if (state_index == -1)  // no such state at this time
		{
			// add new state
			state_index = m_VSStateVector.size();
			m_VSStateVector.push_back(new_element);
			m_state_map[string_key] = state_index;
		}
		else
		{//DP
			if (new_element.LabelCost < m_VSStateVector[state_index].LabelCost)
			{
				m_VSStateVector[state_index].Copy(&new_element);
			}

		}

	}

	void Sort()
	{
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		m_state_map.clear(); // invalid
	}

	void SortAndCleanEndingState(int BestKValue)
	{
		if (m_VSStateVector.size() > 2 * BestKValue)
		{
			std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

			m_state_map.clear(); // invalid
			m_VSStateVector.erase(m_VSStateVector.begin() + BestKValue, m_VSStateVector.end());
		}
	}

	float GetBestValue(int DualPriceFlag, int vehicle_id)
	{
		// LabelCost not PrimalCost when sorting
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		if (m_VSStateVector.size() >= 1)
		{
			std::string state_str = m_VSStateVector[0].generate_string_key();
			//0 means least cost
			m_VSStateVector[0].CountPassengerNumberOfVisits(vehicle_id);
			if (DualPriceFlag == 1)
			{
				fprintf(g_pFileDebugLog, "Dual \t{{%s}}; Label Cost %f\n ",
					state_str.c_str(), m_VSStateVector[0].LabelCost);
			}
			else
			{
				fprintf(g_pFileDebugLog, "Primal \t{{%s}}; Label Cost %f\n",
					state_str.c_str(), m_VSStateVector[0].PrimalLabelCost);
			}

			if (DualPriceFlag == 1)
				return m_VSStateVector[0].LabelCost;
			else
				return m_VSStateVector[0].PrimalLabelCost;

		}
		else
			return _MAX_LABEL_COST;
	}

};

//vehicle state at time t
C_time_indexed_state_vector g_time_dependent_state_vector[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];

// for collecting the final feasible states accesible to the depot
C_time_indexed_state_vector g_ending_state_vector[_MAX_NUMBER_OF_VEHICLES];


float g_optimal_time_dependenet_dynamic_programming(
	int vehicle_id,
	int origin_node,
	int departure_time_beginning,
	int departure_time_ending,
	int destination_node,
	int arrival_time_beginning,
	int arrival_time_ending,
	int vehicle_capacity,
	//maximum choose
	int BestKSize,
	int DualCostFlag)
	// time-dependent label correcting algorithm with double queue implementation
{

	if (arrival_time_ending > g_number_of_time_intervals || g_outbound_node_size[origin_node] == 0)
	{
		return _MAX_LABEL_COST;
	}

	for (int p = 1; p < g_number_of_passengers; p++)
	{//release all passengers to this vehicle
		g_passenger_vehicle_visit_flag[p][vehicle_id] = 0;
	}
	//step 2: Initialization for origin node at the preferred departure time, at departure time
	for (int i = 0; i < g_number_of_nodes; i++)
	{
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time

		{
			g_time_dependent_state_vector[vehicle_id][i][t].Reset();
		}

	}
	g_ending_state_vector[vehicle_id].Reset();

	CVSState element;

	element.current_node_id = origin_node;
	g_time_dependent_state_vector[vehicle_id][origin_node][departure_time_beginning].update_state(element);


	// step 3: //dynamic programming
	for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
	{

		for (int n = 0; n < g_number_of_nodes; n++)
		{
			// step 1: sort m_VSStateVector by labelCost for scan best k elements in step2
			g_time_dependent_state_vector[vehicle_id][n][t].Sort();


			// step 2: scan the best k elements
			for (int w_index = 0; w_index < min(BestKSize, g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector.size()); w_index++)
			{
				CVSState* pElement = &(g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector[w_index]);

				int from_node = pElement->current_node_id;//current_node_id is not node n?

				// step 2.1 link from node to toNode
				for (int i = 0; i < g_outbound_node_size[from_node]; i++)
				{
					int to_node = g_outbound_node_id[from_node][i];
					int to_node_passenger_id = g_node_passenger_id[to_node];
					int to_node_type = g_node_type[to_node];
					int link_no = g_outbound_link_no[from_node][i];
					int next_time = max(g_node_timestamp[to_node], t + g_link_free_flow_travel_time[link_no]);

					// step 2.2. check feasibility of node type with the current element
					if (next_time <= arrival_time_ending)
					{

						if (to_node_passenger_id >= 1 && g_activity_node_starting_time[to_node] >= 0 && g_activity_node_ending_time[to_node] >= 0 &&
							g_vehicle_passenger_visit_allowed_flag[vehicle_id][to_node_passenger_id] == 1)
						{// address the passengers' state transitions
						 // passegner activity node: origin or destination

							// skip scanning when the origin/destination nodes arrival time is out of time window
							if (next_time > g_activity_node_ending_time[to_node])
							{
								continue;
							}
							//feasible state transitions
							if ((to_node_type == 1 && pElement->GetPassengerServiceState(to_node_passenger_id) == 0)//pickup
								|| (to_node_type == 2 && pElement->GetPassengerServiceState(to_node_passenger_id) == 1))  // delivery)
							{//origin state is 0, then to 1; origin state is 1, then to 2.
								// step 2.3 label cost updating inside this function

								// for pickup process
								if (to_node_type == 1)
								{
									// skip pickup when the vehicle if on its capacity
									if (g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector[w_index].m_vehicle_capacity >= vehicle_capacity)// Tony: to_node? or from_node?
										continue;
									//waiting
									if (next_time < g_activity_node_starting_time[to_node])
									{
										CVSState new_element;
										new_element.Copy(pElement);
										new_element.MarkCarryingService(to_node_passenger_id, to_node_type, 0);

										new_element.current_node_id = to_node;
										new_element.passenger_service_state[to_node_passenger_id] = to_node_type;

										//for arriving at activity node and begin wait
										new_element.m_visit_time_sequence.push_back(next_time);
										new_element.m_visit_sequence.push_back(to_node);

										//for wait until activity node's depature time
										new_element.m_visit_time_sequence.push_back(g_activity_node_starting_time[to_node]);
										new_element.m_visit_sequence.push_back(to_node);

										new_element.CalculateLabelCost(vehicle_id);

										int link_no = g_outbound_link_no[from_node][i];

										g_time_dependent_state_vector[vehicle_id][to_node][g_activity_node_starting_time[to_node]].update_state(new_element);
										continue;
									}
								}
								if (to_node_type == 2)
								{
									//waiting
									if (next_time < g_activity_node_starting_time[to_node])
									{
										CVSState new_element;
										new_element.Copy(pElement);
										new_element.MarkCarryingService(to_node_passenger_id, to_node_type, g_activity_node_starting_time[to_node]);

										new_element.current_node_id = to_node;
										new_element.passenger_service_state[to_node_passenger_id] = to_node_type;

										//for arriving at activity node and begin wait
										new_element.m_visit_time_sequence.push_back(next_time);
										new_element.m_visit_sequence.push_back(to_node);
										//for wait until activity node's depature time
										new_element.m_visit_time_sequence.push_back(g_activity_node_starting_time[to_node]);
										new_element.m_visit_sequence.push_back(to_node);

										new_element.CalculateLabelCost(vehicle_id);

										int link_no = g_outbound_link_no[from_node][i];

										g_time_dependent_state_vector[vehicle_id][to_node][g_activity_node_starting_time[to_node]].update_state(new_element);
										continue;
									}

								}
								// do not need waiting
								CVSState new_element;
								new_element.Copy(pElement);
								new_element.MarkCarryingService(to_node_passenger_id, to_node_type, next_time);

								new_element.current_node_id = to_node;
								new_element.passenger_service_state[to_node_passenger_id] = to_node_type;
								new_element.m_visit_time_sequence.push_back(next_time);
								new_element.m_visit_sequence.push_back(to_node);

								new_element.CalculateLabelCost(vehicle_id);

								int link_no = g_outbound_link_no[from_node][i];

								g_time_dependent_state_vector[vehicle_id][to_node][next_time].update_state(new_element);
							}
						}

						else if (to_node_passenger_id < 1 && to_node_type == 0)
						{
							CVSState new_element;
							new_element.Copy(pElement);

							new_element.current_node_id = to_node;
							new_element.m_visit_time_sequence.push_back(next_time);
							new_element.m_visit_sequence.push_back(to_node);
							new_element.CalculateLabelCost(vehicle_id);

							int link_no = g_outbound_link_no[from_node][i];
							g_time_dependent_state_vector[vehicle_id][to_node][next_time].update_state(new_element);
						}
						else if (to_node == destination_node)
						{
							CVSState new_element;
							new_element.Copy(pElement);
							//time window of destination_node
							if (next_time < arrival_time_beginning)
							{
								//for arriving at activity node and begin wait
								pElement->m_visit_time_sequence.push_back(next_time);
								pElement->m_visit_sequence.push_back(to_node);
								//for wait until activity node's depature time
								pElement->m_visit_time_sequence.push_back(arrival_time_beginning);
								pElement->m_visit_sequence.push_back(to_node);
								pElement->CalculateLabelCost(vehicle_id);

							}
							else
							{
								new_element.m_visit_time_sequence.push_back(next_time);
								new_element.m_visit_sequence.push_back(to_node);
								new_element.CalculateLabelCost(vehicle_id);
								pElement->m_visit_time_sequence.push_back(next_time);
								pElement->m_visit_sequence.push_back(to_node);
								pElement->CalculateLabelCost(vehicle_id);
							}
							g_ending_state_vector[vehicle_id].update_state(*pElement);
							g_ending_state_vector[vehicle_id].SortAndCleanEndingState(BestKSize);
						}
					}

				}
			}
		}  // for all nodes
	} // for all time t


// no backf
	return g_ending_state_vector[vehicle_id].GetBestValue(DualCostFlag, vehicle_id);
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
	int interval_node_no = 1;

	// step 1: read node file
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type = 0;
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
			if (node_type != 0)
			{
				cout << "node_type in input_node.csv should be 0!" << endl;
			}
			g_node_type[interval_node_no] = node_type;

			parser.GetValueByFieldName("x", X);
			parser.GetValueByFieldName("y", Y);

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
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

			if (from_node_id <= 0)
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (to_node_id <= 0)
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

				g_link_free_flow_travel_time[g_number_of_links] = max(1, travel_time + 0.5);   // at least 1 min, round to nearest integers
				g_link_free_flow_travel_time_float_value[g_number_of_links] = travel_time;

				g_link_link_length[g_number_of_links] = link_length;
				g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
				g_link_mode_code[g_number_of_links] = mode_code;
				g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
				g_link_speed[g_number_of_links] = speed;

				// increase the link counter by 1
				g_number_of_links++;

				if (g_number_of_links % 1000 == 0)
					cout << "reading " << g_number_of_links << " links.. " << endl;
			}
		}

		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);
		parser.CloseCSVFile();
	}


	//step 3: read agent file including vehicle and passenger
	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int agent_id;
			parser.GetValueByFieldName("agent_id", agent_id);

			if (agent_id == 0)  // break for empty line
				break;

			int agent_type = 0;
			parser.GetValueByFieldName("agent_type", agent_type);
			int external_from_node_id;
			int external_to_node_id;
			int from_node_id;
			int to_node_id;

			if (agent_type == 0) //passenger
			{
				int pax_no = g_number_of_passengers + 1;

				g_internal_agent_no_map[agent_id] = pax_no;
				g_external_passenger_id_map[pax_no] = agent_id;


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
				g_passenger_departure_time_ending[pax_no] = max(0, departure_time_window) + g_passenger_departure_time_beginning[pax_no];
				g_passenger_order_time[pax_no] = g_passenger_departure_time_beginning[pax_no];
				parser.GetValueByFieldName("arrival_time_start", g_passenger_arrival_time_beginning[pax_no]);

				int arrival_time_window = 0;
				parser.GetValueByFieldName("arrival_time_window", arrival_time_window);

				g_passenger_arrival_time_ending[pax_no] = g_passenger_arrival_time_beginning[pax_no] + arrival_time_window;

				g_passenger_arrival_time_ending[pax_no] = max(g_passenger_arrival_time_ending[pax_no], g_passenger_arrival_time_beginning[pax_no]);
				g_number_of_time_intervals = max(g_passenger_arrival_time_ending[pax_no] + 10, g_number_of_time_intervals);

				g_number_of_time_intervals = min(g_number_of_time_intervals, _MAX_NUMBER_OF_TIME_INTERVALS);

				//add virtual node and link for passenger
				int new_artifical_pasenger_origin_id = g_add_new_node_origin(pax_no, g_passenger_departure_time_beginning[pax_no], g_passenger_departure_time_ending[pax_no]);
				g_add_new_link(g_passenger_origin_node[pax_no], new_artifical_pasenger_origin_id, pax_no);  // pick up link
				g_add_new_link(new_artifical_pasenger_origin_id, g_passenger_origin_node[pax_no], pax_no);

				int new_artifical_pasenger_destination_id = g_add_new_node_destination(pax_no, g_passenger_arrival_time_beginning[pax_no], g_passenger_arrival_time_ending[pax_no]);
				g_add_new_link(g_passenger_destination_node[pax_no], new_artifical_pasenger_destination_id, pax_no*(-1));  // delivery link
				g_add_new_link(new_artifical_pasenger_destination_id, g_passenger_destination_node[pax_no], pax_no*(-1));

				parser.GetValueByFieldName("base_profit", g_passenger_base_profit[pax_no]);

				int request_vehicle_id = -1;
				parser.GetValueByFieldName("requested_vehicle_id", request_vehicle_id);

				std::vector<int> prohibited_vehicle_id;
				string prohibited_vehicle_id_list;
				parser.GetValueByFieldName("prohibited_vehicle_id_list", prohibited_vehicle_id_list);

				g_number_of_passengers++;
			}
			else
			{  // vehicle

				int vehicle_no = g_number_of_vehicles + 1;

				g_external_vehicle_id_map[vehicle_no] = agent_id;

				if (agent_type == 1)
				{

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

					if (g_vehicle_arrival_time_ending[vehicle_no] < g_vehicle_departure_time_beginning[vehicle_no] + 13)  // we should use a shortest path travel time to check.
					{
						cout << "warning: Arrival time for vehicle " << vehicle_no << " should be " << g_vehicle_departure_time_beginning[vehicle_no] + 120 << endl;
						g_vehicle_arrival_time_ending[vehicle_no] = g_vehicle_departure_time_beginning[vehicle_no] + 60;
						//				g_ProgramStop();
					}

					//add virtual node and link for vehicle
					int new_artifical_vehicle_origin_id = g_add_new_node(vehicle_no, g_vehicle_departure_time_beginning[vehicle_no], g_vehicle_departure_time_ending[vehicle_no]);
					g_add_new_link(new_artifical_vehicle_origin_id, g_vehicle_origin_node[vehicle_no], 100);
					g_vehicle_depot_origin_node[vehicle_no] = g_number_of_nodes;

					int new_artifical_vehicle_destination_id = g_add_new_node(vehicle_no, g_vehicle_arrival_time_beginning[vehicle_no], g_vehicle_arrival_time_ending[vehicle_no]);
					g_add_new_link(g_vehicle_destination_node[vehicle_no], new_artifical_vehicle_destination_id, 101);  // delivery link
					g_vehicle_depot_destination_node[vehicle_no] = g_number_of_nodes;

					g_activity_node_flag[g_vehicle_depot_origin_node[vehicle_no]] = 1;
					g_activity_node_flag[g_vehicle_depot_destination_node[vehicle_no]] = 1;
					g_activity_node_ending_time[g_vehicle_depot_origin_node[vehicle_no]] = g_vehicle_departure_time_ending[vehicle_no];
					g_activity_node_ending_time[g_vehicle_depot_destination_node[vehicle_no]] = g_vehicle_arrival_time_ending[vehicle_no];

					g_vehicle_capacity[vehicle_no] = -1;

					int capacity = 1;
					parser.GetValueByFieldName("capacity", capacity);
					g_vehicle_capacity[vehicle_no] = max(1, capacity);
					if (g_vehicle_capacity[vehicle_no] < 0)
					{
						cout << "Vehicle data must have values in field capacity in file input_agent.csv!" << endl;
						g_ProgramStop();
					}
					parser.GetValueByFieldName("VOIVTT_per_hour", g_VOIVTT_per_hour[vehicle_no]);
					parser.GetValueByFieldName("VOWT_per_hour", g_VOWT_per_hour[vehicle_no]);

					if (g_max_vehicle_capacity < g_vehicle_capacity[vehicle_no])
						g_max_vehicle_capacity = g_vehicle_capacity[vehicle_no];

				}
				g_number_of_vehicles++;
				g_updated_number_of_time_intervals = g_number_of_time_intervals;
			}
		}

		parser.CloseCSVFile();
	}

	fprintf(g_pFileOutputLog, "number of passengers =,%d\n", g_number_of_passengers);
	fprintf(g_pFileOutputLog, "number of vehicles =,%d\n", g_number_of_vehicles);

	cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_passengers << " passengers, " << g_number_of_vehicles << "vehicles" << endl;
	fprintf(g_pFileDebugLog, "Network has %d nodes, %d links, %d  passengers, %d vehicles\n\n",
		g_number_of_nodes, g_number_of_links, g_number_of_passengers, g_number_of_vehicles);

	//output activity service link(virtual) in debug.txt
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

	fprintf(g_pFileDebugLog, "\n");

}


bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables()  // with varaible y only
{

	cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	g_SolutionStartTime = CTime::GetCurrentTime();


	float g_best_upper_bound = 99999;
	float g_best_lower_bound = -99999;

	g_number_of_LR_iterations = 1;
	float StepSize = 1;

	//loop for each LR iteration
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{
		if ((LR_iteration + 1) % 5 == 0)
			std::cout << LR_iteration + 1 << "/" << g_number_of_LR_iterations << std::endl;

		// reset the vertex visit count
		double LR_global_lower_bound = 0;

		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;

		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_passenger_number_of_visits[p] = 0;
		}

		for (int v = 1; v <= g_number_of_vehicles; v++)
		{

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
				local_vehicle_passenger_additional_profit[v][p] = 0;

			}
		}

		//#pragma omp parallel for
		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			fprintf(g_pFileDebugLog,
				"\nDebug: LB iteration %d, Vehicle %d performing DP: origin %d -> destination %d\n ",
				LR_iteration,
				v,
				g_vehicle_origin_node[v],
				g_vehicle_destination_node[v]);

			float path_cost_by_vehicle_v =
				g_optimal_time_dependenet_dynamic_programming  //DP for each vehicle
				(
					v,
					g_vehicle_depot_origin_node[v],
					g_vehicle_departure_time_beginning[v],
					g_vehicle_departure_time_ending[v],
					g_vehicle_depot_destination_node[v],
					g_vehicle_arrival_time_beginning[v],
					g_vehicle_arrival_time_ending[v],
					g_vehicle_capacity[v],
					2,
					1);

			if (path_cost_by_vehicle_v < _MAX_LABEL_COST)
				LR_global_lower_bound += path_cost_by_vehicle_v;

			//fprintf(g_pFileDebugLog, "LR_global_lower_bound += path_cost_by_vehicle_v, %f, %f", path_cost_by_vehicle_v, LR_global_lower_bound);

		}

		for (int p = 1; p <= g_number_of_passengers; p++)
		{

			LR_global_lower_bound += g_passenger_base_profit[p];

		}

		fprintf(g_pFileDebugLog, "LR_global_lower_bound = , %f", LR_global_lower_bound);
		g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound);  // keep the best lower bound till current iteration

		fprintf(g_pFile_Output_paxprofitLog, "\n%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,",
			LR_iteration, StepSize, g_passenger_base_profit[1],
			g_passenger_base_profit[2], g_passenger_base_profit[3], g_passenger_base_profit[4]
			, g_passenger_base_profit[5], g_passenger_base_profit[6], g_passenger_base_profit[7]
			, g_passenger_base_profit[8], g_passenger_base_profit[9], g_passenger_base_profit[10], g_best_lower_bound);

		//output vehicle's path_node_seq and path_time_seq for each iteration of lower bound
		fprintf(g_pFile_OutputAgentLog, "\n%d,%f,",
			LR_iteration, StepSize);
		for (int j = 1; j <= g_number_of_vehicles; j++)
		{
			if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
				continue;

			cout << "vehicle number = " << j << endl;

			//output vehicle passing node sequence
			fprintf(g_pFile_OutputAgentLog, "%d;", g_vehicle_depot_origin_node[j]);
			for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence.size(); i++)
			{
				if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]] == 0) //physical nodes
				{
					fprintf(g_pFile_OutputAgentLog, "%d;", g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]]);
				}
				else //pickup and drop-off nodes
				{
					fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]);
				}
			}

			fprintf(g_pFile_OutputAgentLog, ",");

			//output vehicle passing node time sequence
			fprintf(g_pFile_OutputAgentLog, "%d;", g_vehicle_departure_time_beginning[j]);
			for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence.size(); i++)
			{

				fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i]);
			}
			fprintf(g_pFile_OutputAgentLog, ",");
		}

		//update step size for the next LR_iteration
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			StepSize = 1 / (LR_iteration + 1.0f);
			g_minimum_subgradient_step_size = 0.1;
			if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
			{
				StepSize = g_minimum_subgradient_step_size;
			}

			int constant = 10;
			g_passenger_base_profit[p] -= constant*StepSize * (g_passenger_number_of_visits[p] - 1);
		}

	}
	// end of LR iterations

	// generate upper bound
	fprintf(g_pFileDebugLog, "\nGenerate upper bound\n");

	double LR_global_upper_bound = 0;
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		g_passenger_number_of_visits[p] = 0;
	}

	for (int v = 1; v <= g_number_of_vehicles; v++)
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
		}


	// sequential DP for each vehicle, based on the LR prices
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_dynamic_programming
			(
				v,
				g_vehicle_depot_origin_node[v],
				g_vehicle_departure_time_beginning[v],
				g_vehicle_departure_time_ending[v],
				g_vehicle_depot_destination_node[v],
				g_vehicle_arrival_time_beginning[v],
				g_vehicle_arrival_time_ending[v],
				g_vehicle_capacity[v],
				5,
				0);
		LR_global_upper_bound += path_cost_by_vehicle_v;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{

			if (g_passenger_number_of_visits[p] == 0)
				LR_global_upper_bound += 30;
		}


		fprintf(g_pFileDebugLog, "LR_global_upper_bound += path_cost_by_vehicle_%d, %f, %f", v, path_cost_by_vehicle_v, LR_global_upper_bound);

		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			for (int i = 0; i < g_number_of_nodes; i++)
			{
				for (int j = 0; j < g_time_dependent_state_vector[v][i][t].m_VSStateVector.size(); j++)
				{
					string temp = g_time_dependent_state_vector[v][i][t].m_VSStateVector[j].generate_string_key();
					fprintf(g_pFile_PathLog, "\n%d,%d,%d,%s",
						v, i, t, g_time_dependent_state_vector[v][i][t].m_VSStateVector[j].generate_string_key().c_str());

				}
			}
		}
		if (v < g_number_of_vehicles)  // mark the passsengers have been visited
		{
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				if (g_passenger_number_of_visits[p] >= 1)
				{
					g_vehicle_passenger_visit_allowed_flag[v + 1][p] = 0;  // not allowed to visit
					fprintf(g_pFileDebugLog, "\nupper bound generation, for vehicle %d, pax %d is not allowed or no needed to serve",
						v + 1, p);

				}
			}
		}
	}  //end of vehicle v

	g_best_upper_bound = min(g_best_upper_bound, LR_global_upper_bound);  // keep the best lower bound till current iteration

	CTimeSpan ctime = CTime::GetCurrentTime() - g_SolutionStartTime;
	cout << "\nComputational time:," << ctime.GetTotalSeconds() << endl;

	//output vehicle's path_node_seq and path_time_seq for Upperbound
	fprintf(g_pFile_OutputAgentLog, "\n%s,%s,",
		"Upperbound:", " ");
	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;

		//output vehicle passing node sequence
		fprintf(g_pFile_OutputAgentLog, "%d;", g_vehicle_depot_origin_node[j]);
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence.size(); i++)
		{
			if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]] == 0) //physical nodes
			{
				fprintf(g_pFile_OutputAgentLog, "%d;", g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]]);
			}
			else //pickup and drop-off nodes
			{
				fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]);
			}
		}
		fprintf(g_pFile_OutputAgentLog, ",");

		//output vehicle passing node time sequence
		fprintf(g_pFile_OutputAgentLog, "%d;", g_vehicle_departure_time_beginning[j]);
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence.size(); i++)
		{

			fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i]);
		}
		fprintf(g_pFile_OutputAgentLog, ",");
	}

	//output the vehicle path matrix information
	fprintf(g_pFile_Vehicle_Path_PassengerLog, "%s,%s,%s,%s,%s,%s,%s\n", "vehicle_id", "path_id", "node_id", "visit_time", "passenger_id", "link_id", "path_cost");

	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;

		//output vehicle passing node sequence
		int temp_pax_id = -1;
		int temp_link_id = -1;
		int temp_from_node_no = 0;

		fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_vehicle_depot_origin_node[j], g_vehicle_departure_time_beginning[j], temp_pax_id, temp_link_id, g_best_upper_bound);

		if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0]] == 0) //physical nodes
		{
			fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0]],
				g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0], -1, -1, g_best_upper_bound);
		}
		else if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0]] == 3) //vehicle depot
		{
			fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0],
				g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0], -1, -1
				, g_best_upper_bound);
		}
		else
		{
			fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0],
				g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0], g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[0]], -1
				, g_best_upper_bound);
		}

		for (int i = 1; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence.size(); i++)
		{
			temp_link_id = -1;
			temp_from_node_no = g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i - 1];

			if (g_node_type[temp_from_node_no] == 0)
			{
				for (int n = 0; n < g_outbound_node_size[temp_from_node_no]; n++)
				{
					if (g_outbound_node_id[temp_from_node_no][n] == g_internal_node_no_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]])
					{
						temp_link_id = g_outbound_link_no[temp_from_node_no][n];
						break;
					}
				}
			}

			if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]] == 0) //physical nodes
			{
				fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]],
					g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i], -1, temp_link_id, g_best_upper_bound);
			}
			else if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]] == 3) //vehicle depot
			{
				fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i],
					g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i], -1, -1
					, g_best_upper_bound);
			}
			else
			{
				fprintf(g_pFile_Vehicle_Path_PassengerLog, "%d,%d,%d,%d,%d,%d,%.3f\n", j, 1, g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i],
					g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i], g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_sequence[i]], -1
					, g_best_upper_bound);
			}
		}
	}

	fprintf(g_pFileDebugLog, "Summary: Lower Bound = %f, upper Bound = %f, gap = %f, relative_gap = %.3f%%\n",

		g_best_lower_bound,
		g_best_upper_bound,
		(g_best_upper_bound - g_best_lower_bound),
		(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0

	);
	fprintf(g_pFileDebugLog, "******************************************************************************************************************\n\n");
	fprintf(g_pFileOutputLog, "%f,%f,%f,%.3f%%,",

		g_best_lower_bound,
		g_best_upper_bound,
		(g_best_upper_bound - g_best_lower_bound),
		(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
	);
	cout << "End of Lagrangian Iteration Process " << endl;

	return true;
}


int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	//output file for Lagrandian and DP updating process
	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}


	//output file for vehicle's path_node_seq and path_time_seq
	g_pFile_OutputAgentLog = fopen("output_agent.csv", "w");
	if (g_pFile_OutputAgentLog == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputAgentLog, "LR_iteration,Stepsize,path_node_seq,path_ime_sequence");

	//output file for the updating of profits of passengers during different iteration
	g_pFile_Output_paxprofitLog = fopen("Output_paxprofitLog.csv", "w");
	if (g_pFile_Output_paxprofitLog == NULL)
	{
		cout << "File Output_paxprofitLog.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_Output_paxprofitLog, "iteration,stepSize,p1' profit,p2's profit,p3's profit,p4's profit,p5's profit,p6's profit,p7's profit,p8's profit,p9's profit,p10's profit,LowerBoundCost");

	//output file for vehicle state update during DP process
	g_pFile_PathLog = fopen("PathLog.csv", "w");
	if (g_pFile_PathLog == NULL)
	{
		cout << "File PassengerProfit.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_PathLog, "vehicle No.,Time,state");

	//output overview of your program data
	g_pFileOutputLog = fopen("output_solution.csv", "w");
	if (g_pFileOutputLog == NULL)
	{
		cout << "File output_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	g_pFile_Vehicle_Path_PassengerLog = fopen("output_VehicleMatrix.csv", "w");

	if (g_pFile_Vehicle_Path_PassengerLog == NULL)
	{
		cout << "File output_VehicleMatrix.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//read input date including input_node.csv, input_link.csv and input_agent.csv
	g_ReadInputData();

	// definte timestamps
	clock_t start_t, end_t, total_t;
	start_t = clock();

	//Lagrangian function---the major function
	g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables();


	end_t = clock();

	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %ld milliseconds\n", total_t);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%ld, milliseconds\n", total_t);

	//output_node.csv with virtual node
	g_pFile_OutputNodeLog = fopen("output_node.csv", "w");
	if (g_pFile_OutputNodeLog == NULL)
	{
		cout << "File output_node.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputNodeLog, "node_id,node_type\n");
	for (int i = 1; i <= g_number_of_nodes; i++)
	{
		fprintf(g_pFile_OutputNodeLog, "%d,%d\n", i, g_node_type[i]);
	}

	//output_link.csv with virtual link
	g_pFile_OutputLinkLog = fopen("output_link.csv", "w");
	if (g_pFile_OutputLinkLog == NULL)
	{
		cout << "File output_link.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputLinkLog, "from_node_id,to_node_id,link_type\n");
	for (int i = 0; i < g_number_of_links; i++)
	{
		fprintf(g_pFile_OutputLinkLog, "%d,%d,%d\n", g_link_from_node_number[i], g_link_to_node_number[i], g_link_service_code[i]);
	}//if link is virtual link, passenger:origin pax_no destination -pax_no ; vehicle:origin 100 destination 101


	fclose(g_pFile_OutputNodeLog);
	fclose(g_pFile_OutputLinkLog);
	fclose(g_pFile_OutputAgentLog);
	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);
	fclose(g_pFile_Output_paxprofitLog);
	fclose(g_pFile_PathLog);


	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	getchar();
	return 1;
}


