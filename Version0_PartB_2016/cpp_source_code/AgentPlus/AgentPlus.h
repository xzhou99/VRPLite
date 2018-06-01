#pragma once


#include "stdafx.h"
#include <vector>
using namespace std; 
#define _MAX_LABEL_COST 99999

#define _MAX_NUMBER_OF_NODES 3000
#define _MAX_NUMBER_OF_LINKS 6000
#define _MAX_NUMBER_OF_TIME_INTERVALS 1400

#define _MAX_NUMBER_OF_VEHICLES 200
#define _MAX_NUMBER_OF_PASSENGERS 70
#define _MAX_NUMBER_OF_STATES 300
extern int g_number_of_physical_vehicles;
extern int g_number_of_passengers;


#define _MAX_NUMBER_OF_OUTBOUND_NODES 10
extern void g_STVComputing();
extern void g_DynamicProgramming_transition_arc(int k);

extern int g_vehicle_path_node_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_link_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_time_sequence[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_TIME_INTERVALS];
extern int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES];
extern float g_external_link_time_dependent_toll[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

extern float g_optimal_time_dependenet_label_correcting(float arc_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS],
	int origin_node, int departure_time_beginning, int departure_time_ending, int destination_node, int arrival_time_beginning, int arrival_time_ending,
	int &path_number_of_nodes,
	int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS], int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS],
	int travel_time_calculation_flag,
	float &travel_time_return_value);

extern void g_TrafficSimulation(int SimulationStartTime, int SimulationEndTime, int b_spatial_capacity_flag);
extern int g_Brand_and_Bound();




class V2PAssignment
{
public:
	int input_assigned_vehicle_id;
	std::vector <int> input_prohibited_vehicle_id_vector;
	std::vector <int> output_competting_vehicle_id_vector;


	V2PAssignment()
	{
		input_assigned_vehicle_id = -1;
	}

	bool AddCompettingVehID(int vehicle_id)
	{
		for (int i = 0; i < output_competting_vehicle_id_vector.size(); i++)  // test if the vehicle id is already in the lest
		{
			if (output_competting_vehicle_id_vector[i] == vehicle_id)
				return false;
		}

		output_competting_vehicle_id_vector.push_back(vehicle_id);

	}
};


class VRP_exchange_data
{
public:
	int BBNodeNo;
	float UBCost;
	float LBCost;

	std::vector <V2PAssignment>  V2PAssignmentVector;

		bool CopyAssignmentInput(std::vector <V2PAssignment> ExternalV2PAssignmentVector)
	{
		for (int p = 0; p <= g_number_of_passengers; p++)
		{
		
			V2PAssignmentVector[p].input_assigned_vehicle_id = ExternalV2PAssignmentVector[p].input_assigned_vehicle_id;
				for (int i = 0; i < ExternalV2PAssignmentVector[p].input_prohibited_vehicle_id_vector.size(); i++)
			{
					V2PAssignmentVector[p].input_prohibited_vehicle_id_vector.push_back(ExternalV2PAssignmentVector[p].input_prohibited_vehicle_id_vector[i]);

			}
		}
		return true;
	}

	bool AddP2VAssignment(int p, int v)
	{

		V2PAssignmentVector[p].input_assigned_vehicle_id = v;
		return true;
	}

	bool AddProhibitedAssignment(int p, std::vector <int> prohibited_vehicle_id_vector)
	{

		V2PAssignmentVector[p].input_prohibited_vehicle_id_vector = prohibited_vehicle_id_vector;
		return true;
	}
	bool reset_output()
	{
		LBCost = -99999;
		UBCost = 99999;
		for (int j = 0; j <= g_number_of_passengers; j++)
		{
			V2PAssignmentVector[j].output_competting_vehicle_id_vector.clear();
		}
		return true;
	}
	bool bV2P_Prohibited(int pax_id, int vehicle_id)
	{

		for (int i = 0; i < V2PAssignmentVector[pax_id].input_prohibited_vehicle_id_vector.size(); i++)
		{
			if (V2PAssignmentVector[pax_id].input_prohibited_vehicle_id_vector[i] == vehicle_id)
				return true;

		}

		return false;

	}
	VRP_exchange_data()
	{
		BBNodeNo = -1;
		LBCost = -99999;
		UBCost = 99999;
		for (int j = 0; j <= g_number_of_passengers; j++)
		{

			V2PAssignment element;
			V2PAssignmentVector.push_back(element);
		}
	}

};
template <typename T>
T **AllocateDynamicArray(int nRows, int nCols, int initial_value = 0)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();

	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int j = 0; j < nCols; j++)
		{
			dynamicArray[i][j] = initial_value;
		}
	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}

template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		if (x % 1000 ==0)
		{
			cout << "allocating 3D memory for " << x << endl;
		}


		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	for (int x = 0; x < nX; x++)
	for (int y = 0; y < nY; y++)
	for (int z = 0; z < nZ; z++)
	{
		dynamicArray[x][y][z] = 0;
	}
	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}



template <typename T>
T ****Allocate4DDynamicArray(int nM, int nX, int nY, int nZ)
{
	T ****dynamicArray;

	dynamicArray = new (std::nothrow) T***[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		if (m%100 ==0)
		cout << "allocating 4D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T**[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T*[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				delete[] dArray[m][x][y];
			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

extern void g_ProgramStop();


template <typename T>
T *****Allocate5DDynamicArray(int nM, int nX, int nY, int nZ, int nW)
{
	T *****dynamicArray;

	dynamicArray = new (std::nothrow) T****[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		cout << "allocating 5D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T***[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T**[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T*[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}

				for (int w = 0; w < nW; w++)
				{
					dynamicArray[m][x][y][w] = new (std::nothrow) T[nW];
					if (dynamicArray[m][x][y][w] == NULL)
					{
						cout << "Error: insufficient memory.";
						g_ProgramStop();
					}
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate5DDynamicArray(T**** dArray, int nM, int nX, int nY, int nW)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				for (int w = 0; w < nW; w++)
				{
					delete[] dArray[m][x][y][w];
				}

				delete[] dArray[m][x][y];


			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}



extern bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables(VRP_exchange_data* local_vrp_data);  // with varaible y only;
