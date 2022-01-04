#pragma once

#include "resource.h"
#include "stdafx.h"
#define _MAX_LABEL_COST 99999

#define _MAX_NUMBER_OF_NODES 200
#define _MAX_NUMBER_OF_LINKS 4000
#define _MAX_NUMBER_OF_TIME_INTERVALS 1400
#define _MAX_NUMBER_OF_STEP_INTERVALS 14000

#define _MAX_NUMBER_OF_VEHICLES 40
#define _MAX_NUMBER_OF_PASSENGERS 10
#define _MAX_NUMBER_OF_STATES 700

#define _MAX_NUMBER_OF_OUTBOUND_NODES 200
#define _MAX_NUMBER_OF_INBOUND_NODES 200
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

using namespace std;

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
void Deallocate5DDynamicArray(T***** dArray, int nM, int nX, int nY, int nW)
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