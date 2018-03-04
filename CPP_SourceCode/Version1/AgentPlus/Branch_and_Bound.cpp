
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <queue>
#include "AgentPlus.h"
#include "CSVParser.h"
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define N_PAX _MAX_NUMBER_OF_PASSENGERS
#define N_VEH _MAX_NUMBER_OF_VEHICLES

ofstream BBDebugfile;

float g_Global_LowerBound=-99999;
float g_Global_UpperBound=99999;

int g_node_id = 0;

//int costMatrix[N_PAX][N_VEH] =
//{
//	{ 9, 2, 7, 8 },
//	{ 6, 4, 3, 7 },
//	{ 5, 8, 1, 8 },
//};

// state space tree node
class Node
{
public:
	// stores parent node of current node
	// helps in tracing path when answer is found
	Node* parent;
	int node_id; 
	// contains cost for ancestors nodes including current node
	float UBCost;

	// contains least lower bound promising cost
	float BestLBCost;

	// contain worker number
	int VehicleID; // vehicle to be considered to evalaute
	// contains passsenger ID
	int PassengerID;

	// Boolean array bAssignedPax will contains
	// info about available passenger

	//in one Node
	VRP_exchange_data l_vrp_data;

	Node()
	{
		VehicleID = -1;
		// contains passsenger ID
		PassengerID = -1;
	}

	int CalculateCost()
	{
		int cost = 0;

		// add cost of next vehicle

		g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables(&l_vrp_data);

		if (l_vrp_data.UBCost < g_Global_UpperBound)
		{
			BBDebugfile << "!!!   Cal: update upper bound from (node UB)" << g_Global_UpperBound << " to a new " << l_vrp_data.UBCost << endl;
			g_Global_UpperBound = l_vrp_data.UBCost;

		}


		BBDebugfile << "   Node " << node_id << " with LB =" << l_vrp_data.LBCost << " UB= " << l_vrp_data.UBCost  << endl;

		bool bFeasibleLBSolution = true;
		for (int p = 1; p < l_vrp_data.V2PAssignmentVector.size(); p++)
		{
			int competing_vehicle_size = l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector.size();
			if (competing_vehicle_size >= 2)
			{
				for (int v = 0; v < competing_vehicle_size; v++)
				{
					BBDebugfile << "   Cal: pax " << p << " with competing v[ " << v << "]=" <<l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector[v] << endl;

				}

			}

			if (competing_vehicle_size != 1)  //0 or more than 2
				bFeasibleLBSolution = false;
		}

		if (bFeasibleLBSolution)
		{
			BBDebugfile << "   Cal: We have a good feasible LB solution! " << endl;
			
			if (l_vrp_data.LBCost < g_Global_UpperBound)
			{
				BBDebugfile << "!!!   Cal: update upper bound from (node LB)" << g_Global_UpperBound << " to " << l_vrp_data.LBCost << endl;
				g_Global_UpperBound = l_vrp_data.LBCost;

			}


		}

		return cost;
	}
};

// Function to allocate a new search tree node
// Here Person p is assigned to vehicle v
Node* newNode(VRP_exchange_data vrp_data, Node* parent)
{
	Node* node = new Node;
	node->node_id = g_node_id++;

	node->l_vrp_data.CopyAssignmentInput(vrp_data.V2PAssignmentVector);

	node->parent = parent;
	node->l_vrp_data = vrp_data;

	node->l_vrp_data.BBNodeNo = node->node_id;


	return node;
}

// Function to calculate the least promising cost
// of node after vehicle v is bAssigned to passenger p.

// Comparison object to be used to order the heap
struct comp
{
	bool operator()(const Node* lhs,
	const Node* rhs) const
	{
		return lhs->UBCost > rhs->UBCost;
	}
};

// print assignments
void printAssignments(Node *min)
{
	if (min->parent == NULL)
		return;

	printAssignments(min->parent);

	//BBDebugfile << "Assign vehicle " << min->VehicleID << " to pax " << min->PassengerID << " with cost " << costMatrix[min->VehicleID][min->PassengerID] << endl;

}

// Finds minimum cost using Branch and Bound.
float findMinCost(int max_number_of_nodes)
{
	g_Global_LowerBound = -99999;
	g_Global_UpperBound = 99999;

	// Create a priority queue to store live nodes of
	// search tree;
	priority_queue<Node*, std::vector<Node*>, comp> pq;

	// initailize heap to dummy node with cost 0
	VRP_exchange_data vrp_data;

	Node* root = newNode(vrp_data, NULL);
	root->CalculateCost();

	// Add dummy node to list of live nodes;
	pq.push(root);

	// Finds a active node with least cost,
	// add its childrens to list of active nodes and
	// finally deletes it from the list.
	int search_count = 0;
	while (!pq.empty())
	{
		// Find a active node with least estimated cost
		Node* min_Node = pq.top();

		BBDebugfile << "Find an active node " << min_Node->node_id << " with a  UB cost = " << min_Node->l_vrp_data.UBCost << " LB cost = " << min_Node->l_vrp_data.LBCost << endl;

		// The found node is deleted from the list of
		// live nodes
		pq.pop();

		if (search_count >= max_number_of_nodes)
			break;

		search_count++; 
		if (min_Node->l_vrp_data.LBCost > g_Global_UpperBound + 0.00001)//if node cost > current upper bound, cut this node
		{
			BBDebugfile << "   cut node " << min_Node->node_id << " as LBCost = " << min_Node->l_vrp_data.LBCost << " > current best upper bound " << g_Global_UpperBound << endl;
			continue;
		}


		//branching based competing vehicles

		bool bBranchedFlag = false;
		for (int p = 1; p < min_Node->l_vrp_data.V2PAssignmentVector.size(); p++) // for each p
		{
			int competing_vehicle_size = min_Node->l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector.size();
			if (competing_vehicle_size >= 2) // if there are more than 2 competing vehicles, branching
			{
				// first vi inclusive braches 
				for (int vi = 0; vi < competing_vehicle_size; vi++)
				{
					BBDebugfile << "   Branch: pax " << p << " with competing v[ " << vi << "]=" << min_Node->l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector[vi] << endl;

					// create a new tree node
					Node* child = newNode(min_Node->l_vrp_data, min_Node);
					int veh_id = min_Node->l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector[vi];
					child->VehicleID = veh_id;
					child->PassengerID = p;

					BBDebugfile << "   New Node " << child->node_id << " with vehicle " << child->VehicleID << " to Pax " << child->PassengerID << endl;

					child->l_vrp_data.AddP2VAssignment(p, veh_id);  // designate p to veh_id

					for (int vj = 0; vj < competing_vehicle_size; vj++)
					{
					  if(vj!=vi)  // for the other vehicle not this vehicle
					  { 
						  int new_vehicle_id = min_Node->l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector[vj];
						  child->VehicleID = veh_id;
						  child->l_vrp_data.AddProhibitedAssignment(p,
							  min_Node->l_vrp_data.V2PAssignmentVector[p].input_prohibited_vehicle_id_vector,
							  new_vehicle_id);  // prohibit the other vehicles to access p
					  }
					
					}
					child->CalculateCost();
					// Add child to list of live nodes;
					pq.push(child);
				}

				// last exclusive branch

				BBDebugfile << "   pax " << p << " with exclusive branches "<< endl;

				Node* child = newNode(min_Node->l_vrp_data, min_Node);
				for (int vj = 0; vj < competing_vehicle_size; vj++)
				{ 
					int new_vehicle_id = min_Node->l_vrp_data.V2PAssignmentVector[p].output_competting_vehicle_id_vector[vj];
					child->l_vrp_data.AddProhibitedAssignment(p, min_Node->l_vrp_data.V2PAssignmentVector[p].input_prohibited_vehicle_id_vector, new_vehicle_id);
				}

				child->CalculateCost();
				// Add child to list of live nodes;
				pq.push(child);

				break;  // branch once and break
			}
			else if (competing_vehicle_size == 0) // if there are 0 vehicle to serve a pax
			{
			
			}
		}
		//	}
		
	}

	return g_Global_UpperBound;
}

int g_Brand_and_Bound()
{
	BBDebugfile.open("BBDebug.txt");
	
	//The maximum node number is 100
	BBDebugfile << "\nOptimal Cost is " << findMinCost(100) << " Number of Nodes checked = " << g_node_id << endl;;

	BBDebugfile.close();

	return 0;
}

