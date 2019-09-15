#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml)
{
	int counter = 0;
	for (Model::Node node : this->Nodes())
	{
		m_Nodes.push_back(Node(counter, this, node));
		counter++;
	}
	CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap()
{
	for(auto &road : this->Roads())
	{
		if(road.type != Model::Road::Type::Footway) // For each reference &road in the vector, check that the type is not a footway: 
		{
			for(auto node_idx : this->Ways()[road.way].nodes) // Loop over each node_idx in the way that the road belongs to
			{
				if(node_to_road.find(node_idx) == node_to_road.end()) // If the node index is not in the node_to_road hashmap yet, set the value for the node_idx key to be an empty vector of const Model::Road* objects.
					node_to_road[node_idx] = std::vector<const Model::Road*> {};
				node_to_road[node_idx].push_back(&road);
			}
		}
	}
}

/*
	The goal of FindNeighbor is to return a pointer to the closest unvisited node from a vector of node indices, where the distance is measured to the current node (this).
	This method will be used later to help find all of the possible next steps in the A* search.
*/
RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) // RouteModel::Node:: Here we are inside the Node Class world so we can still access distance() if it is declared under private access modifier.
{
	Node *closest_node = nullptr;
	Node node;
	for(auto node_index : node_indices)
	{
		node = parent_model->SNodes()[node_index]; // For each index in the loop, you can retrieve the Node object with
		if(!node.visited && this->distance(node) != 0) // For each retrieved Node in the loop, you should check that the node has not been visted (!node.visited) and that the distance to this is nonzero. In other words, you want the closest unvisted node that is not the current node.
			if(closest_node == nullptr || (this->distance(node) < this->distance(*closest_node)))
			closest_node = &parent_model->SNodes()[node_index];
	}
	return closest_node;
}

/*
	The goal of FindNeighbors is to populate the neighbors vector of the current Node object (the vector this->neighbors). 
*/

void RouteModel::Node::FindNeighbors()
{
	for( auto &road : parent_model->node_to_road[this->index]) 
	{
		RouteModel::Node* new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes); // you can get each road that the current node belongs to using the node_to_road hash table as follows: parent_model->node_to_road[this->index]
		if(new_neighbor)
			this->neighbors.push_back(new_neighbor);
	}
}

/*
	in order for the search to be performed with the map data, you will be finding a path between two nodes.
	This means that you need to find the nodes in the RouteModel that are closest to the starting and ending coordinates given by the user. 
*/

RouteModel::Node& RouteModel::FindClosestNode(float x, float y) // Here we are inside the RouteModel class world so we cannot access the function distance() if it is declared under private access modifier
{
	RouteModel::Node input;
	input.x = x;
	input.y = y;
	float dist;
	float min_dist = std::numeric_limits<float>::max();
	int closest_idx;
	for (auto &road : Roads())
	{
		if (road.type != Model::Road::Type::Footway)
		{
			for (auto node_index : Ways()[road.way].nodes)
			{
				dist = input.distance(SNodes()[node_index]);
				if (dist < min_dist)
				{
					closest_idx = node_index;
					min_dist = dist;
				}

			}

		}
	}
	return SNodes()[closest_idx];
}