#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>
#include <vector>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        std::vector<Node *> neighbors;
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {} // The constructor intialization is not clear. 
        float distance(Node other) const 
        {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }
        void FindNeighbors();


      private:
        // Add private Node variables and methods here.      
        int index;
        RouteModel * parent_model = nullptr; //  pointer to a RouteModel object named parent_model. This variable is important, as it allows each node to access data stored in the parent model that the node belongs to.
        Node* FindNeighbor(std::vector<int> node_indices);

    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);
    auto& SNodes()   { return m_Nodes; }    // Getter function returns a reference to m_Nodes
    auto& GetNodeToRoadMap() {return node_to_road;} 
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    Node& FindClosestNode(float x, float y);

  private:
    // Add private RouteModel variables and methods here.
  std::vector<Node> m_Nodes;
  std::unordered_map <int, std::vector<const Model::Road*>> node_to_road;
  void CreateNodeToRoadHashmap();
  
};
