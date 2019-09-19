#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    auto &GetDistance() { return distance; }
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node *start_node, *end_node; // These will point to the nodes in the model which are closest to our starting and ending points.
    float distance; // his variable will hold the total distance for the route that A* search finds from start_node to end_node
    std::vector<RouteModel::Node *> open_list;
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    float CalculateHValue( const RouteModel::Node *);
    bool compareHvalue(const RouteModel::Node *, const RouteModel::Node *);
    RouteModel::Node * NextNode();
    void AddNeighbors(RouteModel::Node *);


};	
