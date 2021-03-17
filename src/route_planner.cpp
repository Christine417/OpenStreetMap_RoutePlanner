#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x,start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// Implement the CalculateHValue method.
// use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*end_node);
    return h_value;

}

// Use the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto node : current_node->neighbors){
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        this->open_list.emplace_back(node);
        node->visited = true; 
    }

}


// Use the NextNode method to sort the open list and return the next node.
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare (const RouteModel::Node*node1, const RouteModel::Node*node2){
        float distance_sum1 = node1->g_value + node2->h_value;
        float distance_sum2 = node2->g_value + node2->h_value;
        return distance_sum1 > distance_sum2;
    }

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *next_node;

    

    sort(this->open_list.begin(), this->open_list.end(), Compare);
    next_node = this->open_list.back();
    open_list.pop_back();

    return next_node;

}


// Use the ConstructFinalPath method to return the final path found from your A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::vector<RouteModel::Node> path_found_0;
    while(current_node->parent != start_node){
        path_found_0.push_back(*current_node);
        distance = distance + current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    path_found_0.push_back(*current_node);
    path_found_0.push_back(*start_node);
    distance = distance + current_node->distance(*(current_node->parent));

    for(int i = path_found_0.size()-1; i>=0; i--){
        path_found.push_back(path_found_0[i]);
    }
    
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm 
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    RouteModel::Node *next_node = nullptr;

    current_node = start_node;

    current_node->visited = true;
    this->open_list.emplace_back(current_node);
       
    while(open_list.size() >0){
        current_node = this->NextNode();
        
        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(end_node);
            }
        else
            {
                this->AddNeighbors(current_node);/* code */
            }
            
        
    }
    
}
