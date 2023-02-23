#include "ros/ros.h"
#include "grid.hpp"
#include "common.hpp"
#include "planner.hpp"
#include <vector>
#include <deque>

#ifndef BFS_HPP
#define BFS_HPP

class BFS
{
    public:
        struct Node
        {
            bool visited;
            Index idx, parent;
            Node();
        };
        struct Open
        {
            Index idx;
            Open();
            Open(Index idx);
        };
        Index start, goal;
        Grid & grid; // REFERENCE <-- you cannot put the Planner class into containers (vectors , arrays etc.) 
        
        BFS(Grid & grid);
        std::vector<Index> get(Index idx);
        std::vector<Position> get(Position pos_start);

    private:
        std::vector<Node> nodes;
        std::deque<Open> open_list;
        Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

        void add_to_open(Node * node);
        Node * poll_from_open();    
};
#endif