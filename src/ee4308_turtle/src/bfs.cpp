#include "bfs.hpp"
BFS::Node::Node() 
    : idx(-1, -1), parent(-1, -1) 
    {}
BFS::Open::Open() 
    : idx(-1, -1) 
    {}
BFS::Open::Open(Index idx) 
    : idx(idx) 
    {}
BFS::BFS(Grid & grid) // assumes the size of the grid is always the same
    : start(-1, -1), goal(-1, -1), grid(grid), nodes(grid.size.i * grid.size.j), open_list()
    {
        // write the nodes' indices
        int k = 0;
        for (int i = 0; i < grid.size.i; ++i)
        {
            for (int j = 0; j < grid.size.j; ++j)
            {
                nodes[k].idx.i = i;
                nodes[k].idx.j = j;
                ++k;
            }
        }
    }
    

void BFS::add_to_open(Node * node)
{   // sort node into the open list

    open_list.emplace_back(node->idx);
}
BFS::Node * BFS::poll_from_open()
{   
    Index & idx = open_list.front().idx; //ref is faster than copy
    int k = grid.get_key(idx);
    Node * node = &(nodes[k]);

    open_list.pop_front();

    return node;
}

Index BFS::get(Index idx)
{
    std::vector<Index> path_idx; // clear previous path

    // set start node g cost as zero
    int k = grid.get_key(idx);
    Node * node = &(nodes[k]);

    // add start node to openlist
    add_to_open(node);

    // main loop
    while (!open_list.empty())
    {
        // (1) poll node from open
        node = poll_from_open();

        // (3) return path if node is the goal
        if (grid.get_cell(node->idx))
        {   // reached the goal, return the path
            ROS_INFO("[BFS] Found free cell");
            open_list.clear();
            return node->idx;
        }

        // (4) check neighbors and add them if cheaper
        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   // for each neighbor in the 8 directions

            // get their index
            Index & idx_nb_relative = NB_LUT[dir];
            Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            // check if in map and accessible
            int nb_k = grid.get_key(idx_nb);
            if ((grid.grid_log_odds[nb_k] > grid.log_odds_thresh) && grid.out_of_map(idx_nb))
            {   // if not, move to next nb
                // in map, not inflated, and log odds occupied
                continue;
            }

            Node & nb_node = nodes[nb_k]; // use reference so changing nb_node changes nodes[k]
            nb_node.parent = node->idx;
            add_to_open(&nb_node); // & a reference means getting the pointer (address) to the reference's object.

            // toggle is_cardinal
            is_cardinal = !is_cardinal;
        }
    }

    // clear open list
    open_list.clear();
    return idx; // is empty if open list is empty
}

