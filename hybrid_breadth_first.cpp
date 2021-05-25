#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "hybrid_breadth_first.h"

HBF::HBF() = default;

HBF::~HBF() = default;

int HBF::ThetaToStackNumber(double theta)
{
  /*
   * Takes an angle (in radians) and returns which "stack" in the 3D configuration space
   * this angle corresponds to. Angles near 0 go in the lower stacks
   * while angles near 2 * pi go in the higher stacks.
  */
  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) % NUM_THETA_CELLS;
  return stack_number;
}


int HBF::Idx(double float_num) {
  /*
   * Returns the index into the grid for continuous position. So if x is 3.621,
   * then this would return 3 to indicate that 3.621 corresponds to array index 3.
  */
  return int(floor(float_num));
}


std::vector<HBF::MazeS> HBF::Expand(HBF::MazeS state) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;

  int g2 = g+1;

  std::vector<HBF::MazeS> next_states;
  for(double delta_i = -35; delta_i < 40; delta_i += 5)
  {

    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 > 0)
    {
        theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta2);
    double y2 = y + SPEED * sin(theta2);

    HBF::MazeS state2;
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);

  }
  return next_states;
}

std::vector<HBF::MazeS> HBF::ReconstructPath(std::vector<std::vector<std::vector<HBF::MazeS>>> came_from, std::vector<double> start, HBF::MazeS final)
{
    std::vector<MazeS> path = {final};
    
    int stack = ThetaToStackNumber(final.theta);

    MazeS current = came_from[stack][Idx(final.x)][Idx(final.y)];
    
    stack = ThetaToStackNumber(current.theta);
    
    double x = current.x;
    double y = current.y;
    while( x != start[0] && y != start[1] )
    {
        path.push_back(current);
        current = came_from[stack][Idx(x)][Idx(y)];
        x = current.x;
        y = current.y;
        stack = ThetaToStackNumber(current.theta);
    }
    
    return path;

}

HBF::MazePath HBF::Search(std::vector<std::vector<int>> grid, std::vector<double> start, std::vector<int> goal)
{
    /*
     * Working Implementation of breadth first search.
     * Does NOT use a heuristic and as a result this is pretty inefficient.
     * Try modifying this algorithm into hybrid A* by adding heuristics appropriately.
    */
    std::vector<std::vector<std::vector<MazeS>>> closed(NUM_THETA_CELLS, std::vector<std::vector<MazeS>>(grid[0].size(), std::vector<MazeS>(grid.size())));
    std::vector<std::vector<std::vector<int>>> closed_value(NUM_THETA_CELLS, std::vector<std::vector<int>>(grid[0].size(), std::vector<int>(grid.size())));
    std::vector<std::vector<std::vector<MazeS>>> came_from(NUM_THETA_CELLS, std::vector<std::vector<MazeS>>(grid[0].size(), std::vector<MazeS>(grid.size())));
    double theta = start[2];
    int stack = ThetaToStackNumber(theta);
    int g = 0;

    MazeS state;
    state.g = g;
    state.x = start[0];
    state.y = start[1];

    closed[stack][Idx(state.x)][Idx(state.y)] = state;
    closed_value[stack][Idx(state.x)][Idx(state.y)] = 1;
    came_from[stack][Idx(state.x)][Idx(state.y)] = state;

    int total_closed = 1;
    std::vector<MazeS> opened = {state};
    while(!opened.empty())
    {
        sort(opened.begin(), opened.end());

        //grab first element
        MazeS next = opened[0];

        //pop first element
        opened.erase(opened.begin());

        int x = next.x;
        int y = next.y;

        if(Idx(x) == goal[0] && Idx(y) == goal[1])
        {
            std::cout << "found path to goal in " << total_closed << " expansions" <<std::endl;
            MazePath path;
            path.closed = closed;
            path.came_from = came_from;
            path.final = next;
            return path;

        }

        std::vector<MazeS> next_state = Expand(next);

        for(auto &i : next_state)
        {
            int g2 = i.g;
            double x2 = i.x;
            double y2 = i.y;
            double theta2 = i.theta;

            if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size()))
            {
                // invalid cell
                continue;
            }

            int stack2 = ThetaToStackNumber(theta2);
            if(closed_value[stack2][Idx(x2)][Idx(y2)] == 0 && grid[Idx(x2)][Idx(y2)] == 0)
            {
                int h2 = HEURISTIC[Idx(x2)][Idx(y2)];
                int f2 = g2 + h2;

                MazeS state2;
                state2.f = f2;
                state2.g = g2;
                state2.x = x2;
                state2.y = y2;
                state2.theta = theta2;

                opened.push_back(state2);

                closed[stack2][Idx(x2)][Idx(y2)] = i;
                closed_value[stack2][Idx(x2)][Idx(y2)] = 1;
                came_from[stack2][Idx(x2)][Idx(y2)] = next;
                total_closed += 1;
            }
        }
    }

    std::cout << "no valid path." << std::endl;
    HBF::MazePath path;
    path.closed = closed;
    path.came_from = came_from;
    path.final = state;
    return path;
}

