#ifndef HYBRID_BREADTH_FIRST_H_
#define HYBRID_BREADTH_FIRST_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>

class HBF
{
public:
    int NUM_THETA_CELLS = 90;
    double SPEED = 1.45;
    double LENGTH = 0.5;

    std::vector<std::vector<int>>HEURISTIC =
    {
            {30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,},
            {29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,},
            {28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,},
            {27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,},
            {26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,},
            {25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,},
            {24,23,22,21,20,19,18,17,16,15,14,13,12,11,10, 9,},
            {23,22,21,20,19,18,17,16,15,14,13,12,11,10, 9, 8,},
            {22,21,20,19,18,17,16,15,14,13,12,11,10, 9, 8, 7,},
            {21,20,19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6,},
            {20,19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5,},
            {19,18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4,},
            {18,17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3,},
            {17,16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2,},
            {16,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1,},
            {15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,},
    };

    struct maze_s
    {
        int f;
        int g;  // iteration
        double x;
        double y;
        double theta;
        
        bool operator<(const maze_s& a) const
        {
            return f < a.f;
        }
    };

    struct maze_path
    {
        std::vector<std::vector<std::vector<maze_s>>> closed;
        std::vector<std::vector<std::vector<maze_s>>> came_from;
        maze_s final;

    };

    HBF();

    virtual ~HBF();
    
    int ThetaToStackNumber(double theta);

    int Idx(double float_num);

    std::vector<maze_s> Expand(maze_s state);

    maze_path Search(std::vector<std::vector<int> > grid, std::vector<double> start, std::vector<int> goal);

    std::vector<maze_s> ReconstructPath(std::vector<std::vector<std::vector<maze_s>>> came_from, std::vector<double> start, HBF::maze_s final);
};

#endif
