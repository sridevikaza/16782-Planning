/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <cmath>
#include <queue>
#include <iostream>
#include <algorithm>
#include <stack>
#include <vector>
#include <memory>
#include <chrono>

// #include "astar.h"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std;


// get the target goal state
pair<int, int> getTargetGoal(int* target_traj, int target_steps){
    int x = target_traj[target_steps-1];
    int y = target_traj[target_steps-1+target_steps];
    return make_pair(x, y);
}

// euclidean distance (heuristic)
double getDistance(int x1, int y1, int x2, int y2){
    double dx = x1-x2;
    double dy = y1-y2;
    return sqrt(dx*dx + dy*dy);
}


// custom comparison function for min-heap
struct compareSmaller {
    bool operator()(const pair<double, int>& a, const pair<double, int>& b) const {
        return a.first > b.first;
    }
};

// actual A* search algorithm
tuple<vector<int>, vector<int>> astar(
    int xstart,
    int ystart,
    int xgoal,
    int ygoal,
    int* map,
    int x_size,
    int y_size,
    int collision_thresh
    )
{
    cout << "running A* search" << endl;

    // initialize data structures
    unique_ptr<vector<int>> xmoves = make_unique<vector<int>>();
    unique_ptr<vector<int>> ymoves = make_unique<vector<int>>();
    unique_ptr<vector<int>> g_vals = make_unique<vector<int>>(x_size*y_size, INFINITY);     // g values 
    unique_ptr<vector<bool>> closed_list = make_unique<vector<bool>>(x_size*y_size);        // closed list
    priority_queue<pair<double, int>, vector<pair<double, int>>, compareSmaller> open_list; // open list 

    // initialize start conditions
    int x = xstart;
    int y = ystart;
    (*g_vals)[x_size*(y-1)+x] =  0;               // g start = 0
    double h = getDistance(x, y, xgoal, ygoal);   // euclidean dist heuristic
    open_list.push(make_pair(h, x_size*(y-1)+x)); // add start to open list

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // while open list not empty and s_goal not expanded
    while( !open_list.empty() && !(*closed_list)[xgoal*ygoal+xgoal] ){

        // cout << "open list size: " << open_list.size() << endl;
        // cout << "closed list size: " << closed_list.size() << endl;

        // add the index of the smallest f value to closed list
        (*closed_list)[open_list.top().second] = true;

        // get x and y from index
        if(open_list.top().second % x_size == 0){
            x = x_size;
            y = open_list.top().second / x_size;
        }
        else{
            x = open_list.top().second % x_size;
            y = ceil(open_list.top().second / x_size);
        }
        
        // remove s from open list
        open_list.pop();

        // get all potential s' values
        int xprime;
        int yprime;
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            xprime = x + dX[dir];
            yprime = y + dY[dir];

            // check size of s'
            if (xprime >= 1 && xprime <= x_size && yprime >= 1 && yprime <= y_size)
            {
                // get cost of moving to s'
                int cost = map[GETMAPINDEX(xprime,yprime,x_size,y_size)];

                // check if free to move to
                if ((cost >= 0) && (cost < collision_thresh))
                {
                    // check that s' not in closed_list 
                    if ( !(*closed_list)[x_size*(yprime-1)+xprime] ){

                        // if g(s') > g(s)
                        if ( (*g_vals)[x_size*(yprime-1)+xprime] > (*g_vals)[x_size*(y-1)+x] + cost ){
                            (*g_vals)[x_size*(yprime-1)+xprime] = (*g_vals)[x_size*(y-1)+x] + cost; // g(s') = g(s) + c(s,s')

                            // save moves
                            xmoves->push_back(dX[dir]);
                            ymoves->push_back(dY[dir]);

                            // update open list
                            double f = (*g_vals)[x_size*(yprime-1)+xprime] + getDistance(x, y, xprime, yprime);
                            open_list.push(make_pair(f, x_size*(yprime-1)+xprime)); // insert s into open list

                        }
                    }
                }
            }
        }
    }

    // return moves
    return make_tuple(*xmoves, *ymoves);
}

// backtrack to get the complete path
stack<pair<int,int>> backtrack(int xstart, int ystart, int xgoal, int ygoal, const vector<int>& xmoves, const vector<int>& ymoves){

    cout << "backtracking to compute path" << endl;

    // make a path of States
    stack<pair<int,int>> path;

    int xcurr = xgoal;
    int ycurr = ygoal;
    int index = xmoves.size();

    // reconstruct the path
    while (xcurr != xstart || ycurr != ystart) {
        path.push({xcurr, ycurr});
        xcurr -= xmoves[index];
        ycurr -= ymoves[index];
        index--;
    }

    return path;
}


// initializing the search, call astar algorithm, and backtrack to get path
stack<pair<int,int>> getPath(int xstart, int ystart, int xgoal, int ygoal, int* map,int x_size,int y_size,int collision_thresh){
    
    // call A* algorithm
    // shared_ptr<vector<int>> xmoves = make_shared<vector<int>>();
    // shared_ptr<vector<int>> ymoves = make_shared<vector<int>>();
    tuple<vector<int>, vector<int>> moves = astar(xstart, ystart, xgoal, ygoal, map, x_size, y_size, collision_thresh);
    cout << "completed A*" << endl;

    vector<int> xmoves = get<0>(moves);
    vector<int> ymoves = get<1>(moves);
    cout << "number of steps: " << xmoves.size() << endl;

    // backtrack moves
    if ( xmoves.size() > 0 ){
        return backtrack(xstart, ystart, xgoal, ygoal, xmoves, ymoves);
    }
    else{
        cout << "EMPTY PATH" << endl;
        return stack<pair<int,int>>();
    }

}

stack<pair<int,int>> path;
bool first_run = true;

// main planner function that gets called
void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    cout << "Running Planner V2" << endl;

    if (first_run){

        cout << "Start state - x: " << robotposeX << ", y: " << robotposeY << endl;

        // set goal state
        pair<int, int> goal = getTargetGoal(target_traj, target_steps);
        cout << "Goal state - x: " << goal.first << ", y: " << goal.second << endl;

        // start clock
        auto start = chrono::high_resolution_clock::now();

        // get computed path
        path = getPath(robotposeX, robotposeY, goal.first, goal.second, map, x_size, y_size, collision_thresh);

        // get wall clock computation time
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);

        cout << "Computation time: " << duration.count() << " milliseconds" << endl;
        cout << "Computed path size: " << path.size() << endl;

        first_run = false;
    }

    // if there's a computed path
    if (!path.empty()) {
        // send the next state
        action_ptr[0] = path.top().first;
        action_ptr[1] = path.top().second;
        path.pop();
        // cout << "x pos: " << action_ptr[0] << endl;
        // cout << "y pos: " << action_ptr[1] << endl;
    }
    else{
        // don't move if there's no path
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    return;
}

