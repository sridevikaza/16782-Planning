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
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <stack>

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

// struct to store robot state info
struct State {
    int x; // x pos
    int y; // y pos
    // int t; // time

    bool operator==(const State& other) const {
        return x == other.x && y == other.y;
        // return x == other.x && y == other.y && t == other.t;
    }
    bool operator!=(const State& other) const {
        return x != other.x && y != other.y;
        // return x != other.x && y != other.y && t != other.t;
    }
    bool operator<(const State& other) const {
        return x < other.x && y < other.y;
        // return x < other.x && y < other.y && t < other.t;
    }
};

// hash function for the State struct
struct StateHash {
    size_t operator()(const State& s) const {
        return hash<int>()(s.x) ^ hash<int>()(s.y);
        // return hash<int>()(s.x) ^ hash<int>()(s.y) ^ hash<int>()(s.t);
    }
};

// get the target goal state
State getTargetGoal(int* target_traj, int target_steps){
    State goal;
    goal.x = target_traj[target_steps-1];
    goal.y = target_traj[target_steps-1+target_steps];
    // goal.t = target_steps;
    return goal;
}

// euclidean distance (heuristic)
double getDistance(const State& s1, const State& s2){
    int dx = s1.x - s2.x;
    int dy = s1.y - s2.y;
    // int dt = s1.t - s2.t;
    return sqrt(dx * dx + dy * dy);
    // return sqrt(dx * dx + dy * dy + dt * dt);
}

// diagonal distance (heuristic)
double getDiagDist(const State& s1, const State& s2){
    int dx = abs(s1.x - s2.x);
    int dy = abs(s1.y - s2.y);
    // int dt = s1.t - s2.t;
    return dx + dy + (sqrt(2)-2) * min(dx,dy);
    // return sqrt(dx * dx + dy * dy + dt * dt);
    
}

// custom comparison function for min-heap
struct compareSmaller {
    bool operator()(const std::pair<double, State>& a, const std::pair<double, State>& b) const {
        return a.first > b.first;
    }
};

// actual A* search algorithm
unordered_map<State, State, StateHash> astar(
    std::priority_queue<std::pair<double, State>, std::vector<std::pair<double, State>>, compareSmaller> open_list,
    unordered_set<State, StateHash> closed_list,
    unordered_map<State, State, StateHash> parent_list,
    unordered_map<State, double, StateHash> g_values,
    State start,
    State goal,
    int* map,
    int x_size,
    int y_size,
    int collision_thresh
    )
{
    cout << "running A* search" << endl;

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // while s_goal not expanded and open list not empty
    while( !open_list.empty() && closed_list.count(goal)<=0 ){
        // cout << "open list size: " << open_list.size() << endl;
        // cout << "closed list size: " << closed_list.size() << endl;

        // remove s with the smallest f from open_list and add to closed
        State s = open_list.top().second;
        open_list.pop();
        closed_list.insert(s);

        // get all potential s' values
        State s_prime;
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            s_prime.x = s.x + dX[dir];
            s_prime.y = s.y + dY[dir];
            // s_prime.t = s.t + 1;

            // check size of s'
            if (s_prime.x >= 1 && s_prime.x <= x_size && s_prime.y >= 1 && s_prime.y <= y_size)
            {
                // get cost of moving to s'
                int cost = map[GETMAPINDEX(s_prime.x,s_prime.y,x_size,y_size)];

                // check if free to move to
                if ((cost >= 0) && (cost < collision_thresh))
                {
                    // check that s' not in closed_list 
                    if ( closed_list.count(s_prime) <= 0){

                        // if g(s') > g(s)
                        if ( g_values.count(s_prime) <=0 || g_values[s_prime] > g_values[s] + cost ){
                            g_values[s_prime] = g_values[s] + cost; // g(s') = g(s) + c(s,s')
                            parent_list[s_prime] = s;               // update parent list
                            open_list.push(make_pair(g_values[s_prime] + getDiagDist(s_prime, goal), s_prime)); // insert s into open list
                            // open_list.push(make_pair(g_values[s_prime], s_prime)); // insert s into open list
                        }
                    }
                }
            }
        }
    }
    cout << "returning parent list" << endl;
    return parent_list;
}

// backtrack to get the complete path
stack<State> backtrack(State goal, State start, unordered_map<State, State, StateHash> parent_list){

    cout << "backtracking to compute path" << endl;

    // make a path of States
    stack<State> path;
    State current = goal;

    // reconstruct the path
    while (current.x != start.x || current.y != start.y) {
        path.push(current);
        current = parent_list[current];
    }

    return path;
}


// initializing the search, call astar algorithm, and backtrack to get path
stack<State> getPath(State start, State goal, int* map,int x_size,int y_size,int collision_thresh){

    // initialize lists
    std::priority_queue<std::pair<double, State>, std::vector<std::pair<double, State>>, compareSmaller> open_list; // pair: f(s), s
    unordered_set<State, StateHash> closed_list;
    unordered_map<State, State, StateHash> parent_list; // maps s'->s
    unordered_map<State, double, StateHash> g_values;

    // initialize start conditions
    g_values[start] = 0;
    open_list.push(make_pair(g_values[start] + getDiagDist(start, goal), start));
    cout << "initialized starting values" << endl;

    // call A* algorithm
    parent_list = astar(open_list,closed_list,parent_list,g_values,start,goal,map,x_size,y_size,collision_thresh);
    cout << "finished running A*" << endl;
    cout << "parent list size: " << parent_list.size() << endl;
    return backtrack(goal, start, parent_list);

}

stack<State> path;
State start;
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
    if (first_run){
        // set start state
        start.x = robotposeX;
        start.y = robotposeY;
        // start.t = curr_time;
        // cout << "Start state - x: " << start.x << ", y: " << start.y << ", t: " << start.t << endl;
        cout << "Start state - x: " << start.x << ", y: " << start.y << endl;

        // set goal state
        State goal = getTargetGoal(target_traj, target_steps);
        // cout << "Goal state - x: " << goal.x << ", y: " << goal.y << ", t: " << goal.t << endl;
        cout << "Goal state - x: " << goal.x << ", y: " << goal.y << endl;

        // get computed path
        path = getPath(start, goal, map, x_size, y_size, collision_thresh);
        cout << "Computed a path of size: " << path.size() << endl;

        first_run = false;
    }

    // send the next state
    if (!path.empty()) {
        // Find the next state in the path
        action_ptr[0] = path.top().x;
        action_ptr[1] = path.top().y;
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