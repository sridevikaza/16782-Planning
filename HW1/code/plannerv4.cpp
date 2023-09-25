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
#include <chrono>


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
        return x != other.x || y != other.y;
        // return x != other.x || y != other.y || t !=other.t;
    }
    bool operator<(const State& other) const {
        return x < other.x && y < other.y;
        // return x < other.x && y < other.y && t < other.t;
    }
};

State chosen_goal;
stack<State> path;
State start;
bool first_run = true;
chrono::_V2::system_clock::time_point start_time;

// hash function for the State struct
struct StateHash {
    size_t operator()(const State& s) const {
        return hash<int>()(s.x) ^ hash<int>()(s.y);
        // return hash<int>()(s.x) ^ hash<int>()(s.y) ^ hash<int>()(s.t);
    }
};

vector<State> getMultiGoal(int* target_traj, int target_steps){
    vector<State> multiGoal;
    for (int i=0; i<=target_steps; ++i) {
        State goal;
        goal.x = target_traj[i-1];
        goal.y = target_traj[target_steps-1+i];
        // goal.t = i+1;  // time starts from 1 to target_steps
        multiGoal.push_back(goal);
    }
    return multiGoal;
}

// weighted distance
double getWeightedDistance(const State& s1, const State& s2, double weight){
    int dx = s1.x - s2.x;
    int dy = s1.y - s2.y;
    // int dt = s1.t - s2.t;
    // return weight * sqrt(dx*dx + dy*dy + dt*dt);
    return weight * sqrt(dx*dx + dy*dy);
}

// average distance heuristic
double getHeuristic(const State& s, const vector<State>& multiGoal, double weight) {
    double total_distance = 0;
    int i = 0;
    for(const auto& goal : multiGoal) {
        i++;
        total_distance += getWeightedDistance(s, goal, weight*i/multiGoal.size());
    }
    return total_distance / multiGoal.size();
}

// get the last target state
State getFinalGoal(int* target_traj, int target_steps){
    State goal;
    goal.x = target_traj[target_steps-1];
    goal.y = target_traj[target_steps-1+target_steps];
    // goal.t = target_steps;
    return goal;
}

// custom comparison function for min-heap
struct compareSmaller {
    bool operator()(const pair<double, State>& a, const pair<double, State>& b) const {
        return a.first > b.first;
    }
};

// backtrack to get the complete path
int getNumMoves(const State& s, const State& start, unordered_map<State, State, StateHash> parent_list){

    int count = 0;
    State current = s;
    // reconstruct the path
    // while (current.x != start.x || current.y != start.y || current.t != start.t) {
    while (current.x != start.x || current.y != start.y) {
        // path.push(current);
        current = parent_list[current];
        count += 1;
    }

    return count;
}

bool checkGoal(int wall_clock_t, State s, State last_goal, vector<State> multiGoal, int* target_traj, int target_steps, State start, unordered_map<State, State, StateHash> parent_list){
    
    int buffer_time = 0;

    for (int i=wall_clock_t; i<=target_steps; ++i) {

        int x = target_traj[i-1];
        int y = target_traj[target_steps-1+i];

        if (s.x == last_goal.x && s.y == last_goal.y ){
            return true;
        }

        if (s.x == x && s.y == y){

            int num_moves = getNumMoves(s, start, parent_list);
            if ( i > num_moves+wall_clock_t ){
                cout << "found a goal" << endl;
                cout << "number of moves for robot to reach goal: " << num_moves << endl;
                cout << "seconds passed: " << wall_clock_t << endl;
                cout << "number of moves for target to be at goal: " << i << endl;
                return true;
            }
        }
    }
    return false;

}

//  A* search algorithm
unordered_map<State, State, StateHash> astar(
    int size,
    State goal,
    vector<State> multiGoal,
    State start,
    int* map,
    int x_size,
    int y_size,
    int collision_thresh,
    int* target_traj,
    int target_steps
    )
{
    cout << "running A* search" << endl;

    // initialize lists
    priority_queue<pair<double, State>, vector<pair<double, State>>, compareSmaller> open_list; // pair: f(s), s
    unordered_set<State, StateHash> closed_list;
    unordered_map<State, State, StateHash> parent_list; // maps s'->s
    unordered_map<State, double, StateHash> g_values;

    // initialize start conditions
    g_values[start] = 0;
    open_list.push(make_pair(g_values[start] + getHeuristic(start, multiGoal, 15), start)); 
    // open_list.push(make_pair(g_values[start] + getWeightedDistance(start, goal, 10), start));
    cout << "initialized starting values" << endl;

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    State s = start;

    // while s_goal not expanded and open list not empty
    // while( !open_list.empty() && closed_list.count(goal)<=0 ){

    auto current_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::seconds>(current_time - start_time);
    int wall_clock_t = ceil(duration.count());
    // cout << wall_clock_t << endl;

    while( !open_list.empty() ){

        if (wall_clock_t>=target_steps){
            cout << "time exceeded -- no goal found" << endl;
            parent_list = unordered_map<State, State, StateHash>();
            break;
        }

        // get wall clock time
        current_time = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::seconds>(current_time - start_time);
        wall_clock_t = ceil(duration.count());

        if (checkGoal(wall_clock_t, s, goal, multiGoal, target_traj, target_steps, start, parent_list)){
            chosen_goal = s;
            break;
        }

        // remove s with the smallest f from open_list and add to closed
        s = open_list.top().second;
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
                            open_list.push(make_pair(g_values[s_prime] + getHeuristic(s_prime, multiGoal, 15), s_prime)); // insert s into open list
                            // open_list.push(make_pair(g_values[s_prime] + getWeightedDistance(s_prime, goal, 10), s_prime)); // insert s into open list
                        }
                    }
                }
            }
        }
    }
    cout << "returning parent list" << endl; // todo: can probs take out
    if ( parent_list.size() >= size-1 ){
        cout << "no goal found" << endl;
        return unordered_map<State, State, StateHash>();
    }
    return parent_list;
}

// backtrack to get the complete path
stack<State> backtrack(const State& goal, const State& start, unordered_map<State, State, StateHash> parent_list){

    cout << "backtracking to compute path" << endl;

    // make a path of States
    stack<State> path;

    State current = goal;

    // reconstruct the path
    // while (current.x != start.x || current.y != start.y || current.t != start.t) {
    while (current.x != start.x || current.y != start.y) {
        path.push(current);
        current = parent_list[current];
    }

    return path;
}

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

    // cout << " RUNNING PLANNER V3" << endl;

    if (first_run){

        // start clock
        start_time = chrono::high_resolution_clock::now();

        // set start state
        start.x = robotposeX;
        start.y = robotposeY;
        // start.t = START_TIME;
        // cout << "Start state - x: " << start.x << ", y: " << start.y << ", t: " << start.t << endl;
        cout << "Start state - x: " << start.x << ", y: " << start.y << endl;

        // set goal state
        vector<State> multiGoal = getMultiGoal(target_traj, target_steps);
        State goal = getFinalGoal(target_traj, target_steps);

        int size = x_size * y_size;

        // get computed path
        unordered_map<State, State, StateHash> parent_list = astar(size, goal,multiGoal,start,map,x_size,y_size,collision_thresh,target_traj,target_steps);
        cout << "finished running A*" << endl;
        cout << "parent list size: " << parent_list.size() << endl;
        path = backtrack(chosen_goal, start, parent_list);

        // get wall clock computation time
        auto stop_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::seconds>(stop_time - start_time);

        cout << "Wall clock time: " << duration.count() << " seconds" << endl;
        cout << "Computed path size: " << path.size() << endl;

        first_run = false;
    }

    // send the next state
    if (!path.empty()) {
        // get the next state in the stack
        action_ptr[0] = path.top().x;
        action_ptr[1] = path.top().y;
        path.pop();
    }
    else {
        // don't move if there's no path
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    return;
}

// todo: can probs take out the end codition checks since I added finding the last goal check
