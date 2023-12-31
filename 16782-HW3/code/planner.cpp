#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <vector>
#include <string>
#include <numeric>
#include <iterator>
#include <chrono>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }
    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }


    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

// struct to store state info
struct State {
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
    string action_name;
    list<string> action_arg_values;

    bool operator==(const State& other) const {
        return conditions == other.conditions && conditions == other.conditions;
    }
    bool operator!=(const State& other) const {
        return conditions != other.conditions || conditions != other.conditions;
    }
};

// hash function for the State struct
struct StateHasher {
    size_t operator()(const State& state) const {
        size_t combined_hash = 0;
        for (const auto& condition : state.conditions) {
            // rotate the current combined hash to avoid collision patterns
            combined_hash = (combined_hash << 1) | (combined_hash >> (sizeof(size_t) * 8 - 1));
            
            // use the GroundedConditionHasher to hash each condition
            GroundedConditionHasher cond_hasher;
            size_t cond_hash = cond_hasher(condition);

            // combine the current hash with the hash of the condition
            combined_hash ^= cond_hash;
        }
        return combined_hash;
    }
};

// custom comparison function for min-heap
struct compareSmaller {
    bool operator()(const pair<double, State>& a, const pair<double, State>& b) const {
        return a.first > b.first;
    }
};

// # of goal conditions that are not satisfied
int getHeuristic(const State& current, const State& goal){
    int not_satisfied = 0;
    auto currentConditions = current.conditions;
    auto goalConditions = goal.conditions;

    for (const auto& condition : goalConditions){
        // check if condition is in currentConditions
        if (currentConditions.find(condition) == currentConditions.end()){
            not_satisfied++;
        }
    }

    return not_satisfied;
}

// get permutations of symbols
void generatePermutations(vector<string>& symbols, int num_args, vector<string> current, vector<vector<string>>& result, vector<bool>& used) {
    if (num_args == 0) {
        result.push_back(current);
        return;
    }

    for (int i = 0; i < symbols.size(); ++i) {
        if (!used[i]) {
            vector<string> next_current = current;
            next_current.push_back(symbols[i]);
            used[i] = true;
            generatePermutations(symbols, num_args - 1, next_current, result, used);
            used[i] = false;  // backtrack to explore other possibilities
        }
    }
}

vector<vector<string>> getAllPermutations(vector<string>& symbols, int num_args) {
    vector<vector<string>> result;
    vector<string> current;
    vector<bool> used(symbols.size(), false);

    generatePermutations(symbols, num_args, current, result, used);

    return result;
}

// check if preconditions are satisfied
bool checkPreconditions(const State& current, const Action& action, vector<string> symbol_perm){
    auto preconditions = action.get_preconditions();
    auto action_args = action.get_args();

    // for each cond in preconditions -> make grounded condition
    for (const auto& cond : preconditions){
        string predicate = cond.get_predicate();
        bool truth = cond.get_truth();
        list<string> cond_args = cond.get_args();
        list<string> new_args;
        // for each arg in cond args
        for (const auto& cond_arg : cond_args){
            auto it = find(action_args.begin(), action_args.end(), cond_arg);
            // cond arg in action args
            if (it != action_args.end()) {
                int index = distance(action_args.begin(), it);
                new_args.push_back(symbol_perm[index]);
            }
            // cond arg not in action args
            else {
                new_args.push_back(cond_arg);  
            }
        }
        // make grounded condition
        GroundedCondition gr_cond = GroundedCondition(predicate, new_args, truth);

        // check if gr_cond is in the current state
        if (current.conditions.find(gr_cond) == current.conditions.end()) {
            return false;
        }

    }

    return true;
}

// create the next state
State getNextState(const State& current, const Action& action, vector<string> symbol_perm, unordered_set<string> symbols_set){
    // get action args
    auto action_args = action.get_args();

    // make new state struct
    State s_prime;
    list<string> args_list(symbol_perm.begin(), symbol_perm.end());
    s_prime.action_arg_values = args_list;
    s_prime.action_name = action.get_name();
    auto new_conds = current.conditions;
    
    // for each effect_cond in effect_conds -> make grounded cond
    for (const auto& effect_cond : action.get_effects()){
        auto actual_truth = effect_cond.get_truth();
        auto pred = effect_cond.get_predicate();
        list<string> new_args;
        for (const auto& effect_arg : effect_cond.get_args()){

            // if the arg is a symbol already (not a variable)
            if(symbols_set.find(effect_arg) != symbols_set.end()){
                new_args.push_back(effect_arg);
            } else{
                // get the correspinding index from the action_args
                auto it = find(action_args.begin(), action_args.end(), effect_arg);
                if (it != action_args.end()) {
                    // arg found, calculate its index
                    int index = distance(action_args.begin(), it);
                    new_args.push_back(symbol_perm[index]);
                }
            }
        }

        // make a grounded condition with the new_args
        GroundedCondition gr_cond = GroundedCondition(pred, new_args);

        // check if the grounded condition is in the state conditions and accordingly delete or add
        if(new_conds.find(gr_cond) != new_conds.end()){
            if (actual_truth != gr_cond.get_truth()){
                new_conds.erase(gr_cond);
            }
        } else{
            new_conds.insert(gr_cond);
        }
    }

    s_prime.conditions = new_conds;
    return s_prime;
}

// check if the goal conditions are a subset of the current conditions
bool goalConditionsSatisfied(const State& s, const State& goal){
    auto state_conds = s.conditions;
    auto goal_conds = goal.conditions;

    for (const auto& goal_cond : goal_conds){
        if (state_conds.count(goal_cond)<=0){
            return false;
        }
    }
    return true;
}

pair<State, unordered_map<State, State, StateHasher>> astar(
    State goal,
    State start,
    unordered_set<Action, ActionHasher, ActionComparator> actions,
    unordered_set<string> symbols_set
    )
{
    bool goal_found = false;
    State final_goal;

    // initialize lists
    priority_queue<pair<double, State>, vector<pair<double, State>>, compareSmaller> open_list; // pair: f(s), s
    unordered_set<State, StateHasher> closed_list;
    unordered_map<State, State, StateHasher> parent_list; // maps s'->s
    unordered_map<State, double, StateHasher> g_values;

    // initialize start conditions
    g_values[start] = 0;
    open_list.push(make_pair(g_values[start] + getHeuristic(start, goal), start));
    State s = start;

    // check that open list isn't empty
    while( !open_list.empty() && !goal_found ) {

        // remove s with the smallest f from open_list and add to closed
        s = open_list.top().second;
        open_list.pop();
        closed_list.insert(s);
        
        // for each action
        for (const auto& action : actions) {

            // get all the permutations of symbols
            auto args = action.get_args();
            vector<string> symbols(symbols_set.begin(), symbols_set.end());
            vector<vector<string>> symbol_permutations = getAllPermutations(symbols, args.size());

            // for each permutation
            for (const auto& symbol_perm : symbol_permutations){

                // check if preconditions are satisfied
                if (checkPreconditions(s, action, symbol_perm)){

                    // determine the new conditions (make s')
                    State s_prime = getNextState(s, action, symbol_perm, symbols_set);

                    // check that s' not in closed_list
                    if ( closed_list.count(s_prime) <= 0){

                        // if g(s') > g(s)
                        if ( g_values.count(s_prime) <=0 || g_values[s_prime] > g_values[s] + 1 ){
                            g_values[s_prime] = g_values[s] + 1;    // g(s') = g(s) + c(s,s')
                            parent_list[s_prime] = s;               // update parent list
                            open_list.push(make_pair(g_values[s_prime] + getHeuristic(s_prime, goal), s_prime)); // insert s into open list
                            
                            // check if goal is found 
                            if (goalConditionsSatisfied(s_prime, goal)){
                                goal_found = true;
                                final_goal = s_prime;
                            }
                        }
                    }
                }
            }
        }
    }

    if(!goal_found){
        cout << "NO GOAL FOUND" << endl;
    }
    cout << "Number of States Expanded: " << closed_list.size() << endl;
    return make_pair(final_goal, parent_list);
}

// backtrack parent list to get the complete path
list<GroundedAction> backtrack(const State& start, const State& goal, unordered_map<State, State, StateHasher> parent_list){


    // make a list of actions
    list<GroundedAction> actions;

    State current = goal;

    // reconstruct the path
    while (current.conditions != start.conditions) {
        GroundedAction ga = GroundedAction(current.action_name, current.action_arg_values);
        actions.push_back(ga);
        current = parent_list[current];
    }

    actions.reverse();
    return actions;
}

list<GroundedAction> planner(Env* env)
{
    // start clock
    auto start_time = chrono::high_resolution_clock::now();

    State start;
    State goal;
    start.conditions = env->get_initial_conditions();
    goal.conditions = env->get_goal_conditions();
    unordered_set<Action, ActionHasher, ActionComparator> actions = env->get_actions();
    unordered_set<string> symbols = env->get_symbols();

    auto result = astar(goal, start, actions, symbols);
    auto plan = backtrack(start, result.first, result.second);

    // print metrics
    auto plan_end = chrono::high_resolution_clock::now();
    auto planning_time = chrono::duration_cast<chrono::milliseconds>(plan_end - start_time);
    cout << "Planning Time: " << planning_time.count() << " milliseconds" << endl;

    return plan;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}