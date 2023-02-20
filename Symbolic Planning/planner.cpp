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
#include <bits/stdc++.h>
#include <queue>
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
using namespace std::chrono;
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
    void set_truth(bool truth_value)
    {
        this->truth = truth_value;
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
    void set_args( list<string> new_args)
    {
        this->args = new_args;
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
     unordered_set<Action, ActionHasher, ActionComparator> get_all_actions() const
    {
        return this->actions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_condition()
    {
        return this->initial_conditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_condition()
    {
        return this->goal_conditions;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_effects;
    bool action_exists = false;
public:
    GroundedAction()
    {}
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }

    }
    // ***************************************************************************************************************************************
    GroundedAction(string name, list<string> arg_values, Action action)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
        this->action_exists = true;
        unordered_set<Condition, ConditionHasher, ConditionComparator> precond = action.get_preconditions();
        unordered_map<string,string> ac_to_gac;
        auto itr = arg_values.begin();
            for (const auto& arg : action.get_args()) {
                ac_to_gac[arg] = *itr;
                ++itr;
            }
        
        for (Condition con : precond)
        {
            list<string> args_temp;
            for (string st: con.get_args())
            {
                if (ac_to_gac.count(st)>0) args_temp.push_back(ac_to_gac[st]);
                else args_temp.push_back(st);
            }
            GroundedCondition precond(con.get_predicate(), args_temp,con.get_truth());
            this->grounded_preconditions.insert(precond);
        } 
        
        unordered_set<Condition, ConditionHasher, ConditionComparator> effect = action.get_effects();
        for (Condition eff : effect)
        {
            list<string> args_temp;
            for (string st: eff.get_args())
            {
                // cout<<st<<" ";
                if (ac_to_gac.count(st)>0) args_temp.push_back(ac_to_gac[st]);
                else args_temp.push_back(st);
            }
            // cout<<endl;
            GroundedCondition effect(eff.get_predicate(), args_temp,eff.get_truth());
            this->grounded_effects.insert(effect);
        } 
        
    }
 // ***************************************************************************************************************************************
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
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->grounded_preconditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->grounded_effects;
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

// ***************************************************************************************************************************************
class Node
{
    public:
    string name;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> literal;
    Node* parent = NULL;
    double h = 0;
    double g = FLT_MAX;
    double f = FLT_MAX;
    GroundedAction ga;
    Node()
    {}
    Node(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> literal,string name="NoName")
    {
        this->literal = literal;
        this->name = name;
    }
    void Print()
    {
        if (name!= "NoName") cout<<name<<" ";
        cout<<"Node Literals: "<<endl;
        for (GroundedCondition con: literal)
        {
            cout<< con.toString()<<" ";
        }
        
        cout<<endl<<endl;
    }

    bool operator==(Node other)
    {
        // ### My own method of comparing unordered_sets ###

        // if (this->literal.size()!=other.literal.size()) return false;
        // return is_permutation(this->literal.begin(), this->literal.end(), other.literal.begin());
        // for (auto itr = this->literal.begin(); itr != this->literal.end(); ++itr)
        // {
        //     int count = 0;
        //     for (auto itr1 = other.literal.begin(); itr1 != other.literal.end(); ++itr1)
        //     {
        //         count+= int(*itr1==*itr);
        //     }
        //     if (count==0) return false;
        // }
        // return true;
        return this->literal == other.literal;
    }
    
};
double Heuristic (Node* node, Node* goal)
{
    // double h = 0;
    double h2 = 0;
    for(GroundedCondition goal_con:goal->literal)
    {
        // int count = 0;
        // for(GroundedCondition con:node->literal)
        // {
        //     if (goal_con==con) count++;
        // }
        // if (count==0) h++;

        if (node->literal.find(goal_con) == node->literal.end()) h2++;
    }
    // cout << (h==h2) <<endl;
    // return h;

    // ################################### Uncomment the following line for using typ1 Heuristic function ###################################
    // return h2;
    return 0;
    
}
auto cmp= [](const Node* a, const Node* b)
    {
        if (a->f == b->f) return a->h < b->h;
        return a->f > b->f;
    };


bool IsValidAction(Node* parent, GroundedAction single_GroundAction)
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> precondtions = single_GroundAction.get_preconditions();
    // for (auto itr = precondtions.begin(); itr != precondtions.end(); ++itr)
    // {
    //     int count = 0;
    //     for (auto itr1 = parent->literal.begin(); itr1 != parent->literal.end(); ++itr1)
    //     {
    //         count+= int(*itr1==*itr);
    //     }
    //     if (count==0) return false;
    // }
    for (GroundedCondition con: precondtions)
    {
        if (parent->literal.find(con) == parent->literal.end()) return false;
    }
    return true;

}

bool IsGoal(Node* node,Node* goal)
{
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> node_literal = node->literal;
    // unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_literal = goal->literal;
    // int count = 0;
    // for (auto itr = goal_literal.begin(); itr != goal_literal.end(); ++itr)
    // {
        
    //     for (auto itr1 = node_literal.begin(); itr1 != node_literal.end(); ++itr1)
    //     {
    //         count+= int(*itr1==*itr);
    //     }
        
    // }
    
    // if (count==goal_literal.size())
    // {
    //     return true;
    // } 
    // return false;

    for (GroundedCondition con: goal->literal)
    {
        if (node->literal.find(con) == node->literal.end()) return false;
    }
    return true;


}
pair<bool, Node*> IsVisited(Node* &node, vector<Node*> &all_nodes)
{
    pair<bool, Node*> visited(false,node);
    for(int i=0;i<all_nodes.size();i++)
    {
        if (node->literal.size()!=all_nodes[i]->literal.size()) continue;
        if (IsGoal(node,all_nodes[i]))
        {
            visited.second = all_nodes[i];
            visited.first = true;
            return visited;
        }
    }
    return visited;
}
unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> Return_New_literals(Node* parent, GroundedAction single_GroundAction)
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects = single_GroundAction.get_effects();
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> literals = parent->literal;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_literals = literals;
    
    for (GroundedCondition eff: effects)
    {
        if (!eff.get_truth())
            {
                // cout<<eff.toString()<<endl;
                GroundedCondition temp = eff;
                temp.set_truth(true);
                if (new_literals.find(temp)!=new_literals.end()) new_literals.erase(temp);
                // for (GroundedCondition con: new_literals)
                // {
                //     if (temp == con)
                //     {
                //         new_literals.erase(con);
                //         break;
                //     }
                // }
            }
        else{
            new_literals.insert(eff);
        }
        
    }
    return new_literals;
}
void Get_all_combinations(vector<string> const &arr, int i, int k,set<vector<string>> &subarrays, vector<string> &out)
{
    if (arr.size() == 0) {
        return;
    }
    if (k == 0) {
        subarrays.insert(out);
        return;
    }
    if (i < 0) {
        return;
    }
    
    out.push_back(arr[i]);
    Get_all_combinations(arr, i - 1, k - 1, subarrays, out);
 
    out.pop_back(); 
 
    Get_all_combinations(arr, i - 1, k, subarrays, out);
}
list<list<string>> get_all_symbol_permutations(unordered_set<string> symbols, int size = 0)
{
    vector<string> all_symbols;
    for (auto itr = symbols.begin(); itr != symbols.end(); ++itr)
    {
        all_symbols.push_back(*itr);
    }
    set<vector<string>> subarrays;
    vector<string> out;
    Get_all_combinations(all_symbols, all_symbols.size() - 1, size, subarrays, out);
    list<list<string>> all_permutations;
    for (vector<string> vec: subarrays) {
        sort(vec.begin(),vec.end());
        do {
        list<string> single_permutation;
        for (int i=0;i<size;i++)
        {
            single_permutation.push_back(vec[i]);
        }
        all_permutations.push_back(single_permutation);
    } while (next_permutation(vec.begin(),vec.end()));
        
    }
  return all_permutations;
}

bool Process_valid_actions(Action &action, unordered_set<string> &symbols, Node* &parent, Node* &Goal, priority_queue <Node*,vector<Node*>,decltype(cmp)> &q, vector<Node*> &all_nodes)
{   
    list<list<string>> all_symbol_permutations = get_all_symbol_permutations(symbols,action.get_args().size());
    
    // ##########Print All Permutations for all actions#############################
    // cout<<action.toString()<<endl;
    // for (list<string> s1:all_symbol_permutations )
    // {
    //     for (string s2: s1)
    //     {
    //         cout<<s2<<" ";
    //     }
    //     cout<<endl;
    // }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> new_literals;
    GroundedAction single_GroundAction;
    for (auto itr = all_symbol_permutations.begin(); itr != all_symbol_permutations.end(); ++itr)
    {
        
        single_GroundAction= GroundedAction(action.get_name(),*itr,action);
        bool valid = IsValidAction(parent, single_GroundAction);
        if (valid){
        
            new_literals = Return_New_literals(parent, single_GroundAction);
            Node* new_node = new Node(new_literals);
            pair<bool, Node*> visited = IsVisited(new_node,all_nodes);
            
            if (visited.first)
            {
                if (visited.second->g > parent->g + 1)
                {
                    visited.second->ga = single_GroundAction;
                    visited.second->h = Heuristic(new_node,Goal);
                    visited.second->g = parent->g + 1;
                    visited.second->f = new_node->h + new_node->g;
                    visited.second->parent = parent;

                    //// ################################### Uncomment the following if loop for Early Stopping results ###################################
                    // if (IsGoal(visited.second,Goal)) 
                    // {
                    //     Goal->parent = visited.second->parent;
                    //     Goal = visited.second;
                    //     cout<<"Goal g_cost: "<<Goal->g<<endl;
                    //     return true;
                    
                    // }


                    // cout<<"New g_cost of Node was less"<<endl;
                }
                
            }
            else
            {
                new_literals = Return_New_literals(parent, single_GroundAction);
                new_node->literal = new_literals;
                new_node->ga = single_GroundAction;
                new_node->h = Heuristic(new_node,Goal);
                new_node->g = parent->g + 1;
                new_node->f = new_node->h + new_node->g;
                q.push(new_node);
                (all_nodes).push_back(new_node);
                new_node->parent = parent;
                //// ################################### Uncomment the following if loop for Early Stopping results ###################################
                // if (IsGoal(new_node,Goal)) 
                // {
                //     Goal->parent = new_node->parent;
                //     Goal = new_node;
                //     cout<<"Goal g_cost: "<<Goal->g<<endl;
                //     return true;
                
                // }
            }
        }
        
    }
    return false;

}
void Expand_Nodes(Env* env, Node* &Start, Node* &Goal, vector<Node*> &all_nodes, vector <Node*> &closed)
{
    unordered_set<string> st =  env->get_symbols();
    Node* current = Start;
    unordered_set<Action, ActionHasher, ActionComparator> actions_all = env->get_all_actions();
    priority_queue <Node*,vector<Node*>,decltype(cmp)> q(cmp);
    q.push(current);
    int count = 0;
    bool isgoal = false;
    while(!q.empty() && count<2000)
    {
        if (count%100 == 0) 
        {
            cout<<"while loop no.: "<<count<<" ||No. of nodes:  "<<all_nodes.size()<<" ||Priority queue size: "<<q.size()<<endl;   
        }
        current = q.top();
        if (IsGoal(current,Goal)) 
        {
            Goal = current;
            return;
        }
        closed.push_back(current);
        q.pop();
        count++;
        for (Action ac : actions_all)
        {
            isgoal = Process_valid_actions(ac, st, current, Goal, q,all_nodes);
            //// ################################### Uncomment the following if loop for Early Stopping results ###################################    
            // if (isgoal)
            // {
            //     cout<<"Goal Found!"<<endl;
            //     return;
            // }
        }
    }
    cout<<"While loop count: "<<count<<endl;
    cout<<"Size of priority queue: "<<q.size()<<endl;
    return;
}

list<GroundedAction> planner(Env* env)
{
    
    auto start_time = high_resolution_clock::now();
    Node* start = new Node(env->get_initial_condition(),"start");
    Node* goal = new Node(env->get_goal_condition(),"goal");
    start->h = Heuristic(start,goal);
    start->g = 0;
    start->f = start->g + start->h;
    start->Print();
    vector<Node*> all_nodes;
    (all_nodes).push_back(start);
    vector <Node*> closed;
    Expand_Nodes(env, start,goal,all_nodes, closed);
    
    list<GroundedAction> actions;
    // Node* current = all_nodes[all_nodes.size() -1];
    Node* current = goal;
    int count = 0;

    // current = all_nodes[all_nodes.size() -1];
    // count = 0;
    
    while(current->parent!=NULL)
    {
        actions.push_back(current->ga);
        current = current->parent;
        count++;
    }
    actions.reverse();
    
    
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));
    auto stop_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop_time - start_time);
    cout << "Time required for planning in sec: " <<duration.count()/1000.0 << endl;
    cout<<"Number of nodes: "<<(all_nodes).size()<<endl;
    cout<< "Number of Nodes expanded: "<<closed.size()<<endl;
    cout<<"No. of Actions: "<<count<<endl;
    goal->Print();
    cout<<endl;
    return actions;
}

// *********************************************************************************************************************************************
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