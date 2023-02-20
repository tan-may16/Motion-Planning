#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include <vector>
#include "Node.h"
#include "helper.h"
#endif
#include <unordered_map>
using namespace std;
#define PI 3.141592654

 class PRM_planner
 {
    public:

    vector <Node*> PRM;
    unordered_map <Node*, vector<Node*>> graph;
    double* map;
    int x_size;
    int y_size;
    double* armstart_anglesV_rad;
    double* armgoal_anglesV_rad;
    int numofDOFs;
    double**** plan;
    int** planlength;
    int MaxSamples;
    double epsilon;
    double f_interpolate;
    int goal_index;
    bool random_seed;
    PRM_planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs,double**** plan, int** planlength,int MaxSamples, double epsilon, double f_interpolate, bool seed)
    {
        this->map = map;
        this->x_size = x_size;
        this->y_size = y_size;
        this->armstart_anglesV_rad = armstart_anglesV_rad;
        this->armgoal_anglesV_rad = armgoal_anglesV_rad;
        this->numofDOFs = numofDOFs;
        this->plan = plan;
        this->planlength = planlength;
        this->MaxSamples = MaxSamples;
        this->epsilon = epsilon;
        this->f_interpolate = f_interpolate;
        goal_index = 0;
        this->random_seed = seed;
        randomAngleGenerator = std::uniform_real_distribution<double>(0.0, 2 * PI); 
        unsigned seeding = std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seeding);
        
    }
    
    void add_vertex(Node* qnear,Node* qnew);
    double config_distance(vector<double> config1, vector<double> config2);
    vector <double> GenerateRandomConfig();
    vector <double*> Build_PRM();
    void print_angles(Node* q);
    std::mt19937 gen; 
    std::uniform_real_distribution<double> goalSampleGenerator; 
    std::uniform_real_distribution<double> randomAngleGenerator; 
    vector<Node*> NearNeighbors(Node* qrand,double r);
    Node* GetValidNearestNode(Node* q);
    bool IsObstacleFree(Node*q1, Node*q2);
    pair<bool,vector <double*>> BFS(Node* src, Node* dest, unordered_map<Node*, Node*> pred, Node* start, Node* goal);
 };