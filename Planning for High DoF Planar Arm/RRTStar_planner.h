#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H
#include <vector>
#include "Node.h"
#include <random>
#include "helper.h"
#endif
using namespace std;
#define PI 3.141592654
// class Node
// {
//     public:
//     vector <double> angles;
//     Node* parent;
//     // vector <Node*> Children;
//     Node()
//     {}
//     Node(vector<double> angles,int numDOF)
//     {
//         for (int i = 0;i<numDOF;i++)
//         {
//             this->angles.push_back(angles[i]);
//         }
//         this->parent = NULL;
//     }
// };

 class RRTStar_planner
 {
    public:

    vector <Node*> RRT;
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
    RRTStar_planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs,double**** plan, int** planlength,int MaxSamples, double epsilon, double f_interpolate, bool seed)
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
        vector <double> start_config;
        for (int i=0;i<numofDOFs;i++)
        {
            start_config.push_back(armstart_anglesV_rad[i]);
        }
        Node* root = new Node(start_config,numofDOFs);
        RRT.push_back(root);
        goalSampleGenerator = std::uniform_real_distribution<double>(0.0, 1.0); 
        randomAngleGenerator = std::uniform_real_distribution<double>(0.0, 2 * PI); 
        unsigned seeding = std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seeding);
        
    }
    
    void add_vertex(Node* qnear,Node* qnew);
    // void add_edge(Node* qnear,Node* qnew);
    double config_distance(vector<double> config1, vector<double> config2);
    void extend(Node* qrand);
    vector <double> GenerateRandomConfig();
    Node* NearestNeighbor(Node* qrand);
    void Build_RRT();
    bool IsNewConfig(Node* qnear,Node* qrand);
    bool IsGoal(Node* q);
    vector<double*> getpath();
    void print_angles(Node* q);
    std::mt19937 gen; 
    std::uniform_real_distribution<double> goalSampleGenerator; 
    std::uniform_real_distribution<double> randomAngleGenerator; 
    vector<Node*> NearNeighbors(Node* qrand,double r);
    bool IsObstacleFree(Node* q1, Node*q2);
 };