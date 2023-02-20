#include "RRTConnect_planner.h"
#include <math.h>  
#include <random>
#include <cstdlib>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <bits/stdc++.h>
#include <limits>

// using namespace std;
#define PI 3.141592654
double RRTConnect_planner::config_distance(vector<double> config1, vector<double> config2)
{
    double dist = 0;
    for (int i= 0;i<numofDOFs;i++)
    {
        double dist_i = pow((config1[i] - config2[i]),2);
        dist+= dist_i;
    }
    return sqrt(dist);
}

Node* RRTConnect_planner::NearestNeighbor(vector <Node*> &RRT,Node* qrand)
{
    Node* qnear = new Node();
    double distance = FLT_MAX;
    int size;
    
    // cout<<"NN start"<<endl;
    for (int i = 0;i<RRT.size();i++)
    {
        Node* temp = RRT[i];
        
        double distance_i = config_distance(temp->angles,qrand->angles);
        if (distance_i <= distance)
        {
            distance = distance_i;
            qnear = temp;
        }
    }
    // cout<<"Distance: "<<distance<<endl;
    return qnear;
}
void RRTConnect_planner::Build_RRT()
{
    // for (int k=0;k<MaxSamples;k++)
    while((RRT.size() + RRT_goal.size())<MaxSamples)
    {
        vector <double> config_rand = GenerateRandomConfig();
        vector<Node*>* temp1 = &RRT;
        vector<Node*>* temp2 = &RRT_goal;
        Node* qrand = new Node(config_rand, numofDOFs);
        pair<Node*,int> extend_state;
        pair<Node*,int> connect_state;
        extend_state.second = -1;
        connect_state.second = -1;
        extend_state = extend(*temp1,qrand,true);
        if(extend_state.second!=2)
        {
            connect_state = connect(*temp2, extend_state.first);
        }
        if (connect_state.second == 0)
        {
            break;
        }
        else{
            // cout<<"Swapping"<<endl;
            swap(*temp1,*temp2);
        }
    }
    cout<<"No. of Nodes: "<<RRT.size() + RRT_goal.size()<<endl;

}
vector <double> RRTConnect_planner::GenerateRandomConfig()
{
    vector <double> config_rand;
    double bias = goalSampleGenerator(gen);
    if (bias<0.1)
    {
        for (int i=0;i<numofDOFs;i++)
        {
            config_rand.push_back(armgoal_anglesV_rad[i]);
        }
        return config_rand;
    }
    for (int i=0; i<numofDOFs; i++)
    {
        double angle_i = randomAngleGenerator(gen);
        config_rand.push_back(angle_i);
    }
    return config_rand;
}
bool RRTConnect_planner::IsNewConfig(Node* qnear,Node* qrand)
{
    for (int i=0;i<numofDOFs;i++)
    {
        if (qnear->angles[i]!= qrand->angles[i])
        {
            return true;
        }
    }
    return false;
}
pair<Node*,int> RRTConnect_planner::extend(vector <Node*> &RRT,Node* qrand, bool add_point)
{
    Node* qnear = NearestNeighbor(RRT,qrand);
    pair<Node*,int>  Node_state = add_vertex(RRT, qnear,qrand, add_point);
    return Node_state;

}
pair<Node*,int> RRTConnect_planner::add_vertex(vector <Node*> &RRT, Node* qnear,Node* qnew, bool add_point)
{
    vector <double> start_angles = qnear->angles;
    vector <double> end_angles = qnew->angles;
    double config_dist = config_distance(start_angles,end_angles);
    vector <double> eps_vec;
    if (config_dist > epsilon)
    {
        for (int m=0;m<numofDOFs;m++)
        {
            eps_vec.push_back((epsilon/config_dist)*(end_angles[m]-start_angles[m]));
        }
    }
    else
    {
        for (int m=0;m<numofDOFs;m++)
        {
            eps_vec.push_back((end_angles[m]-start_angles[m]));
        }
    }
    int i,j;
	int countNumvalid = 0;
    Node* qadd = new Node();
    for (i = 1; i <= f_interpolate; i++)
    {
        Node* qtemp = new Node(); 
        for(j = 0; j < numofDOFs; j++)
        {
            qtemp->angles.push_back(start_angles[j] + ((double)(i)/(f_interpolate))*eps_vec[j]);
        
        }
        double* config = &qtemp->angles[0];
        if(IsValidArmConfiguration(config, numofDOFs, map, x_size, y_size)) 
        {
			++countNumvalid;
            qadd = qtemp;
        }
        else{
            break;
        }
    }
    pair<Node*,int> new_point;
    new_point.first = qadd;
    if (countNumvalid > 0)
    {
        RRT.push_back(qadd);
        qadd->parent = qnear;
        if (AreEqual(qadd,qnew)) new_point.second=0;//reached
        else new_point.second=1; //advanced
    }
    else new_point.second=2; //Trapped
    return new_point;
}
bool RRTConnect_planner::IsGoal(Node* q)
{
    vector <double> goal_config;
    for (int i=0;i<numofDOFs;i++)
    {
        goal_config.push_back(armgoal_anglesV_rad[i]);
    }
    if (config_distance(q->angles, goal_config) > 0.001) //0.1 deg.
    {
        return false;
    }
    return true;
}
vector<double*> RRTConnect_planner::getpath()
{
    vector <double*> path_temp_start;
    vector <double*> path_temp_goal;
    Node *temp = RRT[RRT.size()-1]; 
    int i=0;
    double* config=NULL;
    while(temp->parent!=NULL)
    {
        config = &temp->angles[0];
        path_temp_start.push_back(config);
        temp = temp->parent;
        i++;

    }
    config = &temp->angles[0];
    path_temp_start.push_back(config);
    temp = RRT_goal[RRT_goal.size()-1]; 
    i=0;
    config=NULL;
    while(temp->parent!=NULL)
    {
        config = &temp->angles[0];
        path_temp_goal.push_back(config);
        temp = temp->parent;
        i++;

    }
    config = &temp->angles[0];
    path_temp_goal.push_back(config);
    
    reverse(path_temp_start.begin(), path_temp_start.end());
    path_temp_start.insert( path_temp_start.end(), path_temp_goal.begin(), path_temp_goal.end() );
    
    **planlength = path_temp_start.size();
    return path_temp_start;

}
void RRTConnect_planner::print_angles(Node*q)
{
    for (int k=0;k<numofDOFs;k++)
    {
        cout<<q->angles[k]<<" ";
    
    }
    cout<<endl;
}
pair<Node*,int>  RRTConnect_planner::connect(vector <Node*> &RRT,Node* q)
{
    pair<Node*,int>  s = extend(RRT,q,true);
    int start_idx = RRT.size() - 1;
    
    while(s.second == 1)
    {
        s = extend(RRT,q,true);
    }
    int end_idx = RRT.size() - 1;
    return s;
}
bool RRTConnect_planner::AreEqual(Node*q1,Node*q2)
{
    for (int i = 0; i < numofDOFs; ++i) {
        if (abs(q1->angles[i] - q2->angles[i]) > 1e-3) {
            // cout << endl;
            return false;
        }
    }
    return true;
}
