#include "RRT_planner.h"
#include <math.h>  
#include <random>
#include <cstdlib>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <bits/stdc++.h>
#include <limits>

using namespace std;
#define PI 3.141592654
double RRT_planner::config_distance(vector<double> config1, vector<double> config2)
{
    double dist = 0;
    for (int i= 0;i<numofDOFs;i++)
    {
        double dist_i = pow((config1[i] - config2[i]),2);
        dist+= dist_i;
    }
    return sqrt(dist);
}

Node* RRT_planner::NearestNeighbor(Node* qrand)
{
    Node* qnear = new Node();
    double distance = FLT_MAX;
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
    return qnear;
}
void RRT_planner::Build_RRT()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    gen.seed(seed);
    // for (int k=0;k<MaxSamples;k++)
    while(RRT.size()<MaxSamples)
    {
        // if (k%10000 == 0) cout<<k<<" "<<RRT.size()<<endl;
        // gen.seed((int)0);
        // if (RRT.size()%1000 == 0)
        // {
        //     cout<<RRT.size()<<endl;
        // }
        vector <double> config_rand = GenerateRandomConfig();
        Node* qrand = new Node(config_rand, numofDOFs);
        extend(qrand);
        if (goal_index!= 0)
        {
            // cout<<"Goal Index: "<<goal_index<<endl;
            break;
        }
    }
    cout<<"No. of Nodes: "<<RRT.size()<<endl;

}
vector <double> RRT_planner::GenerateRandomConfig()
{
    // gen.seed((int)0);
    vector <double> config_rand;
    double bias = goalSampleGenerator(gen);
    // double bias = (double)rand()/RAND_MAX;
    // // cout<<bias<<endl;
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
        // double angle_i = 2*PI*(double)rand()/RAND_MAX;
        config_rand.push_back(angle_i);
    }
    return config_rand;
}
bool RRT_planner::IsNewConfig(Node* qnear,Node* qrand)
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
void RRT_planner::extend(Node* qrand)
{
    Node* qnear = NearestNeighbor(qrand);
    add_vertex(qnear,qrand);

}
void RRT_planner::add_vertex(Node* qnear,Node* qnew)
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
    if (countNumvalid > 0 && goal_index==0)
    {
        RRT.push_back(qadd);
        qadd->parent = qnear;
        if (IsGoal(qadd))
        {
            goal_index = RRT.size();
        }
    }
}
bool RRT_planner::IsGoal(Node* q)
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
vector<double*> RRT_planner::getpath()
{
    vector <double*> path_temp;
    if (goal_index == 0)
    {
        cout<<"Path Not found| Returning Empty Vector"<<endl;
        return path_temp;
    }
    
    Node *temp = RRT[goal_index-1]; 
    int i=0;
    double* config=NULL;
    while(temp->parent!=NULL)
    {
        config = &temp->angles[0];
        path_temp.push_back(config);
        temp = temp->parent;
        i++;

    }
    // cout<<"Backtracking Steps: "<<path_temp.size()<<endl;
    // double* config = &temp->angles[0];
    config = &temp->angles[0];
    path_temp.push_back(config);
    reverse(path_temp.begin(), path_temp.end());
    **planlength = path_temp.size();
    return path_temp;

}
void RRT_planner::print_angles(Node*q)
{
    for (int k=0;k<numofDOFs;k++)
    {
        cout<<q->angles[k]<<" ";
    
    }
    cout<<endl;
}