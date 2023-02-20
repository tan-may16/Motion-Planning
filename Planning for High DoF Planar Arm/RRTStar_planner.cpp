#include "RRTStar_planner.h"
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
double RRTStar_planner::config_distance(vector<double> config1, vector<double> config2)
{
    double dist = 0;
    for (int i= 0;i<numofDOFs;i++)
    {
        double dist_i = pow((config1[i] - config2[i]),2);
        dist+= dist_i;
    }
    return sqrt(dist);
}

Node* RRTStar_planner::NearestNeighbor(Node* qrand)
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
vector<Node*> RRTStar_planner::NearNeighbors(Node* qrand,double r)
{
    vector<Node*> Near;
    for (int i = 0;i<RRT.size();i++)
    {
        Node* temp = RRT[i];
        double distance_i = config_distance(temp->angles,qrand->angles);
        if (distance_i <= r)
        {
            // distance = distance_i;
            Near.push_back(temp);
            // qnear = temp;
        }
    }
    return Near;
}
void RRTStar_planner::Build_RRT()
{
    // for (int k=0;k<MaxSamples;k++)
    while(RRT.size()<MaxSamples)
    {
        // if (k%10000 == 0) cout<<k<<" "<<RRT.size()<<endl;
        vector <double> config_rand = GenerateRandomConfig();
        Node* qrand = new Node(config_rand, numofDOFs);
        extend(qrand);
        if (goal_index!= 0)
        {
            // cout<<"Goal Index: "<<goal_index<<endl;
            break;
        }
    }
    cout<<"No. of Nodes "<<RRT.size()<<endl;

}
vector <double> RRTStar_planner::GenerateRandomConfig()
{
    vector <double> config_rand;
    // double bias = ((double)rand() / (RAND_MAX));
    double bias = goalSampleGenerator(gen);
    if (bias<0.1)
    {
        // cout<<"Bias is less than 0.2"<<endl;
        for (int i=0;i<numofDOFs;i++)
        {
            // config_rand[i] = armgoal_anglesV_rad[i];
            config_rand.push_back(armgoal_anglesV_rad[i]);
        }
        return config_rand;
    }
    for (int i=0; i<numofDOFs; i++)
    {
        // double angle_i = 2*PI*((double)rand() / (RAND_MAX));
        double angle_i = randomAngleGenerator(gen);
        // cout<<angle_i<<" ";
        config_rand.push_back(angle_i);
    }
    // cout<<endl;
    // cout<<"Node added"<<endl;
    return config_rand;
}
bool RRTStar_planner::IsNewConfig(Node* qnear,Node* qrand)
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
void RRTStar_planner::extend(Node* qrand)
{
    Node* qnear = NearestNeighbor(qrand);
    add_vertex(qnear,qrand);

}
void RRTStar_planner::add_vertex(Node* qnear,Node* qnew)
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
        qadd->cost = qnear->cost + config_distance(qnear->angles,qadd->angles);
        if (IsGoal(qadd))
        {
            goal_index = RRT.size();
        }
    }
    if (countNumvalid>0)
    {
        vector<Node*> Near = NearNeighbors(qadd,epsilon);
        for (int i =0;i<Near.size();i++)
        {
            if (IsObstacleFree(Near[i],qadd))
            {
                double new_cost = Near[i]->cost + config_distance(Near[i]->angles,qadd->angles);
                if (new_cost < qadd->cost)
                {
                    qadd->parent = Near[i];
                }

            }
        }
        for (int i =0;i<Near.size();i++)
        {
            if (IsObstacleFree(Near[i],qadd))
            {
                double new_cost = qadd->cost + config_distance(Near[i]->angles,qadd->angles);
                if (new_cost < Near[i]->cost)
                {
                    Near[i]->parent = qadd;
                }

            }
        }
    }  
}
bool RRTStar_planner::IsGoal(Node* q)
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
vector<double*> RRTStar_planner::getpath()
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
        // double* config = &temp->angles[0];
        config = &temp->angles[0];
        // (*plan)[i] = config;
        path_temp.push_back(config);
        temp = temp->parent;
        // cout<<i<<endl;
        i++;

    }
    config = &temp->angles[0];
    path_temp.push_back(config);
    reverse(path_temp.begin(), path_temp.end());
    // cout<<"Path reversed"<<endl;
    **planlength = path_temp.size();
    return path_temp;

}
void RRTStar_planner::print_angles(Node*q)
{
    for (int k=0;k<numofDOFs;k++)
    {
        cout<<q->angles[k]<<" ";
    
    }
    cout<<endl;
}
bool RRTStar_planner::IsObstacleFree(Node*q1, Node*q2)
{
    vector <double> start_angles = q1->angles;
    vector <double> end_angles = q2->angles;
    for (int i = 1; i <= f_interpolate; i++)
    {
        Node* qtemp = new Node(); 
        for(int j = 0; j < numofDOFs; j++)
        {
            qtemp->angles.push_back(start_angles[j] + ((double)(i)/(f_interpolate))*(end_angles[j]-start_angles[j]));
        }
        
        double* config = &qtemp->angles[0];
        if(!IsValidArmConfiguration(config, numofDOFs, map, x_size, y_size)) 
        {
			return false;
        }
    }
    return true;
}