#include "PRM_planner.h"
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
double PRM_planner::config_distance(vector<double> config1, vector<double> config2)
{
    double dist = 0;
    for (int i= 0;i<numofDOFs;i++)
    {
        double dist_i = pow((config1[i] - config2[i]),2);
        dist+= dist_i;
    }
    return sqrt(dist);
}

vector <double*> PRM_planner::Build_PRM()
{
    for (int k=0;k<MaxSamples;k++)
    {
        // if (k%10000 == 0) cout<< k <<" "<< PRM.size() <<endl;
        vector <double> config_rand = GenerateRandomConfig();
        
        Node* qrand = new Node(config_rand, numofDOFs);
        
        double* config = &qrand->angles[0];
        if(IsValidArmConfiguration(config, numofDOFs, map, x_size, y_size))
        {
            if (!PRM.empty())
            {
                vector<Node*> Near = NearNeighbors(qrand,epsilon);
                for (int i = 0;i<Near.size();i++)
                {
                    add_vertex(Near[i],qrand);
                }
            }
            PRM.push_back(qrand);
        }
    }
    vector <double> start_config;
    for (int i=0;i<numofDOFs;i++)
    {
        start_config.push_back(armstart_anglesV_rad[i]);
    }
    Node* root = new Node(start_config,numofDOFs);
    Node* start_NearestNode = GetValidNearestNode(root);
    
    vector <double> end_config;
    for (int i=0;i<numofDOFs;i++)
    {
        end_config.push_back(armgoal_anglesV_rad[i]);
    }
    Node* goal = new Node(end_config,numofDOFs);
    Node* goal_NearestNode = GetValidNearestNode(goal);
    unordered_map<Node*, Node*> pred;
    pair<bool,vector <double*>> path = BFS(start_NearestNode,goal_NearestNode, pred, root, goal);
    cout<<"Path found:"<<path.first<<endl;
    cout<<"No. of Nodes: "<<PRM.size()<<endl;
    return path.second;

}
vector<Node*> PRM_planner::NearNeighbors(Node* qrand,double r)
{
    vector<Node*> Near;
    for (int i = 0;i<PRM.size();i++)
    {
        Node* temp = PRM[i];
        double distance_i = config_distance(temp->angles,qrand->angles);
        if (distance_i <= r)
        {
            Near.push_back(temp);
        }
    }
    return Near;
}
vector <double> PRM_planner::GenerateRandomConfig()
{
    vector <double> config_rand;
    double bias = goalSampleGenerator(gen);
    for (int i=0; i<numofDOFs; i++)
    {
        // double angle_i = 2*PI*((double)rand() / (RAND_MAX));
        double angle_i = randomAngleGenerator(gen);
        config_rand.push_back(angle_i);
    }
    return config_rand;
}

void PRM_planner::add_vertex(Node* qnear,Node* qnew)
{
    if (IsObstacleFree(qnear,qnew))
    {
        graph[qnew].push_back(qnear);
        graph[qnear].push_back(qnew);
    }
   
}

void PRM_planner::print_angles(Node*q)
{
    for (int k=0;k<numofDOFs;k++)
    {
        cout<<q->angles[k]<<" ";
    
    }
    cout<<endl;
}
bool PRM_planner::IsObstacleFree(Node*q1, Node*q2)
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
Node* PRM_planner::GetValidNearestNode(Node* q)
{
    Node* qnear;
    bool found = false;
    double distance = FLT_MAX;
    double eps = epsilon;
    vector<Node*> Near = NearNeighbors(q,eps);
    while(!found)
    {
        for (int i=0;i<Near.size();i++)
        {
            if (IsObstacleFree(q,Near[i]))
            {
                double distance_i = config_distance(q->angles,Near[i]->angles);
                if (distance_i < distance)
                {
                    qnear = Near[i];
                    found = true;
                }
            }
        }
        eps *= 2;
        Near = NearNeighbors(q,eps);
    }
    return qnear;
}
pair<bool,vector <double*>> PRM_planner::BFS(Node* src, Node* dest, unordered_map<Node*, Node*> pred, Node* start, Node* goal)
{
    list<Node*> queue;
    unordered_map<Node*, int> visited;
    vector <double*> path;
    pair<bool,vector <double*>> path_return;
    path_return.first = false;
    path_return.second = path;
    double* config_init = &goal->angles[0];
    path.push_back(config_init);
    visited[src] = 1;
    queue.push_back(src);
    int count = 0;
    while (!queue.empty()) {
        count++;
        Node* u = queue.front();
        queue.pop_front();
        for (int i = 0; i < graph[u].size(); i++) {
            if (visited.count(graph[u][i]) == 0) {
                visited[graph[u][i]] = 1;
                // dist[graph[u][i]] = dist[u] + 1;
                pred[graph[u][i]] = u;
                queue.push_back(graph[u][i]);
                if (config_distance(graph[u][i]->angles,dest->angles)<0.001)
                {
                    Node* crawl = dest;
                    double* config = &crawl->angles[0];
                    path.push_back(config);
                    while (pred.count(crawl)>0) {
                        double* config = &pred[crawl]->angles[0];
                        path.push_back(config);
                        crawl = pred[crawl];
                    }
                    config = &start->angles[0];
                    path.push_back(config);
                    **planlength = path.size();
                    path_return.second = path;
                    path_return.first = true;
                    return path_return;
                }
                    
            }
            
        }
    }
    return path_return;
}