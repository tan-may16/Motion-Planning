#include <bits/stdc++.h>
class state {
    public:
    int robotposx;
    int robotposy;
    double g_cost;
    double h_cost;
    double f_cost;
    bool state_expanded;
    state * parent;

    state(int robotposx, int robotposy)
    {
        this->robotposx = robotposx;
        this->robotposy = robotposy;
        parent = NULL;
        this->g_cost = INT_MAX;
        this->h_cost = 0;
        this->f_cost = INT_MAX;
        state_expanded = false;
    }
};
