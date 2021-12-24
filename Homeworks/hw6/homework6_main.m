%% Homework 6
% Multipoint trajectories

%% Cubic splines with assigned initial and final velocities: computation based on the accelerations
clear;
clc;

% qk=[0 20 0 20];
% tk=[0 4 8 12];

qk=[5 20 -25 6 -5 10];
tk=[0 2 8 12 15 20];

Ts=0.01;

dqi=0;
dqf=0;

%last boolean parameter tell if the plot must be showed

[q,dq,ddq,dddq,ddqk]=multipoint_comp_based_on_acc(qk,tk,Ts,dqi,dqf,true);

%% Smoothing Cubic Splines, assumption qi,qf=0
clear;
clc;

% qk=[0 20 0 20];
% tk=[0 4 8 12];

qk=[5 20 -25 6 -5 10];
tk=[0 1 4 6 7.5 10];

Ts=0.01;

mu=0.7;

% attention the w_vector is of the form [1/w1,...,1/wn]
% if you want to impose to pass through a point i, impose w_vector(i)=0
w_vec=[1 1 1 1 1 1];


% last boolean parameter tell if the plot must be showed
% the q parameters represent the interpolated splines, s the smoothed ones

[q,dq,ddq,dddq,ddqk,s,ds,dds,ddds,ddsk]=smoothing_traj(qk,tk,Ts,mu,w_vec,true);



