%% Homework 5
% Multipoint trajectories

%% Computed velocities at path points and imposed initial and final velocity
clc;
clear;

qk=[0 20 0 20];
tk=[0 4 8 12];

% qk=[3 -2 -5 0 6 12 8];
% tk=[0 5 7 8 10 15 18];

Ts=0.001;

dqi=-3;
dqf=3;

[q,dq,ddq,dddq,dqk]=multipoint_traj_computed_vel(qk,tk,Ts,dqi,dqf,true);

%% Continuos acceleretaion at path points and imposed initial and final velocities
clc;
clear;

qk=[0 20 0 20];
tk=[0 4 8 12];

% qk=[3 -2 -5 0 6 12 8];
% tk=[0 5 7 8 10 15 18];

Ts=0.001;

dqi=-3;
dqf=3;

[q,dq,ddq,dddq,dqk]=multipoint_traj_continuos_acc(qk,tk,Ts,dqi,dqf,true);
dqk=dqk';
