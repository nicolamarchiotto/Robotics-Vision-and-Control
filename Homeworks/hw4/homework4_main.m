%% Homework 4
%Double S trajectories

%% Double S given dqmax ddqmax dddqmax
clc;
clear

qi=5;
qf=2;

Ts=0.01;

ti=0;

dqi=0;
dqf=1;

dqmax=5;
ddqmax=10;
dddqmax=30;

[q,dq,ddq,dddq,DT]=doubleS_given_dqmax_ddqmax_dddqmax(qi,qf,dqi,dqf,ti,Ts,dqmax,ddqmax,dddqmax, true);

%% Double S with preassigned duration

clc;
clear

qi=0;
qf=-10;

Ts=0.01;

ti=0;

DT=2;
alpha=1/3;
beta=1/5;


[q,dq,ddq,dddq]=doubleS_preassigned_duration(qi,qf,DT,ti,Ts,alpha,beta,true);





