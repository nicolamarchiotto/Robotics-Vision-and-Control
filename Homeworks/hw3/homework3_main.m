%% Homework 3
%implement functions for trapeizoidal velocity trajectories

%% Equal acceleration and decelleration times, tc given                            
%must be tc <= tm=(tf-ti)/2]

clc;
clear

qi=5;
qf=2;

ti=1;
tf=6;

Ts=0.001;
tc=2;

[q,dq,ddq]=trap_vel_equal_acc_decc_times_given_tc(qi,qf,ti,tf,Ts,tc,true);


%% Equal acceleration and decelleration times, velocity dqc given

clc;
clear

qi=5;
qf=2;

ti=1;
tf=5;

Ts=0.001;
dqc=1;
[q,dq,ddq]=trap_vel_equal_acc_decc_times_given_dqc(qi,qf,ti,tf,Ts,dqc,true);

%% Equal acceleration and decelleration times, acceleration ddqc given
clc;
clear

qi=5;
qf=2;

ti=1;
tf=5;

Ts=0.001;

ddqc=2;
[q,dq,ddq]=trap_vel_equal_acc_decc_times_given_ddqc(qi,qf,ti,tf,Ts,ddqc,true);

%% Equal acceleration and decelleration times, Preassigned ddqc and dqc
clc;
clear

qi=5;
qf=2;

Ts=0.001;

dqc=0.5;
ddqc=2;

[q,dq,ddq,tf]=trap_vel_equal_acc_decc_times_given_dqc_and_ddqc(qi,qf,Ts,dqc,ddqc,true);


%% Trajectory with dqi,dqf!=0, ddqi,ddqf=0
% 
%
% 
%
%% Trajectory with dqi,dqf!=0, ddqi,ddqf=0, preassigned duration DT and maximum accelerstion ddqcmax
clc;
clear

qi=5;
qf=2;

ti=1;
tf=5;

Ts=0.001;


ddqcmax=2;
dqi=0;
dqf=1;

[q,dq,ddq]=trap_vel_initial_and_final_vel_not_null_given_ti_tf_and_ddqcmx(qi,qf,dqi,dqf,ti,tf,Ts,ddqcmax,true);



%% Trajectory with dqi,dqf!=0, ddqi,ddqf=0, preassigned ddqcmax and dqcmax
clc;
clear

qi=5;
qf=2;

Ts=0.001;

dqi=0;
dqf=1;
dqcmax=10;
ddqcmax=40;



[q,dq,ddq,tf]=trap_vel_initial_and_final_vel_not_null_given_dqcmx_and_ddqcmx(qi,qf,dqi,dqf,Ts,dqcmax,ddqcmax,true);





