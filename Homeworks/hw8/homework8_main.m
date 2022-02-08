%% Homework 8
% Operational space trajectories with path primitives

%% Operational space trajectory - Forward example
clc;
clear;
close all

pos_tot=[];
vel_tot=[];
acc_tot=[];
jerk_tot=[];
sampling=0.01;
%
% first rectilinear path

pi=[0 0 0]';
pf=[1 0 0]';

[pos,vel,acc,jerk]=rectilinear_path_operational(pi,pf,sampling);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% first circular path

pi=[1 0 0]';
pf=[2 1 0]';
c=[1 1 0]';

% [pos,vel,acc,jerk]=circular_path_operational(pi,pf,c,normal,sampling);
[pos,vel,acc,jerk]=circular_path_operational(pi,pf,c,sampling,1);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% second circular path

pi=[2 1 0]';
pf=[2 1 2]';
c=[2 1 1]';

[pos,vel,acc,jerk]=circular_path_operational(pi,pf,c,sampling,-1);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% second rectilinear path
 
pi=[2 1 2]';
pf=[2 0 2]';

[pos,vel,acc,jerk]=rectilinear_path_operational(pi,pf,sampling);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];
 
% visualization

figure;
sgtitle('Operational space trajectory - Forward')
subplot(2,2,1)
plot3(pos_tot(1,:),pos_tot(2,:),pos_tot(3,:));
hold on
plot3(pos_tot(1,1),pos_tot(2,1),pos_tot(3,1),'*','Color','g');
plot3(pos_tot(1,end),pos_tot(2,end),pos_tot(3,end),'*','Color','r');
title('Position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,2)
plot3(vel_tot(1,:),vel_tot(2,:),vel_tot(3,:));
hold on
plot3(vel_tot(1,1),vel_tot(2,1),vel_tot(3,1),'*','Color','g');
plot3(vel_tot(1,end),vel_tot(2,end),vel_tot(3,end),'*','Color','r');
title('Velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,3)
plot3(acc_tot(1,:),acc_tot(2,:),acc_tot(3,:));
hold on
plot3(acc_tot(1,1),acc_tot(2,1),acc_tot(3,1),'*','Color','g');
plot3(acc_tot(1,end),acc_tot(2,end),acc_tot(3,end),'*','Color','r');
title('Acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,4)
plot3(jerk_tot(1,:),jerk_tot(2,:),jerk_tot(3,:));
hold on
plot3(jerk_tot(1,1),jerk_tot(2,1),jerk_tot(3,1),'*','Color','g');
plot3(jerk_tot(1,end),jerk_tot(2,end),jerk_tot(3,end),'*','Color','r');
title('Jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
saveas(gcf,'motion_primitives_forward.png')


%% Operational space trajectory - Backward example
clc;
clear;
close all;
pos_tot=[];
vel_tot=[];
acc_tot=[];
jerk_tot=[];
sampling=0.01;

% first rectilinear path

pf=[2 1 2]';
pi=[2 0 2]';

[pos,vel,acc,jerk]=rectilinear_path_operational(pi,pf,sampling);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% first circular path

pi=[2 1 2]';
pf=[2 1 0]';
c=[2 1 1]';

[pos,vel,acc,jerk]=circular_path_operational(pi,pf,c,sampling,1);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];


% second circular path

pi=[2 1 0]';
pf=[1 0 0]';
c=[1 1 0]';

[pos,vel,acc,jerk]=circular_path_operational(pi,pf,c,sampling,1);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% second rectilinear path

pi=[1 0 0]';
pf=[0 0 0]';

[pos,vel,acc,jerk]=rectilinear_path_operational(pi,pf,sampling);
pos_tot=[pos_tot pos];
vel_tot=[vel_tot vel];
acc_tot=[acc_tot acc];
jerk_tot=[jerk_tot jerk];

% visualization

figure;
sgtitle('Operational space trajectory - Backwards')
subplot(2,2,1)
plot3(pos_tot(1,:),pos_tot(2,:),pos_tot(3,:));
hold on
plot3(pos_tot(1,1),pos_tot(2,1),pos_tot(3,1),'*','Color','g');
plot3(pos_tot(1,end),pos_tot(2,end),pos_tot(3,end),'*','Color','r');
title('Position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,2)
plot3(vel_tot(1,:),vel_tot(2,:),vel_tot(3,:));
hold on
plot3(vel_tot(1,1),vel_tot(2,1),vel_tot(3,1),'*','Color','g');
plot3(vel_tot(1,end),vel_tot(2,end),vel_tot(3,end),'*','Color','r');
title('Velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,3)
plot3(acc_tot(1,:),acc_tot(2,:),acc_tot(3,:));
hold on
plot3(acc_tot(1,1),acc_tot(2,1),acc_tot(3,1),'*','Color','g');
plot3(acc_tot(1,end),acc_tot(2,end),acc_tot(3,end),'*','Color','r');
title('Acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,4)
plot3(jerk_tot(1,:),jerk_tot(2,:),jerk_tot(3,:));
hold on
plot3(jerk_tot(1,1),jerk_tot(2,1),jerk_tot(3,1),'*','Color','g');
plot3(jerk_tot(1,end),jerk_tot(2,end),jerk_tot(3,end),'*','Color','r');
title('Jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
saveas(gcf,'motion_primitives_backward.png')
