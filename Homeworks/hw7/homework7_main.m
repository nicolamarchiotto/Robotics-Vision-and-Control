%% Homework 7
% Model based trajectory planning

%% IMPORTANT
%In order to be able to run the following code you will have to add to path
%the whole MatlabScipts folder with subfolders, if your current folder is hw7 you can
%execute the following command to do so

addpath('../../../MatlabScripts')
addpath('../../../MatlabScripts/Homeworks/hw6')


%% Computation of the initial trajectory and the relative torques

clc;
clear;

% Robot parameters
%the one in the davit hartenberg table, provided by the manufactor
dh.d = [0.089159 0 0 0.10915 0.09465 0.0823];
% mass of each link
dh.m = [3.7000 8.3930 2.2750 1.2190 1.2190 0.1879];
dh.alpha = [pi/2 0 0 pi/2 -pi/2 0];
dh.a = [0 -0.42500 -0.39225 0 0 0];
% centre of mass of each link
cm1 = [0.0 -0.02561 0.00193];
cm2 = [0.2125 0.0 0.11336];
cm3 = [0.15 0.0 0.0265];
cm4 = [0.0 -0.0018 0.01634];
cm5 = [0.0 0.0018 0.01634];
cm6 = [0.0 0.0 -0.001159];

dh.cm = [cm1' cm2' cm3' cm4' cm5' cm6'];
dh.dof = 6;
dh.issym = false;

%inertia, we assume that the inertia matrix of each joint is diagonal,
%inertia w.r.t. the main axis, not the rotation axis
i1 = [0.010267 0.010267 0.00666];
i2 = [0.2269 0.2269 0.0151];
i3 = [0.0312168 0.0312168 0.004095];
i4 = [0.002559898976 0.002559898976 0.0021942];
i5 = [0.002559898976 0.002559898976 0.0021942];
i6 = [8.46958911216e-5 8.46958911216e-5 0.0001321171875];

%information of the inertia of each link w.r.t the centre of mass
dh.I = zeros(3, 3, 6);
dh.I(:,:,1) = [i1(1) 0 0; 0 i1(2) 0; 0 0 i1(3)];
dh.I(:,:,2) = [i2(1) 0 0; 0 i2(2) 0; 0 0 i2(3)];
dh.I(:,:,3) = [i3(1) 0 0; 0 i3(2) 0; 0 0 i3(3)];
dh.I(:,:,4) = [i4(1) 0 0; 0 i4(2) 0; 0 0 i4(3)];
dh.I(:,:,5) = [i5(1) 0 0; 0 i5(2) 0; 0 0 i5(3)];
dh.I(:,:,6) = [i6(1) 0 0; 0 i6(2) 0; 0 0 i6(3)];

%we must have 6 trajectories for the given 6 dof, so 6 pos, 6 vel and 6 acc

tk=[1 2 3 4 5 6 7 8 9 10];
Ts=0.01;

n=size(tk,2);
t=tk(1):Ts:tk(n);

%gravity=[0 0 9.81]';
gravity=[0 0 0]';

QK=zeros(6,tk(size(tk,2)));

QK(1,:)=[1 2 3 4 5 6 7 8 9 10];
QK(2,:)=[1 2 3 4 5 6 7 8 9 10];
QK(3,:)=[1 2 3 4 5 6 7 8 9 10];
QK(4,:)=[1 2 3 4 5 6 7 8 9 10];
QK(5,:)=[1 2 3 4 5 6 7 8 9 10];
QK(6,:)=[1 2 3 4 5 6 7 8 9 10];

Q=zeros(6,1+(tk(size(tk,2))-tk(1))/Ts);
DQ=zeros(6,1+(tk(size(tk,2))-tk(1))/Ts);
DDQ=zeros(6,1+(tk(size(tk,2))-tk(1))/Ts);

for i=1:1:6
    [Q(i,:),DQ(i,:),DDQ(i,:),~,~]=multipoint_comp_based_on_acc(QK(i,:),tk,Ts,0,0,false);
end

%% Computing the torques for each joint for the given joint trajectories
clc;

TAU=zeros(6,size(tk,2));
for i=1:(1+(tk(size(tk,2))-tk(1))/Ts)
    TAU(:,i) = inv_dyn_recursive_NewtonEulero(dh, Q(:,i), DQ(:,i), DDQ(:,i), gravity, [0 0 0 0 0 0]');
end

figure;
sgtitle('Torques of the input trajectory');
subplot(3,2,1);
plot(t,TAU(1,:))
title('Joint 1')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on

subplot(3,2,2);
plot(t,TAU(2,:))
title('Joint 2')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on

subplot(3,2,3);
plot(t,TAU(3,:))
title('Joint 3')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on

subplot(3,2,4);
plot(t,TAU(4,:))
title('Joint 4')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on

subplot(3,2,5);
plot(t,TAU(5,:))
title('Joint 5')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on

subplot(3,2,6);
plot(t,TAU(6,:))
title('Joint 6')
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk(1) tk(n)])
grid on
saveas(gcf,'model_base_traj_torques_without_constraints.png')


%% Computation of the new trajectory with the given constraints on the maximum torque
clc;

tau1_max=2;
tau2_max=5;
tau3_max=2;
tau4_max=5;
tau5_max=5;
tau6_max=5;

%scaling factor used to increase or decrease the duration in order to allow
%for at leat one torque to be equal to the maximum value

lamba=sqrt(min([tau1_max/max(abs(TAU(1,:))),tau2_max/max(abs(TAU(2,:))),tau3_max/max(abs(TAU(3,:))),tau4_max/max(abs(TAU(4,:))),tau5_max/max(abs(TAU(5,:))),tau6_max/max(abs(TAU(6,:)))]));

tk_hat=tk/lamba;

for i=1:1:6
    [Q_hat(i,:),DQ_hat(i,:),DDQ_hat(i,:),~,~]=multipoint_comp_based_on_acc(QK(i,:),tk_hat,Ts,0,0,false);
end

t_hat=linspace(tk_hat(1),tk_hat(n),size(Q_hat,2));

%computation of the new torques

TAU_hat=zeros(6,size(t_hat,2));
for i=1:size(t_hat,2)
    TAU_hat(:,i) = inv_dyn_recursive_NewtonEulero(dh, Q_hat(:,i), DQ_hat(:,i), DDQ_hat(:,i), gravity, [0 0 0 0 0 0]');
end

%Plot of the new torque for the time scaled trajectory

figure;
sgtitle('Torques of the time scaled trajectory');
subplot(3,2,1);
plot(t_hat,TAU_hat(1,:))
h = yline(tau1_max, 'r--', 'LineWidth', 2);
h = yline(-tau1_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 1 - Max torque imposed: %d', tau1_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on

subplot(3,2,2);
plot(t_hat,TAU_hat(2,:))
h = yline(tau2_max, 'r--', 'LineWidth', 2);
h = yline(-tau2_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 2 - Max torque imposed: %d', tau2_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on

subplot(3,2,3);
plot(t_hat,TAU_hat(3,:))
h = yline(tau3_max, 'r--', 'LineWidth', 2);
h = yline(-tau3_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 3 - Max torque imposed: %d', tau3_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on

subplot(3,2,4);
plot(t_hat,TAU_hat(4,:))
h = yline(tau4_max, 'r--', 'LineWidth', 2);
h = yline(-tau4_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 4 - Max torque imposed: %d', tau4_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on

subplot(3,2,5);
plot(t_hat,TAU_hat(5,:))
h = yline(tau5_max, 'r--', 'LineWidth', 2);
h = yline(-tau5_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 5 - Max torque imposed: %d', tau5_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on

subplot(3,2,6);
plot(t_hat,TAU_hat(6,:))
h = yline(tau6_max, 'r--', 'LineWidth', 2);
h = yline(-tau6_max, 'r--', 'LineWidth', 2);
title(sprintf('Joint 6 - Max torque imposed: %d', tau6_max))
hold on
xlabel('time [sec]')
ylabel('Torque [N*m]')
xlim([tk_hat(1) tk_hat(n)])
grid on
saveas(gcf,'model_base_traj_torques_with_constraints.png')
