%% Homework 2
% Implement functions for harmonic and cycloidal trajectory.

%% Inputs
clc
clear
%positions
qi=0;
qf=30;

%times
ti=0;
tf=3;

%sampling step, must be sampling_step<tf-ti
Ts=0.01;
t=ti:Ts:tf;


%% harmonic trajectory
[q,dq,ddq,dddq]=harmonicTrajectory(qi,qf,ti,tf,Ts);

figure;
sgtitle('Harmonic trajectory')
subplot(4,1,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]')
grid on
xlim([ti tf])
subplot(4,1,2);
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]')
grid on
xlim([ti tf])
subplot(4,1,3);
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(4,1,4);
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]')
grid on
xlim([ti tf])

saveas(gcf,'harmonic_trajectory.png')
%% cycloidal trajectory
[q,dq,ddq,dddq]=cycloidalTrajectory(qi,qf,ti,tf,Ts);

figure;
sgtitle('Cycloidal trajectory')
subplot(4,1,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]')
grid on
xlim([ti tf])
subplot(4,1,2);
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]')
grid on
xlim([ti tf])
subplot(4,1,3);
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(4,1,4);
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]')
grid on
xlim([ti tf])

saveas(gcf,'cycloidal_trajectory.png')

