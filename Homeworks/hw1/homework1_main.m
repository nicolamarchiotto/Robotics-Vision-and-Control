%% Homework 1
%Implement functions for trajectory of the 3rd, 5th and 7th
%order in both formulations ti-td and DeltaT.

%% Inputs

%positions
qi=4;
qf=10;

%velocities
dqi=0;
dqf=5;

%accelerations
ddqi=0;
ddqf=0;

%jerk
dddqi=0;
dddqf=5 ;

%times
ti=5;
tf=10;

%Time sampling, must be Ts<tf-ti
Ts=0.01;
t=ti:Ts:tf;

%% 3rd grade, first formulation, delta T
[q,dq,ddq]=trajectory_3rd_deltaT(qi,qf,dqi,dqf,ti,tf,Ts);

figure;
sgtitle('Trajectories 3rd, DeltaT')
subplot(3,1,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]') 
grid on
xlim([ti tf])
subplot(3,1,2)
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(3,1,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]') 
grid on
saveas(gcf,'3rd_poly_deltaT.png')
xlim([ti tf])

%% 3rd grade, second formulation, ti, tf
[q,dq,ddq]=trajectory_3rd_ti_tf(qi,qf,dqi,dqf,ti,tf,Ts);

figure;
sgtitle('Trajectory 3rd, ti tf')
subplot(3,1,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]') 
grid on
xlim([ti tf])
subplot(3,1,2)
plot(t,dq)
xlim([ti tf])
xlabel('time [sec]')
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(3,1,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]') 
grid on
xlim([ti tf])

saveas(gcf,'3rd_poly_ti_tf.png')

%% 5th grade, first formulation, delta T
[q,dq,ddq,dddq,ddddq]=trajectory_5th_deltaT(qi,qf,dqi,dqf,ddqi,ddqf,ti,tf,Ts);

figure;
sgtitle('Trajectories 5th, DeltaT')
subplot(3,2,1);
plot(t,q)
xlabel('time [sec]')    
ylabel('Position [rad]') 
grid on
xlim([ti tf])
subplot(3,2,2)
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(3,2,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(3,2,4)
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]') 
grid on
xlim([ti tf])
subplot(3,2,5)
plot(t,ddddq)
xlabel('time [sec]') 
ylabel('Snap [rad/s^4]') 
grid on
xlim([ti tf])
saveas(gcf,'5th_poly_deltaT.png')
%% 5th grade, second formulation, ti, tf
[q,dq,ddq,dddq,ddddq]=trajectory_5th_ti_tf(qi,qf,dqi,dqf,ddqi,ddqf,ti,tf,Ts);

figure;
sgtitle('Trajectories 5th, ti tf')
subplot(3,2,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]')
grid on
xlim([ti tf])
subplot(3,2,2)
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(3,2,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(3,2,4)
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]') 
grid on
xlim([ti tf])
subplot(3,2,5)
plot(t,ddddq)
xlabel('time [sec]') 
ylabel('Snap [rad/s^4]') 
grid on
xlim([ti tf])
saveas(gcf,'5th_poly_ti_tf.png')
%% 7th grade, first formulation, delta T
[q,dq,ddq,dddq,ddddq,dddddq,ddddddq]=trajectory_7th_deltaT(qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf,ti,tf,Ts);

figure;
sgtitle('Trajectories 7th, DeltaT')
subplot(4,2,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]') 
grid on
xlim([ti tf])
subplot(4,2,2)
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(4,2,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(4,2,4)
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]') 
grid on
xlim([ti tf])
subplot(4,2,5)
plot(t,ddddq)
xlabel('time [sec]') 
ylabel('Snap [rad/s^4]')
grid on
xlim([ti tf])
subplot(4,2,6)
plot(t,dddddq)
xlabel('time [sec]') 
ylabel('Crackle [rad/s^5]') 
grid on
xlim([ti tf])
subplot(4,2,7)
plot(t,ddddddq)
xlabel('time [sec]') 
ylabel('Pop [rad/s^6]') 
grid on
xlim([ti tf])

saveas(gcf,'7th_poly_deltaT.png')
%% 7th grade, second formulation, ti, tf
[q,dq,ddq,dddq,ddddq,dddddq,ddddddq]=trajectory_7th_ti_tf(qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf,ti,tf,Ts);

figure;
sgtitle('Trajectories 7th, ti tf')
subplot(4,2,1);
plot(t,q)
xlabel('time [sec]') 
ylabel('Position [rad]') 
grid on
xlim([ti tf])
subplot(4,2,2)
plot(t,dq)
xlabel('time [sec]') 
ylabel('Velocity [rad/s]') 
grid on
xlim([ti tf])
subplot(4,2,3)
plot(t,ddq)
xlabel('time [sec]') 
ylabel('Acceleration [rad/s^2]')
grid on
xlim([ti tf])
subplot(4,2,4)
plot(t,dddq)
xlabel('time [sec]') 
ylabel('Jerk [rad/s^3]') 
grid on
xlim([ti tf])
subplot(4,2,5)
plot(t,ddddq)
xlabel('time [sec]') 
ylabel('Snap [rad/s^4]')
grid on
xlim([ti tf])
subplot(4,2,6)
plot(t,dddddq)
xlabel('time [sec]') 
ylabel('Crackle [rad/s^5]') 
grid on
xlim([ti tf])
subplot(4,2,7)
plot(t,ddddddq)
xlabel('time [sec]') 
ylabel('Pop [rad/s^6]') 
grid on
xlim([ti tf])

saveas(gcf,'7th_poly_ti_tf.png')