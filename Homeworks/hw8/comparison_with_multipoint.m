%% Op Space - Multipoint based traj - Smoothed - Forward
clc;
clear;
p=zeros(3,5);

p(:,1)=[0 0 0];
p(:,2)=[1 0 0];
p(:,3)=[2 1 0];
p(:,4)=[2 1 2];
p(:,5)=[2 0 2];

t=[1 2 3 4 5];

% work component by component

Ts=0.01;
mu=1;
w=[1 1 1 1 1];

pos=[];
vel=[];
acc=[];
jerk=[];

for i=1:3
    [pos(i,:), vel(i,:), acc(i,:), jerk(i,:), ~]=smoothing_traj(p(i,:),t,Ts,mu,w,false);
end

figure;
sgtitle('Op Space - Multipoint based traj - Smoothed - Forward')
subplot(2,2,1)
plot3(pos(1,:),pos(2,:),pos(3,:));
hold on
plot3(pos(1,1),pos(2,1),pos(3,1),'*','Color','g');
plot3(pos(1,end),pos(2,end),pos(3,end),'*','Color','r');
title('Position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,2)
plot3(vel(1,:),vel(2,:),vel(3,:));
hold on
plot3(vel(1,1),vel(2,1),vel(3,1),'*','Color','g');
plot3(vel(1,end),vel(2,end),vel(3,end),'*','Color','r');
title('Velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,3)
plot3(acc(1,:),acc(2,:),acc(3,:));
hold on
plot3(acc(1,1),acc(2,1),acc(3,1),'*','Color','g');
plot3(acc(1,end),acc(2,end),acc(3,end),'*','Color','r');
title('Acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,4)
plot3(jerk(1,:),jerk(2,:),jerk(3,:));
hold on
plot3(jerk(1,1),jerk(2,1),jerk(3,1),'*','Color','g');
plot3(jerk(1,end),jerk(2,end),jerk(3,end),'*','Color','r');
title('Jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
saveas(gcf,'op_space_mltpts_traj_smoothed_forward.png')

%% Op Space - Multipoint based traj - Smoothed - Backward

clc;
clear;
p=zeros(3,5);

p(:,5)=[0 0 0];
p(:,4)=[1 0 0];
p(:,3)=[2 1 0];
p(:,2)=[2 1 2];
p(:,1)=[2 0 2];

t=[1 2 3 4 5];

% work component by component

Ts=0.01;
mu=1;
w=[1 1 1 1 1];

pos=[];
vel=[];
acc=[];
jerk=[];

for i=1:3
    [pos(i,:), vel(i,:), acc(i,:), jerk(i,:), ~]=smoothing_traj(p(i,:),t,Ts,mu,w,false);
end

figure;
sgtitle('Op Space - Multipoint based traj - Smoothed - Backward')
subplot(2,2,1)
plot3(pos(1,:),pos(2,:),pos(3,:));
hold on
plot3(pos(1,1),pos(2,1),pos(3,1),'*','Color','g');
plot3(pos(1,end),pos(2,end),pos(3,end),'*','Color','r');
title('Position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,2)
plot3(vel(1,:),vel(2,:),vel(3,:));
hold on
plot3(vel(1,1),vel(2,1),vel(3,1),'*','Color','g');
plot3(vel(1,end),vel(2,end),vel(3,end),'*','Color','r');
title('Velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,3)
plot3(acc(1,:),acc(2,:),acc(3,:));
hold on
plot3(acc(1,1),acc(2,1),acc(3,1),'*','Color','g');
plot3(acc(1,end),acc(2,end),acc(3,end),'*','Color','r');
title('Acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,4)
plot3(jerk(1,:),jerk(2,:),jerk(3,:));
hold on
plot3(jerk(1,1),jerk(2,1),jerk(3,1),'*','Color','g');
plot3(jerk(1,end),jerk(2,end),jerk(3,end),'*','Color','r');
title('Jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
saveas(gcf,'op_space_mltpts_traj_smoothed_backward.png')