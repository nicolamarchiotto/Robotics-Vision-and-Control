%% Homework 9
%Given three p1,p2,p3 points on a sphere with centre po and radious R,
%design the trajectory such that 1) the EE pass through the three points
%along the shortest path 2) and the z axis of the EE is always orthogonal to
%the sphere

% function [trajPosition, trajVelocity, trajAcc, trajTime]=homework9_main(showPlots)

clc;
clear;
close all;
showPlots=true;

robotStartPosition=[0.6465,-0.1091,0.7360]';
robotStartOrientation=[-0.0000,1.5708,-1.5708]';

%SPHERE COMPUTATION

sphereCentre=[2,2,2]';
sphereRadious=2;
sphereSampling=0.01;

[xSphere, ySphere, zSphere, params]=sphere(sphereCentre,sphereRadious,sphereSampling);

Indp1=10;
Indp2=789;
Indp3=1063;

spherePoint1=[xSphere(Indp1),ySphere(Indp1),zSphere(Indp1)]';
spherePoint2=[xSphere(Indp2),ySphere(Indp2),zSphere(Indp2)]';
spherePoint3=[xSphere(Indp3),ySphere(Indp3),zSphere(Indp3)]';

spherePoints=[spherePoint1,spherePoint2,spherePoint3];

%CHECK IF POINTS BELONG TO SPHERE
for i=1:length(spherePoints)
    term=(spherePoints(1,i)-sphereCentre(1))^2+(spherePoints(2,i)-sphereCentre(2))^2+(spherePoints(3,i)-sphereCentre(3))^2;
    if(abs(term - sphereRadious^2)>sphereRadious/1000)
        error("Point "+i+" does not belong to the sphere");
    end
end

%VECTOR CONNECTING POINTS WITH SPHERE CENTRE
v1=spherePoint1-sphereCentre;
v2=spherePoint2-sphereCentre;
v3=spherePoint3-sphereCentre;

%COMPUTATION OF THE ANGLES BETWEEN EACH PAIR OF SPHERE POINTS
angle12=acos(dot(v1 / norm(v1), v2 / norm(v2)));
arc12=sphereRadious*angle12;
angle23=acos(dot(v2 / norm(v1), v3 / norm(v2)));
arc23=sphereRadious*angle23;
angle31=acos(dot(v1 / norm(v1), v3 / norm(v2)));
arc31=sphereRadious*angle31;

angles=[angle12,angle23,angle31];

%COMPUTING THE ORDER OF THE PATH TO FOLLOW ESCLUDING THE GREATEST ANGLE
%=>ESCLUDING THE LARGEST PATH CONNECTING TWO POINTS IN THE SPHERE
[maxValue, maxIndex] = max(angles(:));

if(maxIndex==1)
    %     fprintf("Middle point 3\n");
    order=[spherePoint1,spherePoint3,spherePoint2];
    arcs=[arc13,arc23];
elseif(maxIndex==2)
    %     fprintf("Middle point 1\n");
    order=[spherePoint2,spherePoint1,spherePoint3];
    arcs=[arc12,arc31];
else
    %     fprintf("Middle point 2\n");
    order=[spherePoint1,spherePoint2,spherePoint3];
    arcs=[arc12,arc23];
end

%IF LAST SPHERE POINT IS FURTHER THAN THE FISRT FROM THE ROBOT START POSITION, FLIP ORDER

if(norm(robotStartPosition - order(:,1))>norm(robotStartPosition - order(:,end)))
    %     fprintf("Flip order")
    sup=order(:,3);
    order(:,3)=order(:,1);
    order(:,1)=sup;
    clear sup;
end

clear angle12;
clear angle23;
clear angle31;

%
%
%
%
%

%TRAJECTORY COMPUTATION

time=[];
pos=[];
vel=[];
acc=[];
angPos=[];
angVel=[];
angAcc=[];

eulAngles=[];

eulAngles=robotStartOrientation;


%
%
%EE POSITION
%
%
%
%
%RECTILINEAR SEGMENT
%
%
pathLength=norm(robotStartPosition-order(:,1));

ti=0;
tf=10;
Ts=0.01;
timeSup=ti:Ts:tf;
[s,ds,dds]=trajectory_5th_ti_tf(0,pathLength,0,0,0,0,ti,tf,Ts);

posSup=robotStartPosition-s.*((robotStartPosition-order(:,1))/pathLength);
velSup=ds.*((robotStartPosition-order(:,1))/pathLength);
accSup=dds.*((robotStartPosition-order(:,1))/pathLength);
tSup=ones(1,size(s,2)).*((robotStartPosition-order(:,1))/pathLength);


pos=[pos posSup];
vel=[vel velSup];
acc=[acc accSup];
time=[time timeSup];

%
%
%
%FIRST CIRCULAR SEGMENT
%
%
%

p1=order(:,1);
p2=order(:,2);
p0=sphereCentre;

v1=p1-p0;
v2=p2-p0;

%axis of the circle centered in sphereCentre
circleAxis=cross(v2,v1);
angle=acos(dot(v1 / norm(v1), v2 / norm(v2)));
if(circleAxis==0)
    if(v1(1)~=0)
        v2=[v1(2);v1(1);v1(3)];
    elseif (v1(2)~=0)
        v2=[v1(1);-v1(3);v1(2)];
    elseif (v1(3)~=0)
        v2=[v1(1);v1(3);v1(2)];
    end
    circleAxis=-cross(v1,v2);
    angle=pi;
end

arcLenght=sphereRadious*angle;
zAxCirc=circleAxis/norm(circleAxis);
xAxCirc=(p1-p0)/norm(p1-p0);
yAxCirc=cross(xAxCirc,zAxCirc);

R=[xAxCirc,yAxCirc,zAxCirc];
ti=10;
tf=20;
timeSup=ti:Ts:tf;
[s,ds,dds]=trajectory_5th_ti_tf(0,arcLenght,0,0,0,0,ti,tf,Ts);
posSup=sphereCentre+R*[sphereRadious*cos(s/sphereRadious);sphereRadious*sin(s/sphereRadious);zeros(1,size(s,2))];
velSup=R*[-ds.*sin(s/sphereRadious);ds.*cos(s/sphereRadious);zeros(1,size(s,2))];
accSup=R*[-ds.^2.*cos(s/sphereRadious)/sphereRadious-dds.*sin(s/sphereRadious);-ds.^2.*sin(s/sphereRadious)/sphereRadious+dds.*cos(s/sphereRadious);zeros(1,size(s,2))];

tSup=R*[-sin(s/sphereRadious);cos(s/sphereRadious);zeros(1,size(s,2))];
nSup=R*[-cos(s/sphereRadious)/sphereRadious;-sin(s/sphereRadious)/sphereRadious;zeros(1,size(s,2))];
nSup=nSup./vecnorm(nSup);
bSup=cross(tSup,nSup);

Reul = [tSup(:,1), bSup(:,1), nSup(:,1)];
eulAngle = [atan2(sqrt(Reul(1,3)^2 + Reul(2,3)^2), Reul(3,3));
    atan2(Reul(2,3), Reul(1,3));
    atan2(Reul(3,2), -Reul(3,1)); ];
eulAngles=[eulAngles eulAngle];

pos=[pos posSup];
vel=[vel velSup];
acc=[acc accSup];
time=[time timeSup];

%
%
%
%SECOND CIRCULAR SEGMENT
%
%
%

p1=order(:,2);
p2=order(:,3);
p0=sphereCentre;

v1=p1-p0;
v2=p2-p0;

%axis of the circle centered in sphereCentre
circleAxis=cross(v2,v1);
angle=acos(dot(v1 / norm(v1), v2 / norm(v2)));
if(circleAxis==0)
    if(v1(1)~=0)
        v2=[v1(2);v1(1);v1(3)];
    elseif (v1(2)~=0)
        v2=[v1(1);-v1(3);v1(2)];
    elseif (v1(3)~=0)
        v2=[v1(1);v1(3);v1(2)];
    end
    circleAxis=-cross(v1,v2);
    angle=pi;
end

arcLenght=sphereRadious*angle;
zAxCirc=circleAxis/norm(circleAxis);
xAxCirc=(p1-p0)/norm(p1-p0);
yAxCirc=cross(xAxCirc,zAxCirc);

R=[xAxCirc,yAxCirc,zAxCirc];
ti=20;
tf=30;
timeSup=ti:Ts:tf;
[s,ds,dds]=trajectory_5th_ti_tf(0,arcLenght,0,0,0,0,ti,tf,Ts);
posSup=sphereCentre+R*[sphereRadious*cos(s/sphereRadious);sphereRadious*sin(s/sphereRadious);zeros(1,size(s,2))];
velSup=R*[-ds.*sin(s/sphereRadious);ds.*cos(s/sphereRadious);zeros(1,size(s,2))];
accSup=R*[-ds.^2.*cos(s/sphereRadious)/sphereRadious-dds.*sin(s/sphereRadious);-ds.^2.*sin(s/sphereRadious)/sphereRadious+dds.*cos(s/sphereRadious);zeros(1,size(s,2))];

tSup=R*[-sin(s/sphereRadious);cos(s/sphereRadious);zeros(1,size(s,2))];
nSup=R*[-cos(s/sphereRadious)/sphereRadious;-sin(s/sphereRadious)/sphereRadious;zeros(1,size(s,2))];
nSup=nSup./vecnorm(nSup);
bSup=cross(tSup,nSup);
tPos=posSup;

Reul = [tSup(:,1), bSup(:,1), nSup(:,1)];
eulAngle = [atan2(sqrt(Reul(1,3)^2 + Reul(2,3)^2), Reul(3,3));
    atan2(Reul(2,3), Reul(1,3));
    atan2(Reul(3,2), -Reul(3,1)); ];
eulAngles=[eulAngles eulAngle];

Reul = [tSup(:,end), bSup(:,end), nSup(:,end)];
eulAngle = [atan2(sqrt(Reul(1,3)^2 + Reul(2,3)^2), Reul(3,3));
    atan2(Reul(2,3), Reul(1,3));
    atan2(Reul(3,2), -Reul(3,1)); ];
eulAngles=[eulAngles eulAngle];

a=eulAngle(1);
b=eulAngle(2);
c=eulAngle(3);

pos=[pos posSup];
vel=[vel velSup];
acc=[acc accSup];
time=[time timeSup];

%
%
% EE ORIENTATION
%
%

for i=1:size(eulAngles,2)-1
    ti=(i-1)*10;
    tf=10+(i-1)*10;
    angI=eulAngles(:,i);
    angF=eulAngles(:,i+1);
    [s,ds,dds]=trajectory_5th_ti_tf(0,norm(angF-angI),0,0,0,0,ti,tf,Ts);
    angPosSup=angI+s.*((angF-angI)/(norm(angF-angI)));
    angVelSup=ds.*((angF-angI)/(norm(angF-angI)));
    angAccSup=dds.*((angF-angI)/(norm(angF-angI)));
    angPos=[angPos angPosSup];
    angVel=[angVel angVelSup];
    angAcc=[angAcc angAccSup];
end


%
%
%
%
%VISUALIZATION
%
%
%
%

%SPHERE PLOT

if(showPlots)
    % plot sphere
    figure;
    plot3(xSphere,ySphere,zSphere,'.','MarkerSize',2)
    title("Sphere with centre in (" + sphereCentre(1)+","+sphereCentre(2)+","+sphereCentre(3) +") and radious " + sphereRadious)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on;
    hold on;
    
    %plot path
    plot3(pos(1,:),pos(2,:),pos(3,:),'.','Color','r','MarkerSize',1)
    
    %plot relevant points
    plot3(order(1,1),order(2,1),order(3,1),'*','Color','m','MarkerSize',12)
    plot3(order(1,2),order(2,2),order(3,2),'*','Color','b','MarkerSize',12)
    plot3(order(1,3),order(2,3),order(3,3),'*','Color','r','MarkerSize',12)
    plot3(robotStartPosition(1),robotStartPosition(2),robotStartPosition(3),'*','Color','g','MarkerSize',12)
    title("Sphere with centre in (" + sphereCentre(1)+","+sphereCentre(2)+","+sphereCentre(3) +") and radious " + sphereRadious,"Path along: (" + order(1,1)+","+order(2,1)+","+order(3,1) +"), (" + order(1,2)+","+order(2,2)+","+order(3,2) +"),(" + order(1,3)+","+order(2,3)+","+order(3,3) +")")
    
%     u=0:0.1:1;
%     for i=1:100:size(angPos,2)
%         rotm = eul2rotm(angPos(:,i)','XYZ');
%         rayt=pos(:,i)+rotm(:,1).*[u;u;u];
%         rayn=pos(:,i)+rotm(:,2).*[u;u;u];
%         rayb=pos(:,i)+rotm(:,3).*[u;u;u];
%         
%         plot3(rayt(1,:),rayt(2,:),rayt(3,:),'-','Color','g')
%         plot3(rayn(1,:),rayn(2,:),rayn(3,:),'-','Color','b')
%         plot3(rayb(1,:),rayb(2,:),rayb(3,:),'-','Color','m')
%         
%     end
    saveas(gcf,'sphere_with_path.png')
end

% TRAJECTORY EE POSITION VISUALIZATION.

if(showPlots)
    figure;
    sgtitle('EE POSITION AND VELOCITY')
    subplot(3,3,1)
    plot(time,pos(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Position x [m]')
    hold on
    grid on
    subplot(3,3,4)
    plot(time,pos(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Position y [m]')
    grid on
    subplot(3,3,7)
    plot(time,pos(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Position z [m]')
    grid on
    subplot(3,3,2)
    plot(time,vel(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Velocity x [m/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,5)
    plot(time,vel(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Velocity y [m/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,8)
    plot(time,vel(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Velocity z [m/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,3)
    plot(time,acc(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Acceleration x [m/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,6)
    plot(time,acc(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Acceleration y [m/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,9)
    plot(time,acc(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Acceleration z [m/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    saveas(gcf,'EE_trajectory_position.png')
end

%TRAJECTORY EE ORIENTATION VISUALIZATION


if(showPlots)
    figure;
    sgtitle('EE ANGULAR POSITION AND ANGULAR VELOCITY')
    subplot(3,3,1)
    plot(time,angPos(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Pos x [rad]')
    hold on
    grid on
    subplot(3,3,4)
    plot(time,angPos(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Pos y [rad]')
    grid on
    subplot(3,3,7)
    plot(time,angPos(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Pos z [rad]')
    grid on
    subplot(3,3,2)
    plot(time,angVel(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Vel x [rad/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,5)
    plot(time,angVel(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Vel y [rad/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,8)
    plot(time,angVel(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Vel z [rad/s]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,3)
    plot(time,angAcc(1,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Acc x [rad/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,6)
    plot(time,angAcc(2,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Acc y [rad/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    subplot(3,3,9)
    plot(time,angAcc(3,:))
    xlim([time(1) time(end)])
    xlabel('time [sec]')
    ylabel('Ang Acc z [rad/s^2]')
    h = yline(0, 'r--', 'LineWidth', 2);
    grid on
    saveas(gcf,'EE_trajectory_orientation.png')
end

trajPosition=[pos; angPos];
trajVelocity=[vel; angVel];
trajAcc=[acc; angAcc];
trajTime=time;

% end