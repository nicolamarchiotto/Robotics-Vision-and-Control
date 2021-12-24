
%% robot parameters
clc;
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

% gravity, maybe there should be a minus, in the code there's some minus
% which compensate
gravity = [0 0 9.81]';

% we to have assume no friction proportional to the velocity, in reality we
% would have a n vector for frictions for all dof
Fv = 0;
% and to have no static friction
Fs = 0;
% no external wrench? => Robot no touching extern object
he = [0 0 0 0 0 0]';
% initial config
q0 = [0 -5*pi/6 pi/6 -pi/3 pi/2 0]';
%derivative so velocity
dotq0 = [0 0 0 0 0 0]';

%using kinematics i can compute the pose of the end effector
x0 = kinematics(dh, q0);
dotx0 = [0 0 0 0 0 0]';

%desidered position, want to move in 0...0 position with 0 velocity, it's
%an example
qd = [0 0 0 0 0 0]';
dotqd = [0 0 0 0 0 0]';

xd = kinematics(dh, qd);
dotxd = [0 0 0 0 0 0]';

%% Joint Controller PD + gravity
% matrices for proportional and derivatives gain
KP_J = diag([0.5 0.5 0.5 0.25 0.255 0.25]);
KD_J = diag([1 1 1 0.1 0.1 0.1]);
%KP_J = eye(dh.dof); 
%KD_J = eye(dh.dof);

%% Operational Controller PD + gravity
% matrices for proportional and derivatives gain 
%KP_O = 20.*eye(6);
%KD_O = 30.*eye(6);
KP_O = diag([0.12 0.12 0.12 0.01 0.01 0.01]);
KD_O = diag([0.1 0.1 0.1 0.005 0.005 0.005]);

%% urdf
cd ./ur_description_noEE/urdf
%cd ./ur_description/urdf
UR5_robot = importrobot('ur5.urdf');
showdetails(UR5_robot)
figure;
config = homeConfiguration(UR5_robot);
qi = [-1.6007 -1.7271 -2.203 -0.808 1.5951 -0.031]';
for i=1:6
    config(i).JointPosition = qi(i);
end
show(UR5_robot,config);
xlim([-0.4 0.4])
ylim([-0.5 0.5])
zlim([-0.1 0.6])
cd ..
cd ..
return

%%
%if we want to use the robot with the EE, we should increase the mass and
%chang thee gravity center of the last link, also the inertia



