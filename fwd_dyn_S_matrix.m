function [sys,x0,str,ts] = fwd_dyn_S_matrix(t,x,u,flag,dh,he,gravity,q0,qd0)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization % 
  %Define the parameters of the differential eq that this block is gonna solve
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(dh.dof, q0, qd0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  % the s function solve a system in the state space reppresentation, do the form is (used for non linear function ) :
  %dotx=f(x,u) as the system eq, and the output eq y=h(x,u), h non linear function
  case 1,
    sys=mdlDerivatives(t,x,u,dh,he,gravity);

  %%%%%%%%%%%
  % Outputs %S
  %%%%%%%%%%%
  %this is the output eq, y=h(x,u,t), t if we have a time variance eq.
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(n, q0, qd0)

sizes = simsizes;
%n state for configuration q and n for velocities dotq
sizes.NumContStates  = 2*n;
sizes.NumDiscStates  = 0;
%2n outputs, n for configuration q and n for velocities dotq
sizes.NumOutputs     = 2*n;
%number of inputs, the torque applied at joint level
sizes.NumInputs      = n;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
%initial conditions
x0  = [q0; qd0];  %starting positions and velocities
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,dh,he,gravity)
    % x = [q; qdot]
    %first n components of vector x, look at notes
    q = x(1 : dh.dof);
    %last n components of vector x, look at notes
    qdot = x(dh.dof+1 : 2*dh.dof);
    %the torque is the input
    tau = u;
    
    %computation of the following matrices in next year advanced robot
    %control
    
    %computation of the inertial matrix as a function of the current
    %configuration
    B = double(B_recursive_NewtonEulero(dh, q));
    %computation of the C as a function of the current
    %configuration q and qdot
    C = double(C_recursive_NewtonEulero(dh, q, qdot));
    %computation of the gravity compensation as a function of the current
    %configuration q and the gravity vector
    G = inv_dyn_recursive_NewtonEulero(dh, q, [0 0 0 0 0 0]', [0 0 0 0 0 0]', gravity);
    %the jacobian in the case of an external wrench
    J = double(Jacobian(dh, q));
    
    qddot = inv(B) * (tau - C*qdot - G - J'*he);
    
    %the output of this fucntion is xdot=[qdot, qddot], qddot is the same function on the notes 
    sys = [qdot; qddot];
%sys = A*x + B*u;

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

%output is actually the states, configuration q and velocity qdot, as reported in the notes
sys = x;

% end mdlOutputs
