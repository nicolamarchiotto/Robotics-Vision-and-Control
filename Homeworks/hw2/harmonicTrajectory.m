function [q,dq,ddq,dddq]=harmonicTrajectory(qi,qf,ti,tf,Ts)

t=ti:Ts:tf;

DQ=qf-qi;
DT=tf-ti;

q=(DQ/2)*(1-cos(pi*(t-ti)/DT))+qi;
dq=(DQ*pi)/(2*DT)*(sin(pi*(t-ti)/DT));
ddq=(DQ*pi^2)/(2*DT^2)*(cos(pi*(t-ti)/DT));
dddq=-(DQ*pi^3)/(2*DT^3)*(sin(pi*(t-ti)/DT));
% ddddq=-(DQ*pi^4)/(2*DT^4)*(sin(pi*(t-ti)/DT));
end