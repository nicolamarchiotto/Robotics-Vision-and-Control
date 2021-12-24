function [q,dq,ddq,dddq]=cycloidalTrajectory(qi,qf,ti,tf,Ts)
    DQ=qf-qi;
    DT=tf-ti;
    t=ti:Ts:tf;
    
    q=DQ*((t-ti)/DT - sin(2*pi*(t-ti)/DT)/(2*pi))+qi;
    dq=(DQ/DT)*(1-cos(2*pi*(t-ti)/DT));
    ddq=(2*pi*DQ)/(DT^2)*(sin(2*pi*(t-ti)/DT));
    dddq=((4*pi^2*DQ)/(DT^3))*(cos(2*pi*(t-ti)/DT));
end