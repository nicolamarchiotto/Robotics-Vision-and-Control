function [q,dq,ddq]=trajectory_3rd_deltaT(qi,qf,dqi,dqf,ti,tf,Ts)

Q=[qi;dqi;qf;dqf];
dt=tf-ti;
t=ti:Ts:tf;


X=[1,0,0,0;
    0,1,0,0;
    1,dt,dt^2,dt^3;
    0,1,2*dt,3*dt^2];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);

q=a3*(t-ti).^3+a2*(t-ti).^2+a1*(t-ti)+a0;
dq=3*a3*(t-ti).^2+2*a2*(t-ti)+a1;
ddq=6*a3*(t-ti)+2*a2;

end