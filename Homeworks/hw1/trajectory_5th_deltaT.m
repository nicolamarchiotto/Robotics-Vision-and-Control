function [q,dq,ddq,dddq,ddddq]=trajectory_5th_deltaT(qi,qf,dqi,dqf,ddqi,ddqf,ti,tf,Ts)

Q=[qi;dqi;ddqi;qf;dqf;ddqf];

t=ti:Ts:tf;

dt=tf-ti;

X=[ 1,      0,      0,      0,      0,      0;
    0,      1,      0,      0,      0,      0;
    0,      0,      2,      0,      0,      0;
    1,      dt,     dt^2,   dt^3,   dt^4,   dt^5;
    0,      1,      2*dt,   3*dt^2, 4*dt^3, 5*dt^4;
    0,      0,      2,      6*dt,   12*dt^2,20*dt^3];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);
a4=A(5);
a5=A(6);

q=a5*(t-ti).^5+a4*(t-ti).^4+a3*(t-ti).^3+a2*(t-ti).^2+a1*(t-ti)+a0;
dq=5*a5*(t-ti).^4+4*a4*(t-ti).^3+3*a3*(t-ti).^2+2*a2*(t-ti)+a1;
ddq=20*a5*(t-ti).^3+12*a4*(t-ti).^2+6*a3*(t-ti)+2*a2;
dddq=60*a5*(t-ti).^2+24*a4*(t-ti)+6*a3;
ddddq=120*a5*(t-ti)+24*a4;

end