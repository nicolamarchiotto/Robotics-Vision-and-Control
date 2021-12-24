function [q,dq,ddq]=trajectory_3rd_ti_tf(qi,qf,dqi,dqf,ti,tf,Ts)

Q=[qi;dqi;qf;dqf];

t=ti:Ts:tf;

X=[1,ti,ti^2,ti^3;
    0,1,2*ti,3*ti^2;
    1,tf,tf^2,tf^3;
    0,1,2*tf,3*tf^2];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);

q=a3*t.^3+a2*t.^2+a1*t+a0;
dq=3*a3*t.^2+2*a2*t+a1;
ddq=6*a3*t+2*a2;
end