function [q,dq,ddq,dddq,ddddq]=trajectory_5th_ti_tf(qi,qf,dqi,dqf,ddqi,ddqf,ti,tf,Ts)

Q=[qi;dqi;ddqi;qf;dqf;ddqf];

t=ti:Ts:tf;

X=[ 1,ti,ti^2,ti^3,ti^4,ti^5;
    0,1,2*ti,3*ti^2,4*ti^3,5*ti^4;
    0,0,2,6*ti,12*ti^2,20*ti^3;
    1,tf,tf^2,tf^3,tf^4,tf^5;
    0,1,2*tf,3*tf^2,4*tf^3,5*tf^4;
    0,0,2,6*tf,12*tf^2,20*tf^3];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);
a4=A(5);
a5=A(6);

q=a5*t.^5+a4*t.^4+a3*t.^3+a2*t.^2+a1*t+a0;
dq=5*a5*t.^4+4*a4*t.^3+3*a3*t.^2+2*a2*t+a1;
ddq=20*a5*t.^3+12*a4*t.^2+6*a3*t+2*a2;
dddq=60*a5*t.^2+24*a4*t+6*a3;
ddddq=120*a5*t+24*a4;

end