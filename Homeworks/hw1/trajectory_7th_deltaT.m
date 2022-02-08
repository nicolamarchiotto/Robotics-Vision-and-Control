function [q,dq,ddq,dddq,ddddq,dddddq,ddddddq]=trajectory_7th_deltaT(qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf,ti,tf,Ts)

Q=[qi;dqi;ddqi;dddqi;qf;dqf;ddqf;dddqf];

t=ti:Ts:tf;

dt=tf-ti;

X=[ 1,      0,      0,      0,      0,      0,      0,      0;
    0,      1,      0,      0,      0,      0,      0,      0;
    0,      0,      2,      0,      0,      0,      0,      0;
    0,      0,      0,      6,      0,      0,      0,      0;
    1,      dt,     dt^2,   dt^3,   dt^4,   dt^5,   dt^6,   dt^7;
    0,      1,      2*dt,   3*dt^2, 4*dt^3, 5*dt^4, 6*dt^5, 7*dt^6;
    0,      0,      2,      6*dt,   12*dt^2,20*dt^3,30*dt^4,42*dt^5;
    0,      0,      0,      6,      24*dt,60*dt^2,120*dt^3,210*dt^4];

%XA=Q, A=inv(X)Q

A=X\Q;

a0=A(1);
a1=A(2);
a2=A(3);
a3=A(4);
a4=A(5);
a5=A(6);
a6=A(7);
a7=A(8);

q=      a7*(t-ti).^7+        a6*(t-ti).^6+        a5*(t-ti).^5+    a4*(t-ti).^4+    a3*(t-ti).^3+    a2*(t-ti).^2+    a1*(t-ti)+   a0;
dq=     7*a7*(t-ti).^6+      6*a6*(t-ti).^5+      5*a5*(t-ti).^4+  4*a4*(t-ti).^3+  3*a3*(t-ti).^2+  2*a2*(t-ti)+     a1;
ddq=    42*a7*(t-ti).^5+     30*a6*(t-ti).^4+     20*a5*(t-ti).^3+ 12*a4*(t-ti).^2+ 6*a3*(t-ti)+     2*a2;
dddq=   210*a7*(t-ti).^4+    120*a6*(t-ti).^3+    60*a5*(t-ti).^2+ 24*a4*(t-ti)+    6*a3;
ddddq=  840*a7*(t-ti).^3+    360*a6*(t-ti).^2+    120*a5*(t-ti)+   24*a4;
dddddq= 2520*a7*(t-ti).^2+   720*a6*(t-ti)+       120*a5;
ddddddq=5040*a7*(t-ti)+      720*a6;

end
