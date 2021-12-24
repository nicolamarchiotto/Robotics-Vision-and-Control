function [q,dq,ddq,dddq,ddddq,dddddq,ddddddq]=trajectory_7th_ti_tf(qi,qf,dqi,dqf,ddqi,ddqf,dddqi,dddqf,ti,tf,Ts)

Q=[qi;dqi;ddqi;dddqi;qf;dqf;ddqf;dddqf];

t=ti:Ts:tf;

X=[ 1,      ti,     ti^2,       ti^3,       ti^4,       ti^5,       ti^6,       ti^7;
    0,      1,      2*ti,       3*ti^2,     4*ti^3,     5*ti^4,     6*ti^5,     7*ti^6;
    0,      0,      2,          6*ti,       12*ti^2,    20*ti^3,    30*ti^4,    42*ti^5;
    0,      0,      0,          6,          24*ti,      60*ti^2,    120*ti^3,   210*ti^4;
    1,      tf,     tf^2,       tf^3,       tf^4,       tf^5,       tf^6,       tf^7;
    0,      1,      2*tf,       3*tf^2,     4*tf^3,     5*tf^4,     6*tf^5,     7*tf^6;
    0,      0,      2,          6*tf,       12*tf^2,    20*tf^3,    30*tf^4,    42*tf^5;
    0,      0,      0,          6,          24*tf,      60*tf^2,    120*tf^3,   210*tf^4];

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

q=      a7*t.^7+        a6*t.^6+        a5*t.^5+    a4*t.^4+    a3*t.^3+    a2*t.^2+    a1*t+   a0;
dq=     7*a7*t.^6+      6*a6*t.^5+      5*a5*t.^4+  4*a4*t.^3+  3*a3*t.^2+  2*a2*t+     a1;
ddq=    42*a7*t.^5+     30*a6*t.^4+     20*a5*t.^3+ 12*a4*t.^2+ 6*a3*t+     2*a2;
dddq=   210*a7*t.^4+    120*a6*t.^3+    60*a5*t.^2+ 24*a4*t+    6*a3;
ddddq=  840*a7*t.^3+    360*a6*t.^2+    120*a5*t+   24*a4;
dddddq= 2520*a7*t.^2+   720*a6*t+       120*a5;
ddddddq=5040*a7*t+      720*a6;

end