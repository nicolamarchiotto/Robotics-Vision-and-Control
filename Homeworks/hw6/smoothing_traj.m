% attention the w_vector is of the form [1/w1,...,1/wn]
function [q,dq,ddq,dddq,ddqk,s,ds,dds,ddds,ddsk]=smoothing_traj(qk,tk,Ts,mu,w_vector,show_plot)

n=size(tk,2);

if (n~=size(qk,2))
    error("Positions and times vectors must have the same lenght");
end
if (n~=size(w_vector,2))
    error("Weight vector must have the same lenght of the positions and times vectors");
end

t=tk(1):Ts:tk(n);


W_inv=diag(w_vector);

%function

T=zeros(1,n-1);

for k=1:n-1
    if tk(k)>tk(k+1)
        error("Time at position k+1 must be always greater than time at position k")
    end
    T(k)=tk(k+1)-tk(k);
end

lamba=(1-mu)/(6*mu);

sA=zeros(n);
sC=zeros(n);

sA(1,1)=2*T(1);
sA(1,2)=T(1);
sA(n,n-1)=T(n-1);
sA(n,n)=2*T(n-1);

sC(1,1)=-6/T(1);
sC(1,2)=6/T(1);
sC(n,n-1)=6/T(n-1);
sC(n,n)=-6/T(n-1);

for k=2:n-1
    sA(k,k-1)=T(k-1);
    sA(k,k)=2*T(k-1)+2*T(k);
    sA(k,k+1)=T(k);
    
    sC(k,k-1)=6/T(k-1);
    sC(k,k)=-(6/T(k-1)+6/T(k));
    sC(k,k+1)=6/T(k);
end

%computation of the best sk

ddsk=(sA+lamba.*sC*W_inv*sC')\sC*qk';

sk=qk'-lamba.*W_inv*sC'*ddsk;
sk=sk';

[s,ds,dds,ddds,ddsk]=multipoint_comp_based_on_acc(sk,tk,Ts,0,0,false);
[q,dq,ddq,dddq,ddqk]=multipoint_comp_based_on_acc(qk,tk,Ts,0,0,false);

if(show_plot)
    figure;
    sgtitle('Smoothed Trajectory based on cubic splines');
    subplot(4,1,1);
    plot(t,q,'b',t,s,'r')
    hold on
    for j=1:size(qk,2)
        plot(tk(j),qk(j), '*', 'markerSize', 4, 'color', 'b')
    end
    for j=1:size(qk,2)
        plot(tk(j),sk(j), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Position [rad]')
    xlim([tk(1) tk(n)])
    grid on
    
    subplot(4,1,2);
    plot(t,dq,'b',t,ds,'r')
    xlabel('time [sec]')
    ylabel('Velocity [rad/s]')
    xlim([tk(1) tk(n)])
    grid on
    
    subplot(4,1,3);
    plot(t,ddq,'b',t,dds,'r')
    hold on
    for j=1:size(qk,2)
        plot(tk(j),ddqk(j), '*', 'markerSize', 4, 'color', 'b')
    end
    for j=1:size(qk,2)
        plot(tk(j),ddsk(j), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Acceleration [rad/s^2]')
    xlim([tk(1) tk(n)])
    grid on
    
    subplot(4,1,4);
    plot(t,dddq,'b',t,ddds,'r')
    xlabel('time [sec]')
    ylabel('Jerk [rad/s^3]')
    xlim([tk(1) tk(n)])
    grid on
    saveas(gcf,'smoothed_trajectory.png')
end

end
