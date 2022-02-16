function [total_position,total_velocity,total_acceleration,total_jerk,ddqk]=multipoint_comp_based_on_acc(qk,tk,Ts,dqi,dqf,show_plot)

n=size(qk,2);
t=tk(1):Ts:tk(n);


if (n~=size(qk,2))
    error("Positions and times vectors must have the same lenght");
end

T=zeros(1,n-1);

for k=1:n-1
    if tk(k)>tk(k+1)
        error("Time at position k+1 must be always greater than time at position k")
    end
    T(k)=tk(k+1)-tk(k);
end


A=zeros(n);
C=zeros(1,n);
ddqk=zeros(1,n);

A(1,1)=2*T(1);
A(1,2)=T(1);

A(n,n-1)=T(n-1);
A(n,n)=2*T(n-1);

C(1)=6*((qk(2)-qk(1))/T(1) - dqi);
C(n)=6*(dqf - (qk(n)-qk(n-1))/T(n-1));

for k=2:n-1
    A(k,k-1)=T(k-1);
    A(k,k)=2*T(k-1)+2*T(k);
    A(k,k+1)=T(k);
    
    C(k)=6*((qk(k+1)-qk(k))/T(k) - (qk(k)-qk(k-1))/T(k-1));
end

%Thomas

for k=2:size(A,1)
    m=A(k,k-1)/A(k-1,k-1);
    A(k,k)=A(k,k)-m*A(k-1,k);
    C(k)=C(k)-m*C(k-1);
end

ddqk(size(A,1))=C(size(A,1))/A(size(A,1),size(A,1));
for k=size(A,1)-1:-1:1
    ddqk(k)=(C(k)-A(k,k+1)*ddqk(k+1))/A(k,k);
end

a=[];

for k=1:size(qk,2)-1
    
    %ak0
    a(k,1)=qk(k);
    %ak1
    a(k,2)=(qk(k+1)-qk(k))/T(k) - T(k)*(ddqk(k+1)+2*ddqk(k))/6;
    %ak2
    a(k,3)=ddqk(k)/2;
    %ak3
    a(k,4)=(ddqk(k+1)-ddqk(k))/(6*T(k));
end

total_time=[];
total_position=[];
total_velocity=[];
total_acceleration=[];
total_jerk=[];


for k=1:size(qk,2)-1
    if(k==size(qk,2)-1)
        time=tk(k):Ts:tk(k+1);
    else
        time=tk(k):Ts:tk(k+1)-Ts;
    end

    PI=a(k,4)*(time-tk(k)).^3+a(k,3)*(time-tk(k)).^2+a(k,2)*(time-tk(k))+a(k,1);
    dPI=3*a(k,4)*(time-tk(k)).^2+2*a(k,3)*(time-tk(k))+a(k,2);
    ddPI=6*a(k,4)*(time-tk(k))+2*a(k,3);
    dddPI=6*a(k,4)+ones(1,size(time,2));
    
    total_time=[total_time time];
    total_position=[total_position PI];
    total_velocity=[total_velocity dPI];
    total_acceleration=[total_acceleration ddPI];
    total_jerk=[total_jerk dddPI];
    
end

q=total_position;
dq=total_velocity;
ddq=total_acceleration;
dddq=total_jerk;

if(show_plot)
    figure;
    sgtitle('Multipoint Traj, Comp based on Acc, imposed dqi and dqf');
    subplot(4,1,1);
    plot(t,q)
    hold on
    for j=1:size(qk,2)
        plot(tk(j),qk(j), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Position [rad]')
    xlim([tk(1) tk(n)])
    grid on
    subplot(4,1,2);
    plot(t,dq)
    hold on
    xlabel('time [sec]')
    ylabel('Velocity [rad/s]')
    xlim([tk(1) tk(n)])
    grid on
    subplot(4,1,3);
    plot(t,ddq)
    hold on
    for j=1:size(ddqk,2)
        plot(tk(j),ddqk(j), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Acceleration [rad/s^2]')
    xlim([tk(1) tk(n)])
    grid on    
    subplot(4,1,4);
    plot(t,dddq)
    xlabel('time [sec]')
    ylabel('Jerk [rad/s^3]')
    xlim([tk(1) tk(n)])
    grid on
    saveas(gcf,'multipoint_based_on_acc.png')
end

end