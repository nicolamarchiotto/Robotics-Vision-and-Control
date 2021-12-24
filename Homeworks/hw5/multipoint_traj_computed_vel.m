function [q,dq,ddq,dddq,dqk]=multipoint_traj_computed_vel(qk,tk,Ts,dqi,dqf,showPlots)

n=size(qk,2);

if (n~=size(tk,2))
    error("Positions and times vectors must have the same lenght");
end

T=zeros(1,n-1);
t=tk(1):Ts:tk(n);

for k=1:n-1
    if tk(k)>tk(k+1)
        error("Time at position k+1 must be always greater than time at position k")
    end
    T(k)=tk(k+1)-tk(k);
end


v=[];
dqk=[];

dqk(1)=dqi;
dqk(n)=dqf;

for k=2:size(qk,2)
    v(k)=(qk(k)-qk(k-1))/(tk(k)-tk(k-1));
end

for k=2:size(qk,2)-1
    if (sign(v(k))==sign(v(k+1)))
        dqk(k)=(v(k)+v(k+1))/2;
    else
        dqk(k)=0;
    end
end

a=[];

for k=1:size(qk,2)-1
    
    Tk=tk(k+1)-tk(k);
   
    %ak0
    a(k,1)=qk(k);
    %ak1
    a(k,2)=dqk(k);
    %ak2
    a(k,3)=(3*(qk(k+1)-qk(k))/Tk -2*dqk(k)-dqk(k+1))/Tk;
    %ak3
    a(k,4)=(2*(qk(k)-qk(k+1))/Tk +dqk(k)+dqk(k+1))/Tk^2; 
end

total_time=[];
q=[];
dq=[];
ddq=[];
dddq=[];


for k=1:size(qk,2)-1
    if (k==size(qk,2)-1)
        time=tk(k):Ts:tk(k+1);
    else
        time=tk(k):Ts:tk(k+1)-Ts;
    end
    PI=a(k,4)*(time-tk(k)).^3+a(k,3)*(time-tk(k)).^2+a(k,2)*(time-tk(k))+a(k,1);
    dPI=3*a(k,4)*(time-tk(k)).^2+2*a(k,3)*(time-tk(k))+a(k,2);
    ddPI=6*a(k,4)*(time-tk(k))+2*a(k,3);
    dddPI=6*a(k,4)+ones(1,size(time,2));
    
    %     total_time=[total_time time];
    q=[q PI];
    dq=[dq dPI];
    ddq=[ddq ddPI];
    dddq=[dddq dddPI];
end

if(showPlots)
    figure;
    sgtitle('Multipoint Traj, Computed dq at path points and imposed dqi and dqf');
    subplot(4,1,1);
    plot(t,q)
    hold on
    for i=1:size(qk,2)
        plot(tk(i),qk(i), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Position [rad]')
    xlim([tk(1) tk(n)])
    grid on
    subplot(4,1,2);
    plot(t,dq)
    hold on
    for j=1:size(dqk,2)
        plot(tk(j),dqk(j), '*', 'markerSize', 4, 'color', 'r')
    end
    xlabel('time [sec]')
    ylabel('Velocity [rad/s]')
    xlim([tk(1) tk(n)])
    grid on
    subplot(4,1,3);
    plot(t,ddq)
    hold on
    if(tk(1)==0)
        plot(0,ddq(1), '*', 'markerSize', 4, 'color', 'r')
    else
        plot(tk(1),ddq(1), '*', 'markerSize', 4, 'color', 'r')
    end
    for j=2:size(tk,2)
        plot(tk(j),ddq(tk(j)/Ts), '*', 'markerSize', 4, 'color', 'r')
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
    saveas(gcf,'multipoint_computed_dq_path_point.png')
end




end