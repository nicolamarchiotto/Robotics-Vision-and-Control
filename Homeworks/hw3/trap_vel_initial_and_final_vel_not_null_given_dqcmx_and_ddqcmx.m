function [q,dq,ddq,tf]=trap_vel_initial_and_final_vel_not_null_given_dqcmx_and_ddqcmx(qi,qf,dqi,dqf,Ts,dqcmax,ddqcmax,showPlots)
sigma=sign(qf-qi);

qi=sigma*qi;
qf=sigma*qf;
dqi=sigma*dqi;
dqf=sigma*dqf;

DQ=qf-qi;

if ddqcmax*DQ<((abs(dqi^2-dqf^2))/2)
    error('Not fisible trapeizoidal velocity profile for the given costraints') 
end

Delta=ddqcmax*DQ+(dqi^2+dqf^2)/2;
dqclim=sqrt(Delta);

if ddqcmax*DQ < dqcmax^2-(dqi^2+dqf^2)/2
    
   
    if Delta<0
        error('Delta<0, increase ddqcmax not fisible trapeizoidal velocity profile for the given costraints') 
    end
    
    
    if dqclim>dqcmax
         error('dqclim>dqcmax, not fisible trapeizoidal velocity profile for the given costraints') 
    end
    
    dqc=dqclim;
    ta=(dqclim-dqi)/ddqcmax;
    td=(dqclim-dqf)/ddqcmax;
    
    if ta<0
        error('ta<0, not fisible trapeizoidal velocity profile for the given costraints') 
    end
    if td<0
        error('td<0, not fisible trapeizoidal velocity profile for the given costraints') 
    end
    ti=0;
    tf=ta+td;
else
    dqc=dqcmax;
    
    ta=(dqcmax-dqi)/ddqcmax;
    td=(dqcmax-dqf)/ddqcmax;
    
    if ta<0
        error('ta<0, not fisible trapeizoidal velocity profile for the given costraints') 
    end
    if td<0
        error('td<0, not fisible trapeizoidal velocity profile for the given costraints') 
    end
    
    tf=DQ/dqcmax + dqcmax*(1-dqi/dqcmax)^2/(2*ddqcmax) + dqcmax*(1-dqf/dqcmax)^2/(2*ddqcmax);
    ti=0;
end

fprintf('trap_vel_initial_and_final_vel_not_null_given_dqcmx_and_ddqcmx\n');
fprintf('ta: %d\n',ta);
fprintf('td: %d\n',td);
fprintf('Velocity dqc: %d\n',dqc);
fprintf('Velocity dqclim: %d\n',dqclim);
fprintf('Velocity dqcmax: %d\n',dqcmax);
fprintf('Acceleration ddqcmax: %d\n',ddqcmax);

t=ti:Ts:tf;

t_p1=ti:Ts:ti+ta-Ts;
t_p2=ti+ta:Ts:tf-td-Ts;

s=size(t,2)-size(t_p1,2)-size(t_p2,2);
t_p3=linspace(tf-td,tf,s);


q_p1=qi+dqi*(t_p1-ti)+((dqc-dqi)/(2*ta))*(t_p1-ti).^2;
q_p2=qi+dqi*ta/2+dqc*(t_p2-ti-ta/2);
q_p3=qf-dqf*(tf-t_p3)-((dqc-dqf)/(2*td))*(tf-t_p3).^2;

q=[q_p1, q_p2, q_p3];

dq_p1=dqi+(dqc-dqi)*(t_p1-ti)/ta;
dq_p2=ones(1,size(t_p2,2))*dqc;
dq_p3=dqf+(dqc-dqf)*(tf-t_p3)/td;

dq=[dq_p1, dq_p2, dq_p3];

ddq_p1=ones(1,size(t_p1,2))*((dqc-dqi)/ta);
ddq_p2=zeros(1,size(t_p2,2));
ddq_p3=-ones(1,size(t_p3,2))*((dqc-dqf)/td);

ddq=[ddq_p1, ddq_p2, ddq_p3];

q=sigma*q;
dq=sigma*dq;
ddq=sigma*ddq;

t=ti:Ts:tf;

% Visualization
if(showPlots)
    figure;
    sgtitle('Trap Vel Profile, initital & final vel not null, dqcmax & ddqcmax given')
    subplot(3,1,1)
    plot(t,q)
    xlim([ti tf])
    xlabel('time [sec]')
    ylabel('Position [rad]')
    grid on
    subplot(3,1,2)
    plot(t,dq)
    xlim([ti tf])
    xlabel('time [sec]')
    ylabel('Velocity [rad/s]')
    grid on
    subplot(3,1,3)
    grid on
    plot(t,ddq)
    xlim([ti tf])
    xlabel('time [sec]')
    ylabel('Acceleration [rad/s^2]')
    grid on
    saveas(gcf,'trap_vel_dqi_dqf_not_null_given_dqcmx_ddqcmx.png')

end

end