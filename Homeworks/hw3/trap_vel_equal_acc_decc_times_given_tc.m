function [q,dq,ddq]=trap_vel_equal_acc_decc_times_given_tc(qi,qf,ti,tf,Ts,tc,showPlots)

sigma=sign(qf-qi);

qi=sigma*qi;
qf=sigma*qf;

if tc>((tf-ti)/2)
    fprintf('value of tc: %d\n',tc)
    error('Not fisible profile, tc too big, must be tc<=(tf-ti)/2');
end

ddqc=(qf-qi)/(tc*(tf-ti)-tc^2);
dqc=ddqc*tc;
fprintf('trap_vel_equal_acc_decc_times_given_tc\n');
fprintf('tc: %d\n',tc);
fprintf('Velocity dqc: %d\n',dqc);
fprintf('Acceleration ddqc: %d\n',ddqc);
  
%times
t=ti:Ts:tf;

t_p1=ti:Ts:ti+tc-Ts;
t_p2=ti+tc:Ts:tf-tc-Ts;

s=size(t,2)-size(t_p1,2)-size(t_p2,2);
t_p3=linspace(tf-tc,tf,s);

%position
q_p1=qi+(dqc/(2*tc))*(t_p1-ti).^2;
q_p2=qi+dqc*(t_p2-ti-tc/2);
q_p3=qf-(dqc/(2*tc))*(tf-t_p3).^2;

q=[q_p1,q_p2,q_p3];

%velocity
dq_p1=(dqc/tc)*(t_p1-ti);
dq_p2=ones(1,size(t_p2,2))*dqc;
dq_p3=(dqc/tc)*(tf-t_p3);

dq=[dq_p1,dq_p2,dq_p3];

%acceleration
ddq_p1=ones(1,size(t_p1,2))*(dqc/tc);
ddq_p2=zeros(1,size(t_p2,2));
ddq_p3=-ones(1,size(t_p3,2))*(dqc/tc);

ddq=[ddq_p1,ddq_p2,ddq_p3];

q=sigma*q;
dq=sigma*dq;
ddq=sigma*ddq;

t=ti:Ts:tf;

% Visualization
if(showPlots)
    figure;
    sgtitle('Trap Vel Profile, equal acc and decel times, tc given')
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
    saveas(gcf,'trap_vel_equal_acc_decc_given_tc.png')

end
end