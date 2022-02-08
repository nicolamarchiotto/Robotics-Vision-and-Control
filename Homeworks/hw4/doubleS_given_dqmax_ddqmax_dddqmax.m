function [q,dq,ddq,dddq,DT]=doubleS_given_dqmax_ddqmax_dddqmax(qi,qf,dqi,dqf,ti,Ts,dqmax,ddqmax,dddqmax, showPlots)

dqmin=-dqmax;
ddqmin=-ddqmax;
dddqmin=-dddqmax;

% Check if qf<qi
sigma=sign(qf-qi);

qi=sigma*qi;
qf=sigma*qf;
dqi=sigma*dqi;
dqf=sigma*dqf;

dqmax=((sigma+1)*dqmax/2) +(sigma-1)*dqmin/2;
ddqmax=((sigma+1)*ddqmax/2) +(sigma-1)*ddqmin/2;
dddqmax=((sigma+1)*dddqmax/2) +(sigma-1)*dddqmin/2;

dqmin=((sigma+1)*dqmin/2) +(sigma-1)*dqmax/2;
ddqmin=((sigma+1)*ddqmin/2) +(sigma-1)*ddqmax/2;
dddqmin=((sigma+1)*dddqmin/2) +(sigma-1)*dddqmax/2;


% Feasibility

DQ=qf-qi;
tj=min([sqrt(abs(dqf-dqi)/dddqmax), ddqmax/dddqmax]);


if(tj<(ddqmax/dddqmax))
    if(DQ < (tj*(dqi+dqf)))
        error("\nNot fisible double S trajectory\n")
    end
elseif (tj==(ddqmax/dddqmax))
    if(DQ < ((dqi+dqf)*(tj+abs(dqf-dqi)/dddqmax))/2)
        error("\nNot fisible double S trajectory\n")
    end
else
    error("(tj>(ddqmax/dddqmax)")
end

% Check if dqlim=dqmax

%Acceleration
if(((dqmax-dqi)*dddqmax) < ddqmax^2)
    %ddqmax is not reached
    Tj1=sqrt((dqmax-dqi)/dddqmax);
    ta=2*Tj1;
else
    %ddqmax is reached
    Tj1=ddqmax/dddqmax;
    ta=Tj1+(dqmax-dqi)/ddqmax;
end


%Decelleration
if(((dqmax-dqf)*dddqmax) < ddqmax^2)
    %ddqmin is not reached
    Tj2=sqrt((dqmax-dqf)/dddqmax);
    td=2*Tj2;
else
    %ddqmin is reached
    Tj2=ddqmax/dddqmax;
    td=Tj2+(dqmax-dqf)/ddqmax;
end

%Time duration of costant velocity phase
tv=(qf-qi)/dqmax - (ta/2)*(1+dqi/dqmax)-(td/2)*(1+dqf/dqmax);

if(tv<=0)
    %Velocity dqlim is actually smaller than dqmax, recomputation of ta,td
    tv=0;
    Tj=ddqmax/dddqmax;
    Tj1=Tj;
    Tj2=Tj;
    
    %computation of D
    D=(ddqmax^4)/(dddqmax^2) + 2*(dqi^2+dqf^2)+ddqmax*(4*(qf-qi)-2*(ddqmax/dddqmax)*(dqi+dqf));
    
    ta=((ddqmax^2)/dddqmax - 2*dqi+ sqrt(D))/(2*ddqmax);
    td=((ddqmax^2)/dddqmax - 2*dqf+ sqrt(D))/(2*ddqmax);
    
    if(ta<(2*Tj) || td<(2*Tj))
        error("\nThe maximum/minimum acceleration is not reached in at least one of the two segments, decrease ddqmax\n");
    end
    
    if(ta<0)
        error("\nOnly the decelleration phase is necessary??\n");
    end
    
    if(td<0)
        error("\nOnly the decelleration phase is necessary??\n");
    end
end

fprintf("\nValue of ta: %f",ta);
fprintf("\nValue of tv: %f",tv);
fprintf("\nValue of td: %f",td);
fprintf("\nValue of Tj1: %f",Tj1);
fprintf("\nValue of Tj2: %f",Tj2);
fprintf("\nTotal duration DT: %f\n",ta+tv+td);

%Computation of the max/min acc and max vel of the trajectory
ddqalim=dddqmax*Tj1;
ddqdlim=-dddqmax*Tj2;

dqlim=dqi+ddqalim*(ta-Tj1);





% Trajectory computation

DT=ta+tv+td;
t=ti:Ts:DT+ti;

t_a1=ti:Ts:Tj1+ti-Ts;
t_a2=Tj1+ti:Ts:ti+ta-Tj1-Ts;
t_a3=ti+ta-Tj1:Ts:ti+ta-Ts;

t_c=ti+ta:Ts:ti+ta+tv-Ts;

t_d1=DT-td+ti:Ts:DT-td+Tj2+ti-Ts;
t_d2=DT-td+Tj2+ti:Ts:DT-Tj2-Ts+ti;

% t_d_3=DT-Tj2:Ts:DT-Ts;
s=size(t,2)-size(t_a1,2)-size(t_a2,2)-size(t_a3,2)-size(t_c,2)-size(t_d1,2)-size(t_d2,2);

t_d3=linspace(DT-Tj2-Ts+ti,DT+ti,abs(s));

%Acceleration phase

q_a1=qi+dqi*(t_a1-ti)+dddqmax*((t_a1-ti).^3/6);
dq_a1=dqi+dddqmax*((t_a1-ti).^2)/2;
ddq_a1=dddqmax*(t_a1-ti);
dddq_a1=dddqmax*ones(1,size(t_a1,2));

q_a2=qi+dqi*(t_a2-ti)+(ddqalim/6)*(3*(t_a2-ti).^2-3*Tj1*(t_a2-ti)+Tj1^2);
dq_a2=dqi+ddqalim*(t_a2-ti-Tj1/2);
ddq_a2=ddqalim*ones(1,size(t_a2,2));
dddq_a2=zeros(1,size(t_a2,2));

q_a3=qi+(dqlim+dqi)*(ta/2)-dqlim*(ta-t_a3+ti)-(dddqmin/6)*(ta-t_a3+ti).^3;
dq_a3=dqlim+dddqmin*((ta-t_a3+ti).^2)/2;
ddq_a3=-dddqmin*(ta-t_a3+ti);
dddq_a3=-dddqmax*ones(1,size(t_a3,2));


%Costant velocity phase
q_c=qi+(dqlim+dqi)*(ta/2)+dqlim*(t_c-ti-ta);
dq_c=dqlim*ones(1,size(t_c,2));
ddq_c=zeros(1,size(t_c,2));
dddq_c=zeros(1,size(t_c,2));


%Decelleration phase
q_d1=qf-(dqlim+dqf)*(td/2)+dqlim*(t_d1-DT+td-ti)-(dddqmax/6)*(t_d1-DT+td-ti).^3;
dq_d1=dqlim-dddqmax*((t_d1-DT-ti+td).^2)/2;
ddq_d1=-dddqmax*(t_d1-DT+td-ti);
dddq_d1=-dddqmax*ones(1,size(t_d1,2));


q_d2=qf-(dqlim+dqf)*(td/2)+dqlim*(t_d2-DT+td-ti) +(ddqdlim/6)*(3*((t_d2-DT+td-ti).^2)-3*Tj2*(t_d2-DT+td-ti)+Tj2^2);
dq_d2=dqlim+ddqdlim*(t_d2-DT+td-Tj2/2-ti);
ddq_d2=ddqdlim*ones(1,size(t_d2,2));
dddq_d2=zeros(1,size(t_d2,2));

q_d3=qf-dqf*(DT-t_d3+ti)-(dddqmax/6)*(DT-t_d3+ti).^3;  
dq_d3=dqf+dddqmax*((DT-t_d3+ti).^2)/2;
ddq_d3=-dddqmax*(DT-t_d3+ti);
dddq_d3=dddqmax*ones(1,size(t_d3,2));


q=[q_a1, q_a2, q_a3, q_c, q_d1, q_d2, q_d3];
dq=[dq_a1, dq_a2, dq_a3, dq_c, dq_d1, dq_d2, dq_d3];
ddq=[ddq_a1, ddq_a2, ddq_a3, ddq_c, ddq_d1, ddq_d2, ddq_d3];
dddq=[dddq_a1, dddq_a2, dddq_a3, dddq_c, dddq_d1, dddq_d2, dddq_d3];

q=sigma*q;
dq=sigma*dq;
ddq=sigma*ddq;
dddq=sigma*dddq;


dqlim=sigma*dqlim;

if(sigma==-1)
    sup=-ddqalim;
    ddqalim=-ddqdlim;
    ddqdlim=sup;
end

fprintf("\nValue of ddqalim: %f",ddqalim);
fprintf("\nValue of ddqdlim: %f",ddqdlim);
fprintf("\nValue of dqlim: %f\n",dqlim);

if(showPlots)
    figure;
    sgtitle(['Double S trajectory dqmax=',num2str(dqmax),' ddqmax= ',num2str(ddqmax),' dddqmax= ',num2str(dddqmax)]);
    subplot(4,1,1);
    plot(t,q)
    xlabel('time [sec]')
    ylabel('Position [rad]')
    xlim([ti DT+ti])
    grid on
    subplot(4,1,2);
    plot(t,dq)
    xlabel('time [sec]')
    ylabel('Velocity [rad/s]')
    xlim([ti DT+ti])
    grid on
    subplot(4,1,3);
    plot(t,ddq)
    xlabel('time [sec]')
    ylabel('Acceleration [rad/s^2]')
    xlim([ti DT+ti])
    grid on
    subplot(4,1,4);
    plot(t,dddq)
    xlabel('time [sec]')
    ylabel('Jerk [rad/s^3]')
    xlim([ti DT+ti])
    grid on
    saveas(gcf,'doubleS_given_dqmax_ddqmax_dddqmax.png')
end
end