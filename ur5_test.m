q0 = [0.2 -0.5 0.3 -2 2.5 3]';
qf = [-0.2 -1.9 0.3 1 1.5 -1.2]';
ti = 0;
tf = 10;
Ts = 0.01;
 

DT=tf-ti;


alpha=1/3;
beta=1/3;
%important qd = desired position, qdq desidered velocity
% TimeValues=ti:Ts:tf;
TimeValues=ti:Ts:tf;
DimValues=6;

%Matrix 6*numberOfSamples from ti to tf
DataPositions=[];
DataVelocities=[];

for i=1:DimValues 
%     [q,dq,ddq,dddq]=harmonicTrajectory(q0(i),qf(i),ti,tf,Ts);
    [q,dq,ddq,dddq] = doubleS_preassigned_duration(q0(i),qf(i),DT,0,Ts,alpha,beta);
    DataPositions(i,:)=q;
    DataVelocities(i,:)=dq;
end

qd.time=TimeValues;
qd.signals.values=DataPositions';
qd.signals.dimensions=DimValues;

dotqd.time=TimeValues;
dotqd.signals.values=DataVelocities';
dotqd.signals.dimensions=DimValues;
