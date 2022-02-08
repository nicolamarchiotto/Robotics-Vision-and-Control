DimValues=6;

[trajPosition, trajVelocity, timeSerie]=homework9_main(false);

DataPositions=[];
DataVelocities=[];

for i=1:DimValues
    DataPositions(i,:)=trajPosition(i,:);
    DataVelocities(i,:)=trajVelocity(i,:);
end

xdSim.time=timeSerie; 
xdSim.signals.values=DataPositions';
xdSim.signals.dimensions=DimValues;

dotxdSim.time=timeSerie;
dotxdSim.signals.values=DataVelocities';
dotxdSim.signals.dimensions=DimValues;
