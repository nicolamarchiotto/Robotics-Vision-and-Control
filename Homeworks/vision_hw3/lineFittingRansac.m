function [startPointOfBestModel, directionOfBestModel, inliersOfBestModel, outliersOfBestModel]=lineFittingRansac(data, minNumInliers, numIteration, tresh)
if(size(data,2)~=3)
    error("Matrix in input must have 3 columns and n = number of points rows");
end

iteration=0;
startPointOfBestModel=[];
directionOfBestModel=[];
inliersOfBestModel=[];
outliersOfBestModel=[];

numberOfData=size(data,1);

while iteration < numIteration
    
    startPoint=[];
    direction=[];
    inliers=[];
    outliers=[];
    
    indeces = floor((numberOfData-1).*rand(2,1) + 1);
    
    point1=data(indeces(1),:);
    point2=data(indeces(2),:);
    
    possibleOutliers = data;
    possibleOutliers(indeces(1),:)=[];
    possibleOutliers(indeces(2),:)=[];
    
    %model computation, start point and direction
    startPoint=point1;
    direction=(point2-point1)/(norm(point2-point1));
    inliers(end+1,:)=point1;
    inliers(end+1,:)=point2;
    
    for i=1:1:size(possibleOutliers,1)
        if(abs(pointToLineDistance(possibleOutliers(i,:)',point1',point2'))<tresh)
            inliers(end+1,:)=possibleOutliers(i,:);
        else
            outliers(end+1,:)=possibleOutliers(i,:);
        end
    end
    if(size(inliers,1)>minNumInliers)
        if(size(inliers,1)>size(inliersOfBestModel,1))
           inliersOfBestModel=inliers;
           outliersOfBestModel=outliers;
           startPointOfBestModel=startPoint';
           directionOfBestModel=direction';
        end
    end
        
    iteration=iteration+1;
end
end
