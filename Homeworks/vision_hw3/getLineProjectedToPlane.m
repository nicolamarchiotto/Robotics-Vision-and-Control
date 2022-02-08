function [startPoint, direction]=getLineProjectedToPlane(cloudPoints, planeParameters, norm2Plane)
    if(size(cloudPoints,2)~=3)
        error("Matrix in input must have 3 columns and n = number of points rows");
    end
   
    projectedPoints=[];
    
    for i=1:size(cloudPoints,1)
        distance=point2planeDistance(planeParameters, cloudPoints(i,:));

        u=abs(distance);
        
        if(distance>0)
            planeIntersect=cloudPoints(i,:)'-norm2Plane.*[u;u;u];
        else    
            planeIntersect=cloudPoints(i,:)'+norm2Plane.*[u;u;u];
        end
        projectedPoints(end+1,:)=planeIntersect;
    end
    
    startPoint=[mean(projectedPoints(:,1)),mean(projectedPoints(:,2)),mean(projectedPoints(:,3))]';
    [coefficient]=pca(projectedPoints);
    direction=coefficient(:,1);
end
