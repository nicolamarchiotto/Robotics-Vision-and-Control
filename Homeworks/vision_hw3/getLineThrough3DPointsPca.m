function [startPoint, direction]=getLineThrough3DPointsPca(cloudPoints)
if(size(cloudPoints,2)~=3)
    error("Matrix in input must have 3 columns and n = number of points rows");
end

startPoint=[mean(cloudPoints(:,1)),mean(cloudPoints(:,2)),mean(cloudPoints(:,3))]';
[coefficient]=pca(cloudPoints);
direction=coefficient(:,1);
end



