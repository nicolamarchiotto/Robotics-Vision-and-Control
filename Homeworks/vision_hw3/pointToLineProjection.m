function [projection]=pointToLineProjection(pointToProject, pointLine1,pointLine2)

if(size(pointToProject,1)~=3 && size(pointToProject,2)~=1)
    error("All points given as input must have dimension 3*1")
end
if(size(pointLine1,1)~=3 && size(pointLine1,2)~=1)
    error("All points given as input must have dimension 3*1")
end
if(size(pointLine2,1)~=3 && size(pointLine2,2)~=1)
    error("All points given as input must have dimension 3*1")
end

A=pointLine1;
AP=pointToProject-pointLine1;
AB=pointLine2-pointLine1;

projection=A + dot(AP,AB) / dot(AB,AB) * AB;
end