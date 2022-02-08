function [distance]=pointToLineDistance(outerPoint, pointLine1, pointLine2)
if(size(outerPoint,1)~=3 && size(outerPoint,2)~=1)
    error("outerPoint to line must be 3*1")
end
if(size(pointLine1,1)~=3 && size(pointLine1,2)~=1)
    error("point 1 of line must be 3*1")
end
if(size(pointLine2,1)~=3 && size(pointLine2,2)~=1)
    error("point 2 of line must be 3*1")
end

distance=norm(cross(pointLine1-pointLine2,outerPoint-pointLine1))/norm(pointLine1-pointLine2);

end