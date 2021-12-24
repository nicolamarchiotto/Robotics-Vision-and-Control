function [distance]=point2planeDistance(planeParams,point)

if(size(planeParams,1)~=1 && size(planeParams,2)~=4)
    error("Vector of plane parameters is not 1*4");
end

if(size(point,3)~=1 && size(point,2)~=1)
    error("Vector of plane parameters is not 3*1");
end

a=planeParams(1);
b=planeParams(2);
c=planeParams(3);
d=planeParams(4);

distance=( a*point(1)+b*point(2)+c*point(3)+d )/sqrt(a^2+b^2+c^2);

end