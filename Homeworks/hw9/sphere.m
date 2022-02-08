function [xSphere, ySphere, zSphere, params]=sphere(centre, radious, sampling)

if(radious<0)
    error("radious can't be less than zero, radious: %s", radious)
end

a=-2*centre(1);
b=-2*centre(2);
c=-2*centre(3);
d=centre(1)^2+centre(2)^2+centre(3)^2-radious^2;
    
params=[a,b,c,d];

xSphere=[];
ySphere=[];
zSphere=[];
for theta=0:0.1:pi
    for fi=0:0.1:2*pi
        xSphere=[xSphere centre(1)+radious*sin(theta)*cos(fi)];
        ySphere=[ySphere centre(2)+radious*sin(theta)*sin(fi)];
        zSphere=[zSphere centre(3)+radious*cos(theta)];
    end
end
    
end