function [pos,vel,acc,jerk]=circular_path_operational(p1,p2,p0,sampling,orientationIfAngleIsPi)
v1=p1-p0;
v2=p2-p0;
radious=norm(p1-p0);

if(abs(norm(p1-p0)-norm(p2-p0))>0.01)
    error('Points not belonging to the same circle')
end

%axis of the circle centered in sphereCentre
circleAxis=cross(v2,v1);
angle=acos(dot(v1 / norm(v1), v2 / norm(v2)));
if(circleAxis==0)
    if(v1(1)~=0)
        v2=[v1(2);v1(1);v1(3)];
    elseif (v1(2)~=0)
        v2=[v1(1);-v1(3);v1(2)];
    elseif (v1(3)~=0)
        v2=[v1(1);v1(3);v1(2)];
    end
    circleAxis=orientationIfAngleIsPi*cross(v2,v1);
    
end


arcLenght=radious*angle;
zAxCirc=circleAxis/norm(circleAxis);
xAxCirc=(p1-p0)/norm(p1-p0);
yAxCirc=cross(xAxCirc,zAxCirc);

R=[xAxCirc,yAxCirc,zAxCirc];
s=0:sampling:arcLenght;
% posSup=p0+R*[radious*cos(s/radious);radious*sin(s/radious);zeros(1,size(s,2))];
% velSup=R*[-ds.*sin(s/radious);ds.*cos(s/radious);zeros(1,size(s,2))];
% accSup=R*[-ds.^2.*cos(s/radious)/radious-dds.*sin(s/radious);-ds.^2.*sin(s/radious)/radious+dds.*cos(s/radious);zeros(1,size(s,2))];

pos=p0+R*[radious*cos(s/radious);radious*sin(s/radious);zeros(1,size(s,2))];
vel=R*[-sin(s/radious);cos(s/radious);zeros(1,size(s,2))];
acc=R*[-cos(s/radious)/radious;-sin(s/radious)/radious;zeros(1,size(s,2))];
jerk=R*[sin(s/radious)/(radious)^2;-radious*cos(s/radious)/(radious)^2;zeros(1,size(s,2))];

end