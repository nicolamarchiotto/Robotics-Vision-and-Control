function [intersectionPoint,angle]=coplanarLineIntersection(startPoint1,direction1,startPoint2,direction2)

A=[direction1(1),direction2(1);direction1(2),direction2(2)];
B=[startPoint2(1)-startPoint1(1);startPoint2(2)-startPoint1(2)];
X=inv(A)*B;
intersectionPoint=startPoint1+X(1).*direction1(:,1);

angle=acosd(dot(direction1,direction2)/(norm(direction1)*norm(direction2)));

end