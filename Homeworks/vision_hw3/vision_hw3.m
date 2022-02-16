% 
% 
% 
% THE FIRST SECTION IS RELATED TO THE REULT OF THE ASSIGNMENT, THE OTHERS ARE FOR
% TESTING OF ALL THE FUNCTIONS REQUIRED IN THE HOMEWORK
% 
% 
% 

clc;
clear;
close all;

points=getCloudPoints(false);
ptCloud=pointCloud(points);

% Fit plane to a 3-D point cloud.
[model] = pcfitplane(ptCloud,10);

a=model.Parameters(1);
b=model.Parameters(2);
c=model.Parameters(3);
d=model.Parameters(4);

norm2Plane=model.Normal';
norm2Plane=norm2Plane/norm(norm2Plane);

[x, y] = meshgrid(-200:1:100); % Generate x and y data
z = -1/c*(a*x + b*y + d); % Solve for z data


% GET CONTOUR POINTS
[contourPoints,centroid]=getContourCloudPoints(true);

figure
%plot cloud points
plot3(points(:,1), points(:,2), points(:,3), 'r.');
hold on
plot3(centroid(1), centroid(2), centroid(3),'*','Color', 'w','MarkerSize',10);
% surf(x,y,z) %Plot the surface
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
grid on
title("Plane params: a:"+a+" b:"+b+" c:"+c+" d:"+d)


% PLOT CONTOUR OF THE PLANE
 plot3(contourPoints(:,1), contourPoints(:,2), contourPoints(:,3), 'b.');
 
 
% RANSAC LINE FITTING

t=-250:1:250;

% lineFittingRansac(data, minNumInliers, numIteration, tresh)
[startPoint1, direction1, inliers, outliers]=lineFittingRansac(contourPoints, 50, 150, 5);
ransacLine1=startPoint1+t.*direction1;

[startPoint2, direction2, inliers, outliers]=lineFittingRansac(outliers, 50, 150, 5);
ransacLine2=startPoint2+t.*direction2;

[startPoint3, direction3, inliers, outliers]=lineFittingRansac(outliers, 50, 150, 5);
ransacLine3=startPoint3+t.*direction3;

[startPoint4, direction4, inliers, outliers]=lineFittingRansac(outliers, 50, 150, 5);
ransacLine4=startPoint4+t.*direction4;


plot3(ransacLine1(1,:),ransacLine1(2,:),ransacLine1(3,:),'.','Color','g');
plot3(ransacLine2(1,:),ransacLine2(2,:),ransacLine2(3,:),'.','Color','c');
plot3(ransacLine3(1,:),ransacLine3(2,:),ransacLine3(3,:),'.','Color','m');
plot3(ransacLine4(1,:),ransacLine4(2,:),ransacLine4(3,:),'.','Color','y');

%RANSAC LINE INTERSECTION
% compute point of line 1 which is closer to line 2 not parallel, then
% projected this point to line 2, done because these line may be not lying
% on the same plane

matrixOfLines=[ransacLine1;ransacLine2;ransacLine3;ransacLine4];
matrixOfDirection=[direction1,direction2,direction3,direction4];

for i=1:4
    for j=1:4
        if(j>i)
            angle=angleBetweenLines(matrixOfDirection(:,j),matrixOfDirection(:,i));
            if(angle>70 && angle<160)
                distVec=[];
                line1=matrixOfLines(i*3-2:i*3,:);
                line2=matrixOfLines(j*3-2:j*3,:);
                for index=1:size(matrixOfLines,2)
                    distVec(index)=pointToLineDistance(line1(:,index),line2(:,1),line2(:,end));
                end
                [M,I] = min(distVec);
                projection=pointToLineProjection(line1(:,I(1)), line2(:,1),line2(:,end));
                plot3(projection(1),projection(2),projection(3),'.','Color','k','MarkerSize',35);
            end

        end
    end
end
saveas(gcf,'3d_analysis_1.png')

        
%% OTHER CODE NOT RELEVANT TO THE ASSIGNEMENT
% POINT TO PLANE DISTACE

u=0:30;
startPointIndex=floor(size(x,2)/2);
startPoint4=[x(startPointIndex,startPointIndex),y(startPointIndex,startPointIndex),z(startPointIndex,startPointIndex)];
rayNormal=startPoint4'+norm2Plane.*[u;u;u];

pointIndex=floor(size(x,2)/4);
outerPoint=[x(pointIndex,pointIndex)+50,y(pointIndex,pointIndex)-10,z(pointIndex,pointIndex)+100]';
distance=point2planeDistance(model.Parameters, outerPoint); 


u=0:0.1:abs(distance)-0.1;
if(distance>0)
    rayProjection=outerPoint-norm2Plane.*[u;u;u];
else    
    rayProjection=outerPoint+norm2Plane.*[u;u;u];
end
intersectionPoint=rayProjection(:,end);

%PLOT OUTER POINT, ITS PROJECTION AND THE PERPENDICULAR ARRAY

plot3(outerPoint(1),outerPoint(2),outerPoint(3),'*','Color','b')
plot3(rayProjection(1,:),rayProjection(2,:),rayProjection(3,:),'.','Color','g')
plot3(intersectionPoint(1),intersectionPoint(2),intersectionPoint(3),'*','Color','b')


%% COMPUTATION OF CONTOUR LINES USING PCA; THEY MAY BE SKEW

line1Points=[];
line2Points=[];
line3Points=[];
line4Points=[];

for i=1:size(contourPoints,1)
    if(contourPoints(i,1)<-54 && contourPoints(i,1)>-85 && contourPoints(i,2)<15 && contourPoints(i,2)>-175)
         line1Points(end+1,:) = contourPoints(i,:);
    elseif(contourPoints(i,1)<10 && contourPoints(i,1)>-83 && contourPoints(i,2)<45 && contourPoints(i,2)>16)
         line2Points(end+1,:) = contourPoints(i,:);
    elseif(contourPoints(i,1)<44 && contourPoints(i,1)>6 && contourPoints(i,2)<40 && contourPoints(i,2)>-160)
         line3Points(end+1,:) = contourPoints(i,:);
    elseif(contourPoints(i,1)<44 && contourPoints(i,1)>-54 && contourPoints(i,2)<-160 && contourPoints(i,2)>-180)
         line4Points(end+1,:) = contourPoints(i,:);
    end
end

% plot3(line1Points(:,1), line1Points(:,2), line1Points(:,3), 'b.');
% plot3(line2Points(:,1), line2Points(:,2), line2Points(:,3), 'y.');
% plot3(line3Points(:,1), line3Points(:,2), line3Points(:,3), 'g.');
% plot3(line4Points(:,1), line4Points(:,2), line4Points(:,3), 'm.');


t=-150:1:150;

[startPoint4, direction4]=getLineThrough3DPointsPca(line1Points);
line1=startPoint4+t.*direction4(:,1);

[startPoint4, direction4]=getLineThrough3DPointsPca(line2Points);
line2=startPoint4+t.*direction4(:,1);

[startPoint4, direction4]=getLineThrough3DPointsPca(line3Points);
line3=startPoint4+t.*direction4(:,1);

[startPoint4, direction4]=getLineThrough3DPointsPca(line4Points);
line4=startPoint4+t.*direction4(:,1);


% plot3(line1(1,:),line1(2,:),line1(3,:),'.','Color','g');
% plot3(line2(1,:),line2(2,:),line2(3,:),'.','Color','c');
% plot3(line3(1,:),line3(2,:),line3(3,:),'.','Color','m');
% plot3(line4(1,:),line4(2,:),line4(3,:),'.','Color','y');

%% CONTOUR POINTS PROJECTED TO PLANE AND THEN LINE INTERPOLATION

t=-150:1:150;
[startPoint1, direction1]=getLineProjectedToPlane(line1Points,model.Parameters,norm2Plane);
projectedLine1=startPoint1+t.*direction1(:,1);

[startPoint2, direction2]=getLineProjectedToPlane(line2Points,model.Parameters,norm2Plane);
projectedLine2=startPoint2+t.*direction2(:,1);

[startPoint3, direction3]=getLineProjectedToPlane(line3Points,model.Parameters,norm2Plane);
projectedLine3=startPoint3+t.*direction3(:,1);

[startPoint4, direction4]=getLineProjectedToPlane(line4Points,model.Parameters,norm2Plane);
projectedLine4=startPoint4+t.*direction4(:,1);

plot3(projectedLine1(1,:),projectedLine1(2,:),projectedLine1(3,:),'.','Color','g');
plot3(projectedLine2(1,:),projectedLine2(2,:),projectedLine2(3,:),'.','Color','c');
plot3(projectedLine3(1,:),projectedLine3(2,:),projectedLine3(3,:),'.','Color','m');
plot3(projectedLine4(1,:),projectedLine4(2,:),projectedLine4(3,:),'.','Color','y');

% INTERSECTION OF PROJECTED LINE

[pointIntersection1,angle1]=coplanarLineIntersection(startPoint1,direction1,startPoint2,direction2);
plot3(pointIntersection1(1),pointIntersection1(2),pointIntersection1(3),'.','Color','b','MarkerSize',25);

[pointIntersection2,angle2]=coplanarLineIntersection(startPoint2,direction2,startPoint3,direction3);
plot3(pointIntersection2(1),pointIntersection2(2),pointIntersection2(3),'.','Color','b','MarkerSize',25);

[pointIntersection3,angle3]=coplanarLineIntersection(startPoint3,direction3,startPoint4,direction4);
plot3(pointIntersection3(1),pointIntersection3(2),pointIntersection3(3),'.','Color','b','MarkerSize',25);

[pointIntersection4,angle4]=coplanarLineIntersection(startPoint4,direction4,startPoint1,direction1);
plot3(pointIntersection4(1),pointIntersection4(2),pointIntersection4(3),'.','Color','b','MarkerSize',25);



