%%hw 1.2: Create a 3D cloud of points from a range image

close all;
clear;
clc;

% range image
im=imread('0000001-000000000000.png');
figure(1)
imagesc(im);
saveas(gcf,'range_image.png')


% coordinates of the principle point and the focal length in x and y
fx=525;
cx=319.5;
fy=525;
cy=239.5;

numRow=size(im,1);
numCol=size(im,2);
cloud=zeros(numRow*numCol,3);


X=zeros(numRow,numCol);
Y=zeros(numRow,numCol);
TRUE=zeros(numRow,numCol);

%number of point I will consider, not all points in the range image are
%specified, some may be 0

k=0;
thresh=4000;

for i=1:1:numCol
    for j=1:1:numRow
        %compute the 3d coordinate X and Y
        X(j,i) = -(i-cx) *double(im(j,i))/fx;
        Y(j,i) = -(j-cy) *double(im(j,i))/fy;
        % not all the points of the range image are not encoded, discard
        % the ones which are 0 and the ones not usefull using a  threshold
        if(im(j,i)~=0 && im(j,i)<thresh)
            TRUE(j,i)=1;
            k=k+1;
            cloud(k,:)=[X(j,i) Y(j,i) double(im(j,i))];
        end
    end
end

%We take only the k points we have considered meaningfull
cloud2=cloud(1:k,:);
figure(2)
%Plot in 3d
plot3(cloud2(:,1), cloud2(:,2), cloud2(:,3), 'r.','Markersize', 1);
axis equal
saveas(gcf,'cloud_points.png')


%% store the cloud of points in a file Ply
npoint=size(cloud2,1);
Triangle=[];
exportMeshToPly(cloud2,Triangle,ones(npoint,3),'PC_out');


%% hw 1.3: Mesh reconstruction from range image

% Does not work because you have considered the distance in the range
% image, you have to consider the distance between the corresponding 3d
% points, so taking to points of the range, compute the corresponding 3d
% poitns and check if their distance is below the given threshold
clc;
Triangles=[];
cloud3=zeros(numRow*numCol,3);
k=3;
maxEdgeLength=1;
for i=1:size(X,2)-1
    for j=1:size(X,1)-1
        if(im(j,i)~=0 && im(j,i)<thresh && im(j,i+1)~=0 && im(j,i+1)<thresh && im(j+1,i)~=0 && im(j+1,i)<thresh)
            if(abs(im(j,i)-im(j+1,i))<maxEdgeLength && abs(im(j,i)-im(j,i+1))<maxEdgeLength && abs(im(j,i+1)-im(j+1,i))<maxEdgeLength)
                
                cloud3(k/3,:)=[X(j,i) Y(j,i) double(im(j,i))];
                cloud3(k/3+1,:)=[X(j+1,i) Y(j+1,i) double(im(j+1,i))];
                cloud3(k/3+2,:)=[X(j,i+1) Y(j,i+1) double(im(j,i+1))];
               
                Triangles(k/3,:)=[k/3, k/3+1, k/3+2];
                k=k+3;
%                 Triangles(:,end+1)=[X(j,i);Y(j,i);double(im(j,i))];
%                 Triangles(:,end+1)=[X(j+1,i);Y(j+1,i);double(im(j+1,i))];
%                 Triangles(:,end+1)=[X(j,i+1);Y(j,i+1);double(im(j,i+1))];
                
            end
        end
        
        if(im(j+1,i+1)~=0 && im(j+1,i+1)<thresh && im(j,i+1)~=0 && im(j,i+1)<thresh && im(j+1,i)~=0 && im(j+1,i)<thresh)
            if(abs(im(j+1,i+1)-im(j+1,i))<maxEdgeLength && abs(im(j+1,i+1)-im(j,i+1))<maxEdgeLength && abs(im(j,i+1)-im(j+1,i))<maxEdgeLength)
                
                cloud3(k/3,:)=[X(j+1,i+1) Y(j+1,i+1) double(im(j+1,i+1))];
                cloud3(k/3+1,:)=[X(j+1,i) Y(j+1,i) double(im(j+1,i))];
                cloud3(k/3+2,:)=[X(j,i+1) Y(j,i+1) double(im(j,i+1))];
               
                Triangles(k/3,:)=[k/3, k/3+1, k/3+2];
                k=k+3;
%                 Triangles(:,end+1)=[X(j,i);Y(j,i);double(im(j,i))];
%                 Triangles(:,end+1)=[X(j+1,i);Y(j+1,i);double(im(j+1,i))];
%                 Triangles(:,end+1)=[X(j,i+1);Y(j,i+1);double(im(j,i+1))];
                
            end
        end
    end
end

% cloud4=cloud3(1:(k/3),:);
cloud4=cloud3;
npoint=size(cloud4,1);
exportMeshToPly(cloud4,Triangles,ones(npoint,3),'PC_out_range');
