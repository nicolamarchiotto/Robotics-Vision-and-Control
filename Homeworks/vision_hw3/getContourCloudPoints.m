function [cloud2,centroid]=getContourCloudPoints(showImages)
% range image
im=imread('0000001-000000000000.png');
if(showImages)
    figure
    subplot(2,2,1)
    imagesc(im);
    colorbar
    title("Whole image")
end

numRow=size(im,1);
numCol=size(im,2);


threshUp=643;
treshDown=100;


imageSup=zeros(size(im,1),size(im,2));

for i=1:1:numCol
    for j=1:1:numRow
        if(im(j,i)~=0 && im(j,i)<threshUp && im(j,i)>treshDown)
            imageSup(j,i)=im(j,i);
        end
    end
end
if(showImages)
    subplot(2,2,2)
    imagesc(imageSup);
    colorbar
    title("Extracted plane")
end

BW=imbinarize(imageSup);
BW2=bwareaopen(BW,50);

if(showImages)
    subplot(2,2,3)
    imshow(BW2);
    title("Data cleaning")
end

    
stats = regionprops('table',BW2,'Centroid');
centroidX_index=stats.Centroid(1,1);
centroidY_index=stats.Centroid(1,2);

centroid=zeros(3,1);

fx=525;
cx=319.5;
fy=525;
cy=239.5;

centroid(1)=-(floor(centroidX_index)-cx) *double(imageSup(floor(centroidY_index),floor(centroidX_index)))/fx;
centroid(2)=-(floor(centroidY_index)-cy) *double(imageSup(floor(centroidY_index),floor(centroidX_index)))/fy;
centroid(3)=double(imageSup(floor(centroidY_index),floor(centroidX_index)));


%contourc computes the contour matrix C for use by CONTOUR
Contour=contourc(double(BW2), [1 1]);

% computation of the cloud of points related to contour
imContour=zeros(numRow,numCol);
for i=2:1:size(Contour,2)
    if(Contour(1,i)<numCol && Contour(2,i)<numRow)
        imContour(floor(Contour(2,i)),floor(Contour(1,i)))=imageSup(floor(Contour(2,i)),floor(Contour(1,i)));
    end
end
if(showImages)
    subplot(2,2,4)
    imagesc(imContour);
    colorbar
    title("Plane contour extraction")
end


X=zeros(numRow,numCol);
Y=zeros(numRow,numCol);
cloud=zeros(numRow*numCol,3);

%number of point I will consider, not all points in the range image are
%specified, some may be 0
k=0;

% coordinates of the principle point and the focal length in x and y
fx=525;
cx=319.5;
fy=525;
cy=239.5;


for i=1:1:numCol
    for j=1:1:numRow
        %compute the 3d coordinate X and Y
        X(j,i) = -(i-cx) *double(imContour(j,i))/fx;
        Y(j,i) = -(j-cy) *double(imContour(j,i))/fy;
        % not all the points of the range image are not encoded, discard
        % the ones which are 0 and the ones not usefull using a  threshold
        if(im(j,i)~=0 && imContour(j,i)<threshUp && imContour(j,i)>treshDown)
            TRUE(j,i)=1;
            k=k+1;
            cloud(k,:)=[X(j,i) Y(j,i) double(imContour(j,i))];
        end
    end
end

%We take only the k points we have considered meaningfull
cloud2=cloud(1:k,:);
if(showImages)
    figure
    %Plot in 3d
    plot3(cloud2(:,1), cloud2(:,2), cloud2(:,3), 'r.');
    hold on
    plot3(centroid(1), centroid(2), centroid(3), 'b.');
    axis equal
    title("Plane contour points in 3D")
    xlabel('x')
    ylabel('y')
    zlabel('z')
end

end