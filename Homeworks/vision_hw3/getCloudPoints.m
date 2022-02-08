function [cloud2]=getCloudPoints(showImages)

% range image
im=imread('0000001-000000000000.png');
if(showImages)
    figure(1)
    imagesc(im);
    colorbar
end
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
threshUp=643;
treshDown=100;

for i=1:1:numCol
    for j=1:1:numRow
        %compute the 3d coordinate X and Y
        X(j,i) = -(i-cx) *double(im(j,i))/fx;
        Y(j,i) = -(j-cy) *double(im(j,i))/fy;
        % not all the points of the range image are not encoded, discard
        % the ones which are 0 and the ones not usefull using a  threshold
        if(im(j,i)~=0 && im(j,i)<threshUp && im(j,i)>treshDown)
            TRUE(j,i)=1;
            k=k+1;
            cloud(k,:)=[X(j,i) Y(j,i) double(im(j,i))];
        end
    end
end

%We take only the k points we have considered meaningfull
cloud2=cloud(1:k,:);
if(showImages)
    figure(2)
    %Plot in 3d
    plot3(cloud2(:,1), cloud2(:,2), cloud2(:,3), 'r.');
    axis equal
end

end