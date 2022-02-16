%% Use morphological operators and region properties to infer informations

clc;
clear;

I = imread('coins_usb.png');
I=rgb2gray(I);
figure;
subplot(2,2,1)
imshow(I);
title('rgb2gray image');


bw = imbinarize(I);
subplot(2,2,2)
imshow(bw);
title('Binarized image');
bw = bwareaopen(bw,50);
se = strel('disk',15);
bw = imclose(bw,se);
subplot(2,2,3)
imshow(bw);
title('Image after noise removal');

stats = regionprops('table',bw,'Centroid','MajorAxisLength','MinorAxisLength','Eccentricity','Area');
%Store the x- and y-coordinates of the centroids into a two-column matrix

%Get centroid locations.
centroids = cat(1,stats.Centroid);
eccentricity=stats.Eccentricity;

%selecting the circles in base of the Eccentricity
circleStats=stats;
circleStats(:,:)=[];
for i=1:size(stats,1)
    if(stats.Eccentricity(i)<0.3)
        circleStats= [circleStats;stats(i,:)];
    end
end

%selecting the other object stats
otherObjectStats=stats;
otherObjectStats(:,:)=[];
for i=1:size(stats,1)
    if(stats.Eccentricity(i)>0.8)
        otherObjectStats= [otherObjectStats;stats(i,:)];
    end
end

otherObjectStatsLabels = strings(1,size(otherObjectStats,1));
otherObjectStatsLabels(:)='Usb pen';


%classification of the circles in base of their area, small big coin
circlesArea=circleStats.Area;
circleLabels = strings(1,size(circleStats,1));

for i=1:size(circlesArea,1)
    if(circlesArea(i)>20000)
        circleLabels(i) = 'Big coin';
    else
        circleLabels(i) = 'Small coin';
    end
end


%Get centers and radii of the circles.
centers = circleStats.Centroid;
diameters = mean([circleStats.MajorAxisLength circleStats.MinorAxisLength],2);
radiuses = diameters/2;

subplot(2,2,4)
imshow(I)
title('Recognition of objects')
hold on
plot(centroids(:,1),centroids(:,2),'.', 'markerSize', 14, 'color', 'r');

%Plot the circles.
viscircles(centers,radiuses,'Color','blue');
text(centers(:,1)+30,centers(:,2),circleLabels,'VerticalAlignment','bottom','HorizontalAlignment','left','Color','red','FontWeight','bold')
text(otherObjectStats.Centroid(:,1)+30,otherObjectStats.Centroid(:,2),otherObjectStatsLabels,'VerticalAlignment','bottom','HorizontalAlignment','left','Color','red','FontWeight','bold')
hold off
saveas(gcf,'image_analysis_hw_2.png')

