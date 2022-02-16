%% Use morphological operators and region properties to infer informations on an image chosen by you
clc;
clear;

I = imread('banana.jpg');
I=rgb2gray(I);

figure;
subplot(2,2,1)
imshow(I);
title('rgb2gray image');

I = imadjust(I);
bw = imbinarize(I);
subplot(2,2,2)
imshow(bw);
title('Binarized image');


bw = bwareaopen(bw,20);
se = strel('disk',15);
bw = imclose(bw,se);

cc = bwconncomp(bw,4);
S = regionprops('table',cc,'Area');
filtered = false(size(bw));

for i=1:size(S,1)
    if(S.Area(i)>100000)
        filtered(cc.PixelIdxList{i}) = true;
    end
end
bw=filtered;
bw=imfill(bw,'holes');
subplot(2,2,3)
imshow(bw)
title('Image after noise removal');


stats = regionprops('table',bw,'Area','Circularity','Centroid','MajorAxisLength','MinorAxisLength','ConvexHull');

circleStats=stats;
circleStats(:,:)=[];

rectangleStats=stats;
rectangleStats(:,:)=[];

bananaStats=stats;
bananaStats(:,:)=[];

for i = 1 : size(stats,1)
    if(stats.Circularity(i)>0.75)
        circleStats= [circleStats;stats(i,:)];
    else
        CH=stats.ConvexHull(i,:);
        X=CH{1,1}(:,1);
        Y=CH{1,1}(:,2);
        vi=convhull(X,Y);
        convexArea=polyarea(X(vi),Y(vi));
        if(stats.Area(i)<=convexArea*0.8)
            bananaStats= [bananaStats;stats(i,:)];
        else
            rectangleStats= [rectangleStats;stats(i,:)];
        end
    end
end

%Get centers and radii of the circles.
centers = circleStats.Centroid;
diameters = mean([circleStats.MajorAxisLength circleStats.MinorAxisLength],2);
radiuses = diameters/2;

%plotting

subplot(2,2,4);
imshow(bw)
title('Recognition of objects')
hold on
plot(stats.Centroid(:,1),stats.Centroid(:,2),'.', 'markerSize', 10, 'color', 'r');
viscircles(centers,radiuses,'Color','blue');
text(circleStats.Centroid(:,1)+30, circleStats.Centroid(:,2),"Apple",'VerticalAlignment','bottom','HorizontalAlignment','left','Color','red','FontWeight','bold')

for k=1:size(rectangleStats)
    text(rectangleStats.Centroid(k,1)+30, rectangleStats.Centroid(k,2),"Phone",'VerticalAlignment','bottom','HorizontalAlignment','left','Color','red','FontWeight','bold')
    CH=stats.ConvexHull(k,:);
    X=CH{1,1}(:,1);
    Y=CH{1,1}(:,2);
    plot(X(:),Y(:));
    vi=convhull(X,Y);
    polyarea(X(vi),Y(vi));
    fill(X(vi),Y(vi),'g','facealpha',0.4)
end

text(bananaStats.Centroid(:,1)+30, bananaStats.Centroid(:,2),"Banana",'VerticalAlignment','bottom','HorizontalAlignment','left','Color','red','FontWeight','bold')
saveas(gcf,'image_analysis_hw_4.png')


