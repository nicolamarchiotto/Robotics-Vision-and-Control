%% Matlab demo available on image analysis:
% https://it.mathworks.com/help/images/correcting-nonuniform-illumination.html
clc;
clear;

I = imread('rice.png');
figure;
imshow(I)

%%
% Remove all of the foreground (rice grains) using morphological opening
% and subtract the background approximation image, background, from the original image, I
I2 = imtophat(I,strel('disk',15));
imshow(I2)

%% 
%Use imadjust to increase the contrast of the processed image I2 by saturating 1% of
%the data at both low and high intensities and by stretching the intensity values to fill the uint8 dynamic range.
I3 = imadjust(I2);
imshow(I3)


%%
%Create a binary version of the processed image so you can use toolbox functions for analysis.
%Use the imbinarize function to convert the grayscale image into a binary image. 
%Remove background noise from the image with the bwareaopen function.

bw = imbinarize(I3);
bw = bwareaopen(bw,50);
imshow(bw)

%%
%Find all the connected components (objects) in the binary image.
cc = bwconncomp(bw,4);

%View the rice grain that is labeled 50 in the image.

grain = false(size(bw));
grain(cc.PixelIdxList{50}) = true;
imshow(grain)

%%
% Visualize all the connected components in the image by creating a label matrix and then displaying it 
%as a pseudocolor indexed image.

labeled = labelmatrix(cc);

%Use label2rgb to choose the colormap, the background color, and how objects in the label matrix mapto colors in the colormap.
%In the pseudocolor image, the label identifying each object in the label
%matrix maps to a different color in an associated colormap matrix.

RGB_label = label2rgb(labeled,'spring','c','shuffle');
imshow(RGB_label)


%%
%Compute the area of each object in the image using regionprops. 
%Each rice grain is one connected component in the cc structure.

graindata = regionprops(cc,'basic');
grain_areas = [graindata.Area];
[min_area, idx] = min(grain_areas);

grain = false(size(bw));
grain(cc.PixelIdxList{idx}) = true;
imshow(grain)

%Use the histogram command to create a histogram of rice grain areas.

histogram(grain_areas)
title('Histogram of Rice Grain Area')
