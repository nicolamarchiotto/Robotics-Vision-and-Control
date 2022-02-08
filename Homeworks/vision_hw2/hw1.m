clc;
clear;

I = imread('eight.tif');
figure;
subplot(2,2,1)
imshow(I);
title('Selected image')

bw = imbinarize(I);
bw = bwareaopen(bw,50);
bw = imcomplement(bw);
se = strel('disk',5);
bw = imclose(bw,se);
subplot(2,2,2)
imshow(bw);
title('Image after morphologial operations')


cc = bwconncomp(bw,4);

labeled = labelmatrix(cc);
RGB_label = label2rgb(labeled,'spring','c','shuffle');
subplot(2,2,3)
imshow(RGB_label)
title('Colored image')



coindata = regionprops(cc,'basic');
coin_areas = [coindata.Area];
[min_area, idx] = min(coin_areas);

coin = false(size(bw));
coin(cc.PixelIdxList{idx}) = true;
subplot(2,2,4)
imshow(coin)
title('Image with only one selected coin')

saveas(gcf,'image_analysis_hw_1.png')


