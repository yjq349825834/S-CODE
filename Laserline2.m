%% test changes 2018/08/07 2312312
%%DingCheng Zhang HAHAHA
fontSize = 16;
%%Load image
[pname,adrname]=uigetfile('*.jpg');
if exist(strcat(adrname,pname))
referenceImage =  imread(strcat(adrname,pname));
figure(1);
imshow(referenceImage); 
else 
   return ;
end;

redThresh = 0.15; 
   % Threshold for red detection 
   hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 300, ...
                                'MaximumBlobArea', 3000, ...
                                'MaximumCount', 2);
%    hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... % Set box handling
%                                         'Fill', true, ...
%                                         'FillColorSource', 'Input port', ...
%                                         'Opacity', 0.4);   
   hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port');    
   
   htextinsCent = vision.TextInserter('Text', '+      X:%4d, Y:%4d', ... % Set text for centroid
                                      'LocationSource', 'Input port', ...
                                      'Color', [1 1 0], ... % yellow color
                                      'Font', 'Courier New', ...
                                      'FontSize', 14);
                                
   ExtractRed = imsubtract(referenceImage(:,:,1), rgb2gray(referenceImage)); 
   % Get red component of the image
   ExtractRed = medfilt2(ExtractRed, [3 3]);
   % Filter out the noise by using median filter
   BinaryRed = im2bw(ExtractRed, redThresh);
   % Convert the image into binary image with the red objects as white
    
   BinaryRed= bwareaopen(BinaryRed,100);
   % Remove all those pixels less than 300px
   % axes(handles.axes1); hold off;
   figure(2);
   imshow(BinaryRed);
   title('Binary Image','color',[1 1 1],'FontWeight','bold');
   
   stats = regionprops(BinaryRed, 'BoundingBox', 'Centroid');
   [centroidRed, bboxRed] = step(hblob, BinaryRed); 
   % Get the centroids and bounding boxes of the red blobs
   centroidRed = uint16(centroidRed); 
   % Convert the centroids into Integer for further steps 
   referenceImage = step(hshapeinsBox, referenceImage, bboxRed, uint8([255 2255 255])); 
   % Instert the red box
   % axes(handles.axes2); hold off; 
   figure(3);
   imshow(referenceImage);
   title('Detect Object','color',[1 1 1],'FontWeight','bold');
   
   I2 = imcrop(referenceImage,bboxRed);
   figure(4);
   imshow(I2)
   
   figure(5);
   hsvImage = rgb2hsv(I2);
   
   % Extract out the H, S, and V images individually
hImage = hsvImage(:,:,1);
sImage = hsvImage(:,:,2);
vImage = hsvImage(:,:,3);

% Display the hue image.
subplot(3, 4, 2);
h1 = imshow(hImage);
title('Hue Image', 'FontSize', fontSize);
% Set up an infor panel so you can mouse around and inspect the hue values.
hHuePI = impixelinfo(h1);
set(hHuePI, 'Units', 'Normalized', 'Position',[.36 .68 .15 .04]);
	
% Display the saturation image.
h2 = subplot(3, 4, 3);
imshow(sImage);
title('Saturation Image', 'FontSize', fontSize);
% Set up an infor panel so you can mouse around and inspect the saturation values.
hSatPI = impixelinfo(h2);
set(hSatPI, 'Units', 'Normalized', 'Position',[.57 .68 .15 .04]);
	
% Display the value image.
h3 = subplot(3, 4, 4);
imshow(vImage);
title('Value Image', 'FontSize', fontSize);
% Set up an infor panel so you can mouse around and inspect the value values.
hValuePI = impixelinfo(h3);
set(hValuePI, 'Units', 'Normalized', 'Position',[.78 .68 .15 .04]);

% Compute and plot the histogram of the "hue" band.
hHuePlot = subplot(3, 4, 6); 
[hueCounts, hueBinValues] = imhist(hImage); 
maxHueBinValue = find(hueCounts > 0, 1, 'last'); 
maxCountHue = max(hueCounts); 
% 	bar(hueBinValues, hueCounts, 'r'); 
area(hueBinValues, hueCounts, 'FaceColor', 'r'); 
grid on; 
xlabel('Hue Value'); 
ylabel('Pixel Count'); 
title('Histogram of Hue Image', 'FontSize', fontSize);

% Compute and plot the histogram of the "saturation" band.
hSaturationPlot = subplot(3, 4, 7); 
[saturationCounts, saturationBinValues] = imhist(sImage); 
maxSaturationBinValue = find(saturationCounts > 0, 1, 'last'); 
maxCountSaturation = max(saturationCounts); 
% bar(saturationBinValues, saturationCounts, 'g', 'BarWidth', 0.95); 
area(saturationBinValues, saturationCounts, 'FaceColor', 'g'); 
grid on; 
xlabel('Saturation Value'); 
ylabel('Pixel Count'); 
title('Histogram of Saturation Image', 'FontSize', fontSize);

% Compute and plot the histogram of the "value" band.
hValuePlot = subplot(3, 4, 8); 
[valueCounts, valueBinValues] = imhist(vImage); 
maxValueBinValue = find(valueCounts > 0, 1, 'last'); 
maxCountValue = max(valueCounts); 
% bar(valueBinValues, valueCounts, 'b'); 
area(valueBinValues, valueCounts, 'FaceColor', 'b'); 
grid on; 
xlabel('Value Value'); 
ylabel('Pixel Count'); 
title('Histogram of Value Image', 'FontSize', fontSize);    
    
% Plot all 3 histograms in one plot.
subplot(3, 4, 5); 
plot(hueBinValues, hueCounts, 'r', 'LineWidth', 2); 
grid on; 
xlabel('Values'); 
ylabel('Pixel Count'); 
hold on; 
plot(saturationBinValues, saturationCounts, 'g', 'LineWidth', 2); 
plot(valueBinValues, valueCounts, 'b', 'LineWidth', 2); 
title('Histogram of All Bands', 'FontSize', fontSize); 
maxGrayLevel = max([maxHueBinValue, maxSaturationBinValue, maxValueBinValue]); % Just for our information....
% Make x-axis to just the max gray level on the bright end. 
xlim([0 1]);     

% hueMask=hImage;
% hueMask(0.55<=hueMask <= 0.65)=0;
hueMask = (hImage >= 0.55) & (hImage <= 0.62);
subplot(3, 4, 9); 
imshow(hueMask);
title('HSV Hue threshold extraction', 'FontSize', fontSize);

BW1 = edge(hueMask,'canny');
subplot(3, 4, 10); 
imshow(BW1);
title('Canny edge detecction', 'FontSize', fontSize);
% h4=subplot(3, 4, 11); 
% SaturationMask = (sImage >= 0.80) & (sImage <= 1);
% imshow(SaturationMask);
% title('HSV Saturation threshold extraction', 'FontSize', fontSize);
h4=figure; 
BW2 = bwareaopen(BW1,100);
imshow(BW2);
title('Removing spur pixels ', 'FontSize', fontSize);
hSatPI = impixelinfo(h4);
set(hSatPI, 'Units', 'Normalized', 'Position',[.57 .08 .15 .04]);



BW3 = bwmorph(BW2,'bridge',Inf);
% h5=subplot(3, 4, 12); 
% BW3 = bwareaopen(SaturationMask,100);
h5 = figure;
imshow(BW3);
title('Bridge pixels ', 'FontSize', fontSize);
% imshow(BW3);
% title('Connecting scattered points ', 'FontSize', fontSize);
hValuePI = impixelinfo(h5);
set(hValuePI, 'Units', 'Normalized', 'Position',[.78 .08 .15 .04]);

[H,theta,rho] = hough(BW3);
figure
imshow(imadjust(H),[],...
       'XData',theta,...
       'YData',rho,...
       'InitialMagnification','fit');
xlabel('\theta (degrees)')
ylabel('\rho')
axis on
axis normal 
hold on
colormap(gca,hot)

P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
x = theta(P(:,2));
y = rho(P(:,1));
plot(x,y,'s','color','black');

lines = houghlines(BW3,theta,rho,P,'FillGap',5,'MinLength',7);
figure, imshow(referenceImage), hold on
max_len = 0;
bboxRed1 = double(bboxRed);
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2]+[bboxRed1(1,1),bboxRed1(1,2);bboxRed1(1,1),bboxRed1(1,2)];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
% highlight the longest line segment
plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');




% [y x]=find(BW3);
% 
% cl = (max(y)+min(y))/2;
% top = y>cl;
% yt = y(top);
% xt = x(top);
% yb = y(~top);
% xb = x(~top);
% 
% pt = polyfit(yt, xt, 3);
% pb = polyfit(yb, xb, 3);
% % p = polyfit(y, x, 3);
% yy = linspace(1, size(BW3,1),50);
% figure;
% imshow(BW3,'border','tight');
% hold all
% plot(polyval(pt,yy), yy, '.-', 'LineWidth',1);
% plot(polyval(pb,yy), yy, '.-', 'LineWidth',1);
%    
   
   