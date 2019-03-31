function [  ] = ShowMap( MainMap,H_fig )
%SHOWMAP Accepts an occupancy grid map structure and plots the map
%   Detailed explanation goes here
figure(H_fig)
Factor=10;
ax=gca;
xlimts= MainMap.GridLocationInWorld(1)+[0,MainMap.Dim(1)];
ylimts=MainMap.GridLocationInWorld(2)+[0,MainMap.Dim(2)];
p = exp(double(MainMap.Map)/Factor) ./ (1+exp(double(MainMap.Map)/Factor));
imagesc(xlimts,fliplr(ylimts),imcomplement(p),MainMap.Saturation);
colormap(gray)
 
axis equal
ax.YAxis.Direction='normal';
xlim(xlimts)
ylim(ylimts)
grid on
box on
set(gca, 'layer', 'top')

end

