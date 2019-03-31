function [ MapStruct ] = InitMap( alg )
%INITMAP Summary of this function goes here
%   Detailed explanation goes here
Factor=10;
MapStruct.Dim = [max(alg.X)-min(alg.X),max(alg.Y)-min(alg.Y)]; % this is the width and hight
MapStruct.MapRes= alg.MapRes; % points per meter
MapStruct.GridLocationInWorld=[min(alg.X),min(alg.Y)]; % this is location of the grid with respect to world map
MapStruct.Saturation = alg.Saturation;
MapStruct.Occ = alg.Occ;
p = ones(round(fliplr(MapStruct.Dim)*MapStruct.MapRes))*0.5; % initialize the map

MapStruct.Map = int16(log(p./(1-p))*Factor); % The map contains the logodds 
end

