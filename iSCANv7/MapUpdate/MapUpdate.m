function [ MapStruct ] = MapUpdate( MapStruct,Pose,R,Phi,MaxRange )
%MAPUPDATE Updates the occupancy grid map.

Factor= 10; % this factor is used to allow th utilization of uni8 map
xo  = Pose(1); yo=Pose(2);

Phi(R>MaxRange)=[];
R(R>MaxRange)=[];

xl  = single(R.*cos(Phi+Pose(3))+xo);
yl  = single(R.*sin(Phi+Pose(3))+yo);
%eps = single(1 / MapStruct.MapRes /2);

% Filter Lidar points outside the map
yl(xl > MapStruct.Dim(1)+MapStruct.GridLocationInWorld(1))=[];
xl(xl > MapStruct.Dim(1)+MapStruct.GridLocationInWorld(1))=[];

yl(xl < MapStruct.GridLocationInWorld(1))=[];
xl(xl < MapStruct.GridLocationInWorld(1))=[];


xl(yl > MapStruct.Dim(2)+MapStruct.GridLocationInWorld(2))=[];
yl(yl > MapStruct.Dim(2)+MapStruct.GridLocationInWorld(2))=[];

xl(yl < MapStruct.GridLocationInWorld(2))=[];
yl(yl < MapStruct.GridLocationInWorld(2))=[];


% convert cartisian into rows and columns
cl  =  ceil((xl-MapStruct.GridLocationInWorld(1))*MapStruct.MapRes);
ymax = MapStruct.GridLocationInWorld(2)+MapStruct.Dim(2);
rl  =  floor((ymax-yl)*MapStruct.MapRes+1);

% Removing points outside the map
rl(cl>size(MapStruct.Map,2))=[];
cl(cl>size(MapStruct.Map,2))=[];

rl(cl<1)=[];
cl(cl<1)=[];

cl(rl>size(MapStruct.Map,1))=[];
R(rl>size(MapStruct.Map,1))=[];
rl(rl>size(MapStruct.Map,1))=[];
cl(rl<1)=[];
rl(rl<1)=[];

co  =  ceil((xo-MapStruct.GridLocationInWorld(1))*MapStruct.MapRes);
ro  =  floor((ymax-yo)*MapStruct.MapRes+1);


indicesBlack = sub2ind(size(MapStruct.Map), rl, cl);
% update the LIDAR points in the map with probability eqal to 0.9
MapStruct.Map(indicesBlack)=  MapStruct.Map(indicesBlack) ...
    + log (MapStruct.Occ(2)/(1-MapStruct.Occ(2)))*Factor;
Temp = MapStruct.Map(indicesBlack);
c_Sel=[];
r_Sel=[];
for ctr=1:length(xl)
    % Find the approximate points of the ray using Bresenham Algorithm
    [c_SelTemp, r_SelTemp]=bresenham(cl(ctr),rl(ctr),co,ro);
    c_Sel = [c_Sel; c_SelTemp(1:end-1)]; %#ok<AGROW> % we used end-1 to remove tha last cell as it is the target
    r_Sel = [r_Sel; r_SelTemp(1:end-1)]; %#ok<AGROW>
    
end

% update the LIDAR points in the map with probability eqal to 0.1
% Line=improfile(MapStruct.Map,[0 3],[1 2]);
% Line=Line+ log (MapStruct.Occ(1)/(1-MapStruct.Occ(1)))*Factor;
% MapStruct.Map=insertShape(MapStruct.Map,'line',[1 1,50 50],'LineWidth',1);
indicesWight = sub2ind(size(MapStruct.Map), r_Sel, c_Sel);
MapStruct.Map(indicesWight)=MapStruct.Map(indicesWight) ...
    + log (MapStruct.Occ(1)/(1-MapStruct.Occ(1)))*Factor;
MapStruct.Map(indicesBlack) = Temp;
end

