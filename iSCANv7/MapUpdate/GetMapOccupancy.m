function [ xy ] = GetMapOccupancy( MapStruc,Th )
% GetMapOccupancy( Map Structure, Threlshold ) This function converts a grey scale occuppency map into a list of
% point that have occuppency above Threlshold
% Dr Akram Hourani 2019, RMIT University

Factor=10;
% Convert Th into log odd
Th= log (Th/(1-Th))*Factor;
% for a 1D selector vector
SelectorVec=reshape(MapStruc.Map,1,numel(MapStruc.Map));
%SelectorVec= SelectorVec>Th; % Method 1 (determinstic update)
SelectorVec=rand(size(SelectorVec))<SelectorVec;
IndexVec=SelectorVec.*(1:numel(MapStruc.Map));
IndexVec(SelectorVec==0)=[];
[r,c]=ind2sub(size(MapStruc.Map),IndexVec);
 
rmax= size(MapStruc.Map,1); % Maximum number of rows
x=(c-1)*1/MapStruc.MapRes + 1/MapStruc.MapRes/2 +MapStruc.GridLocationInWorld(1);
y=(rmax-r)*1/MapStruc.MapRes+ 1/MapStruc.MapRes/2+MapStruc.GridLocationInWorld(2);

xy=[x;y];
xy=xy';

end

