function [x, y]=bresenham(x1,y1,x2,y2)

% Bresenham line algorithm
x1=round(x1); x2=round(x2);
y1=round(y1); y2=round(y2);
dx=abs(x2-x1);
dy=abs(y2-y1);
steepLine=abs(dy)>abs(dx);
if steepLine
    t=dx;
    dx=dy;
    dy=t;
end

%% Normal cases
if dy==0
    q=zeros(dx+1,1);
else
    q=[0;diff(mod((floor(dx/2):-dy:-dy*dx+floor(dx/2))',dx))>=0];
end

%% In case of steep line
if steepLine
    if y1<=y2
        y= (y1:y2)';
    else
        y=(y1:-1:y2)';
    end
    if x1<=x2
        x=x1+cumsum(q);
    else
        x=x1-cumsum(q);
    end
else
    if x1<=x2
        x=(x1:x2)';
    else
        x=(x1:-1:x2)';
    end
    if y1<=y2
        y=y1+cumsum(q);
    else
        y=y1-cumsum(q);
    end
end