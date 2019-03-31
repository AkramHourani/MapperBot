function NextPose = OdoToCartV2( Initial_Pose, Stepps, Bot)
% This is an updated function on 20190324
%ODOTOCART This function converts deferential drive motor odometry into
%Cartisian coordinates

N_l= Stepps(1);
N_r= Stepps(2);
Step = 1/Bot.DistFactor/100;

if (N_l == N_r) % i.e., norotation
    N_r = N_l+1e-3; % in order to avoide NaN
end

R = Bot.L /2 *(N_l+N_r) / (N_r-N_l); % Centre of rotation
Omega_dT     = (N_r-N_l)*Step   / Bot.L;


ICC          = [Initial_Pose(1) - R* sin(Initial_Pose(3)), ...
    Initial_Pose(2) + R* cos(Initial_Pose(3))];


NextPose = [cos(Omega_dT), - sin(Omega_dT), 0; ...
    sin(Omega_dT),   cos(Omega_dT), 0;...
    0            ,   0            , 1] *...
    [ Initial_Pose(1)-ICC(1);Initial_Pose(2)-ICC(2) ; Initial_Pose(3)] + ...
    [ICC(1); ICC(2);Omega_dT ];

end

