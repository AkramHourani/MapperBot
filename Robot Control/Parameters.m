%% Robot Parameters
Bot.ip_address = '10.133.18.117'; % IP address of MapperBot
Bot.Epsilon = 1; % minimum detectable distance by the lidar in cm
Bot.HopDistasnce       = 0.5; %m this is the distance for automated commands "f" "l" and "r"
Bot.Speed              = 2000; % increasing above 200 will start causing errors
Bot.DistFactor         = 10000/100; % every 1000 motor step click gives 100 cm
Bot.TimeOut            = 5; % Timeout of the connection
Bot.InputBuffer        = 5000;
Bot.rho_Max = 8; % Maximum range of the LIDAR

Bot.L                  = 0.207; % Inter-wheel distance
Bot.C                  = 0.035;  % Lidar offset
Bot.RotateFactor       = 2*pi / (2* 1/Bot.DistFactor/100 / Bot.L);
Bot.JoyTh = 0.3; % joystock dead zone
Bot.JoyVelMax = 3000; % velocity sensitivity
Bot.JoyDistMax = 2000; % Distance sensitivity

Bot.alpha = 0.005;  % Linear error of wheels step
%Bot.Beta  = 10;  % Fixed error of wheels step
Bot.Beta  = 5;  % Fixed error of wheels step
Bot.rho   = 0.005;   % Correlation coeffeceint between the two wheels

%% Algorithm Paramters

alg.X = [-5,10]; % X is the center of the cell
alg.Y = [-5,15]; % Y is the center of the cell
alg.Saturation = [0.1,0.9]; % map saturation probabilites for unocc, occ cases.
alg.rho_Search = 0.35; % this is the maximum distance to seek neighbout point
alg.Part = 1; % number of particels parallel streams
alg.MapRes = 12; % number of points per meter

alg.rho_Local = Bot.rho_Max; % Local Map raduis
alg.ReSampleTH=0.80; %Resampling threshold
alg.SmallSig = 0.15; % this is the std of the search area for laser matching (Anchoring)
alg.Occ      = [0.3 0.7]; % This is the updating paramters when haing a LIDAr meas;

