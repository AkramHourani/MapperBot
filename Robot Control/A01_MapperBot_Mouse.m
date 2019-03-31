%%
%This scipt is to controll MapperBot for colleting LIDAR measuments and
%performing movments
%Akram Al-Hourani, RMIT Univerisyt 

%%
clc
close all hidden
clearvars -except TelnetPortMotors TelnetPortLIDAR ip_address
addpath('..\iSCANv7\MapUpdate')
%% loading parameters
Parameters
alg.X = [-5,10]; % X is the center of the cell
alg.Y = [-5,10]; % Y is the center of the cell

%% Creating TCP/IP Session
if (~exist ('TelnetPortMotors'))
    fprintf('Creating TCP/IP object motor\n');
    TelnetPortMotors = tcpip(Bot.ip_address, 3000,'InputBufferSize',500,'Timeout',20,'NetworkRole', 'client');
    if (strcmp(TelnetPortMotors.Status,'closed'))
        fopen(TelnetPortMotors);
    end
    %fprintf(TelnetPortMotors,'#')
    fprintf('Ok\n');
    pause(1);
    fprintf('Ready!!\n\n');
elseif (strcmp(TelnetPortMotors.Status,'closed'))
    fprintf('Opening telnet port motor\n');
    fopen(TelnetPortMotors);
end

if (~exist ('TelnetPortLIDAR'))
    fprintf('Creating TCP/IP object LIDAR\n');
    TelnetPortLIDAR = tcpip(Bot.ip_address, 3001,'InputBufferSize',Bot.InputBuffer,'Timeout',20,'NetworkRole', 'client');
    if (strcmp(TelnetPortLIDAR.Status,'closed'))
        fopen(TelnetPortLIDAR);
    end
    %fprintf(TelnetPortLIDAR,'#')
    fprintf('Ok\n');
    pause(1);
    fprintf('Ready!!\n\n');
elseif (strcmp(TelnetPortLIDAR.Status,'closed'))
    fprintf('Opening telnet port LIDAR\n');
    fopen(TelnetPortLIDAR);
end
pause(2);
flushinput(TelnetPortMotors)
flushinput(TelnetPortLIDAR)
fprintf(TelnetPortMotors,'R#'); %Send a LIDAR Reset command
pause(3);
flushinput(TelnetPortMotors)
flushinput(TelnetPortLIDAR)
% fclose(TelnetPortMotors);
% fclose(TelnetPortLIDAR);
%fprintf(TelnetPortMotors,'+2000+20009000#');
%fprintf(TelnetPortMotors,'+2000+2000200#');
%pause(0.2)
%fprintf(TelnetPortMotors,'-2000-2000200#');
%fprintf(TelnetPortLIDAR,'S#');
%% Creating Camera object
MapperBotCam = ipcam(['http://', Bot.ip_address,':8081/'])
preview(MapperBotCam)
%%
x_Current =     0;
y_Current =     0;
theta_Current = 0;
x_Trace=x_Current;
y_Trace=y_Current;
x_LidarTrace =[];
y_LidarTrace =[];
Phi=[];
LIDARDist=[];
odomUpdate=[];
Odo_Trans=[];
Odo_Rotation=[];
LIDAR_x=[]; % LIDAR points
LIDAR_y=[];

R=[];
L = Bot.L;
RotateFactor = Bot.RotateFactor; % number of clicks for a full circle

%% Flush ports
flushinput(TelnetPortLIDAR); % clear the serial buffer
flushinput(TelnetPortMotors);
%% Craeting the figure
fig=figure;
axis([min(alg.X) max(alg.X) min(alg.Y) max(alg.Y)]);
ax=gca;
%% Creating Occupancy Grida
OccuMap = InitMap(alg);
RUN=1;

%% Main Loop
while(1)
    % Store the current pose
    Pose_data{RUN}=[x_Current, y_Current, theta_Current];
    X_Arrow = 0.1*cos(theta_Current) +x_Current;
    Y_Arrow = 0.1*sin(theta_Current) +y_Current;
     
    hold off;
    ShowMap(OccuMap,fig)
    hold on; grid on; box on;
    plot (X_Arrow,Y_Arrow,'o','markersize',2,'color',ax.ColorOrder(2,:),'linewidth',2);
    plot (x_Trace,y_Trace,':','linewidth',2,'color',ax.ColorOrder(1,:));
    plot (x_Trace,y_Trace,'+','markersize',5','linewidth',2,'color',ax.ColorOrder(1,:));
    plot (LIDAR_x,LIDAR_y,'.','linewidth',2,'color',ax.ColorOrder(1,:));
    line ([x_Current,X_Arrow],[y_Current,Y_Arrow],'color',ax.ColorOrder(2,:),'linewidth',2);
    
    %% Get input from user
    fprintf('Getting new destination, press "x" to exit, "f" forward, "l" left, "r" right, "d" to enter distance manually, "a" enter the angle manually, "m" manual control\n');
    %zoom in around the robot
    axis([x_Current-2 x_Current+2 y_Current-2 y_Current+2]);
    [Travel_Odo, Angle_Odo, Travel_Command, Rotation_Command,BreakFlag]= GetUserInput(alg,Bot,x_Current,y_Current,theta_Current);
    
    Odo_data{RUN} = [Travel_Odo Travel_Odo];
    if (Travel_Odo==0)
        Odo_data{RUN} = [-Angle_Odo Angle_Odo];
    end
    if(BreakFlag==1) % This will end the experiment
        break
    end
    
    %% Scanning LIDAR
    fprintf('Sending scanning command\n');
    Error=1;
    while(Error==1)
        [R,Phi,Error]=ReadLIDAR(TelnetPortLIDAR,Bot);
        
        if (Error ==1) % When an error ocurs in reading the LIDAR
            prompt = 'An error occured during LIDAR read. Press any key to continue';
            input(prompt);
            % Sending a reset commnad to the master arduino
            fclose(TelnetPortLIDAR);
            fprintf(TelnetPortMotors,'R#');
            pause(7);
            fclose(TelnetPortLIDAR);
            pause(1);
            fopen(TelnetPortLIDAR);
            pause(4);
            flushinput(TelnetPortLIDAR);
            fprintf('LIDAR head reset sequence completed!\n')
        end
    end
    
    R_data{RUN}=R;
    Phi_data{RUN}=Phi;
    
    %update the occupancy grid
    OccuMap=MapUpdate(OccuMap,Pose_data{RUN},R_data{RUN},Phi_data{RUN},Bot.rho_Max);
    
    % PLot current LIDAR measurments
    theta =   Phi+ theta_Current;
    LIDAR_x =  R.* cos(theta) + x_Current;
    LIDAR_y =  R.* sin(theta) + y_Current;
    
    %% Rotation
    MapperBotMove( TelnetPortMotors,Rotation_Command,Bot);
    [ x, y, theta] = OdoToCart( x_Current,y_Current,theta_Current, L,[0 -Angle_Odo/Bot.DistFactor/100 ], [0 Angle_Odo/Bot.DistFactor/100 ] );
    
    x_Current       = x(end); % update pose
    y_Current       = y(end); %
    theta_Current   = theta(end);
    
    
    %% Motion
    MapperBotMove( TelnetPortMotors,Travel_Command,Bot);
    [ x, y, theta] = OdoToCart( x_Current,y_Current,theta_Current, L,[0 Travel_Odo/Bot.DistFactor/100], [0 Travel_Odo]/Bot.DistFactor/100 );
    
    x_Current       = x(end); % update pose
    y_Current       = y(end); %
    theta_Current   = theta(end);
    
    x_Trace         =[x_Trace, x_Current];
    y_Trace         =[y_Trace, y_Current];
    
    img{RUN} = snapshot(MapperBotCam);
    axis([min(alg.X) max(alg.X) min(alg.Y) max(alg.Y)]);
    drawnow;
    RUN=RUN+1;
end
%%
closePreview(MapperBotCam)
FileName=['data',datestr(now, 'yyyymmdd_HHMM'),'.mat']
save(FileName)
%%

function MapperBotMove( TelnetPortMotors,Travel_Command,Bot)
flushinput(TelnetPortMotors)
fprintf(TelnetPortMotors,Travel_Command);
A=[];
% Wiat until 'Ok' is recieved before allowing the loop to continuou
tic
while(strcmp(A,'Ok')==0)
    A=fgetl(TelnetPortMotors);
    if length(A)>2
        A=A(1:2);
    end
    if (toc > Bot.TimeOut)
        break
    end
end
end

function [Travel_Odo, Angle_Odo, Travel_Command, Rotation_Command,BreakFlag]= GetUserInput(alg,Bot,x_Current,y_Current,theta_Current)
BreakFlag=0;
[X_Target,Y_Target, button] = ginput(1);
axis([min(alg.X) max(alg.X) min(alg.Y) max(alg.Y)]);
drawnow;
if (or((button==120 ), (button==27))) % When using letter (x) to exit
    BreakFlag=1;
end

Detla_Y            = Y_Target-y_Current;
Delta_X            = X_Target-x_Current;

Travel_Dist        = sqrt(Delta_X^2 + Detla_Y^2);
Target_Angle       = wrapToPi(atan2(Detla_Y,Delta_X)-theta_Current);

if (button==102) % When using letter (f) forward 15cm
    Travel_Dist=Bot.HopDistasnce;
    Target_Angle=0;
elseif (button==108) % When using letter (l) left
    Travel_Dist=0;
    Target_Angle=45/180*pi;
elseif (button==114) % When using letter (l) right
    Travel_Dist=0;
    Target_Angle=-45/180*pi;
elseif (button==100)  % when distance is pressed
    prompt = 'Please enter the target distance in [cm]? ';
    Travel_Dist = input(prompt)/100;
    Target_Angle=0;
elseif (button==97)  % when angle is pressed "a"
    prompt = 'Please enter the target angle in [degrees]? ';
    Target_Angle = input(prompt)/360 *2*pi;
    Travel_Dist  = 0;
elseif (button==109) %manual control "m"
    prompt = 'Please enter the target angle in [degrees]? ';
    Target_Angle = input(prompt)/360 *2*pi;
    prompt = 'Please enter the target distance in [cm]? ';
    Travel_Dist = input(prompt)/100;
elseif (button==[]) %manual control "enter"
    Travel_Dist=0;
    Target_Angle=0;
end

Travel_Odo         = round(Travel_Dist *Bot.DistFactor*100);
Angle_Odo          = round(Target_Angle/(2*pi)* Bot.RotateFactor);


display(Travel_Dist);
display(Target_Angle*180/pi);

%% Forming motion commands
Travel_Command   = sprintf('+%04d+%04d%d#',Bot.Speed,Bot.Speed,Travel_Odo);

if Angle_Odo>0
    Rotation_Command = sprintf('-%04d+%04d%d#',0.5*Bot.Speed,0.5*Bot.Speed,abs(Angle_Odo));
else
    Rotation_Command = sprintf('+%04d-%04d%d#',0.5*Bot.Speed,0.5*Bot.Speed,abs(Angle_Odo));
end

end



