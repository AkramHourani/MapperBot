% iScan SLAM Algorithm Version 7
% A. Al-Hourani, RMIT university 2017
% Updated 20190327
% Condition of use, non-commercial.
% For commercial use, need to contact the author
% For academic use, plese cite the asciated article
%% Itialization
clc
clear global 
%% Add pathes
addpath('..\Measurements')
addpath('MapUpdate')
addpath('Functions')

%% Select here the file name to process
%InputFile = 'Data1_OfficesType1';
%InputFile = 'Data2_OfficesType2';
InputFile = 'Data3_House';

%% Loading the file and cleaing some
load(InputFile);
close all
clearvars -except Pose_data R_data Phi_data Bot alg  Odo_data InputFile x_0
%% Load the Parameters
% You can set the SLAM paramters in the Paramters.m file
Parameters
Plotting=1; % set to zero in case you do need poltting
Plot_Uncorrected = 0; % This is for poltting uncorrected odometry, i.e., before applying iSCAN
alg.X = [-3 13];
alg.Y = [-6 17];

%% Creating the main map
MainMap = InitMap(alg);%Initialize the Main Map
%% Initialization of variables

Frame_ctr =1;
if (Plotting==1)
    h_fig=figure('Position',[50 50 800 800]);
    ax=gca; box on; grid on;
end


%% Initialize the particles and the maps (this is t=0)
for Particle=1:alg.Part
    % Apply angle correction to the first step
    % This is just to allign the map with the x-y axis
    ParticleEstPose{1}(Particle,:)=Pose_data{1}+x_0;
    Score(1,:) = 1/alg.Part*ones(1,alg.Part); % initlaizing the weights
    % Initialize the maps
    ParticleMap{Particle} = InitMap(alg);
end
EstPose(1,:)=mean(ParticleEstPose{1},1);
S_StartPose = EstPose(1,:); % Search starting pose

fprintf('Maps Initilized...Ok\n')

%% Obtaining Maps at t=1
for Particle=1:alg.Part
    ParticleMap{Particle}=MapUpdate(ParticleMap{Particle},ParticleEstPose{1}(Particle,:),R_data{1},Phi_data{1},Bot.rho_Max);
end

TempMap = zeros(size(MainMap.Map)); % clearing the main map
for Particle=1:alg.Part
    p = exp(double(ParticleMap{Particle}.Map)/10) ./ (1+exp(double(ParticleMap{Particle}.Map)/10));
    TempMap= TempMap+p*Score(1,Particle);
end
MainMap.Map=log (TempMap./(1-TempMap))*10;
fprintf('Updating the first map, t=0...Ok\n')

if (Plotting==1)
    
    
    % Plotting uncorrected next Lidar scan
    NextPose = OdoToCartV2( EstPose(1,:), Odo_data{1}, Bot); % This is the mean pose trasnlated using the odometery reading
    PlotSLAM(h_fig,MainMap,[EstPose; NextPose'], ax)
    x_LIDAR = R_data{2}.*cos(Phi_data{2}+NextPose(3))+NextPose(1);
    y_LIDAR = R_data{2}.*sin(Phi_data{2}+NextPose(3))+NextPose(2);
    plot (x_LIDAR,y_LIDAR,'.','linewidth',1,'color',ax.ColorOrder(3,:));
    drawnow
end
fprintf('Time step t= 0 \n');


%% This is the main loop of the algorithm
timeval1 = tic;
for RUN=2:length(R_data)
    
    %% Perform laser matching
    timeval2 = tic;
    for Particle=1:alg.Part
        [s_o(Particle,:), ~]=MotionModelSamplingv5 (ParticleEstPose{RUN-1}(Particle,:),Odo_data{RUN-1},Bot); % Apply motion sampling accorind to the control vector
        xy = GetMapOccupancy(ParticleMap{Particle},0.65);% Binary Occupancy grid
        [ParticleEstPose{RUN}(Particle,:), w]=LaserMatchOpt( xy, R_data{RUN}, Phi_data{RUN},s_o(Particle,:),alg,Bot );
        [Score(RUN,Particle)]=w;
    end
    %% Perform sampling importance resampling
    % This section updated 20190408
    Score(RUN,:)=Score(RUN,:) /sum(Score(RUN,:));%normalizing score
    [MaxScore,MaxIDX]= max( Score(RUN,:));
    display(Score(RUN,:)/MaxScore); % make the score realtive to the maximum (this step is nor really nessasry, but make the visualization of the weights easier)
    N_eff     = 1/sum(Score(RUN,:).^2);
    NewGen =  sum(rand(4,1)>=cumsum(Score(RUN,:)),2)+1; % These are the new generation particles
    
    fprintf('New generation particles: ');
    disp(NewGen')
    %Copy particles temporarly
    for k=1:alg.Part
        OldParticleEstPoses(k,:)= ParticleEstPose{RUN}(k,:); % copy the poses
        OldParticleEstMaps{k} = ParticleMap{k}; % replace the map
    end
    
    % assign new particles
    for k=1:alg.Part
        ParticleEstPose{RUN}(k,:) = OldParticleEstPoses(NewGen(k),:);
        OldParticleEstMaps{k} =OldParticleEstMaps{NewGen(k)};
    end

    Step_Time = toc(timeval2);
    Simulation_Progress(RUN,:) = [Step_Time,N_eff];
    fprintf('N_eff = %0.2f\n',N_eff);
    fprintf('Time t= %d, run time %0.1f sec, total time %0.1f min\n',RUN-1,Step_Time,toc(timeval1)/60);
    
    %% Map update
    for Particle=1:alg.Part % Updating the maps per particle
        ParticleMap{Particle}=MapUpdate(ParticleMap{Particle},ParticleEstPose{RUN}(Particle,:),R_data{RUN},Phi_data{RUN},Bot.rho_Max );
    end
    % This will update the main map by averaging particles maps
    TempMap = zeros(size(MainMap.Map)); % clearing the main map
    for Particle=1:alg.Part
        p = exp(double(ParticleMap{Particle}.Map)/10) ./ (1+exp(double(ParticleMap{Particle}.Map)/10));
        TempMap= TempMap+p*Score(RUN-1,Particle);
    end
    MainMap.Map=log (TempMap./(1-TempMap))*10;
    %% Estimated pose
    
    % Finding the mean pose wieghted according to the particles' score
    EstPose(RUN,:)=0;
    VecSum=0;
    for k=1:alg.Part
        EstPose(RUN,1:2)=EstPose(RUN,1:2)+Score(RUN,k)*ParticleEstPose{RUN}(Particle,1:2); % averaging the particles x,y
        VecSum = VecSum+Score(RUN,k)*exp(1i*ParticleEstPose{RUN}(Particle,3));
    end
    EstPose(RUN,3) = atan2(imag(VecSum),real(VecSum));
    
    %% Plotting
    
    if (Plotting==1)
        NextPose = OdoToCartV2( EstPose(RUN,:), Odo_data{RUN}, Bot); % This is the mean pose trasnlated using the odometery reading
        PlotSLAM(h_fig,MainMap,[EstPose; NextPose'], ax)
        % Plot corrected Lidar scan
        x_LIDAR = R_data{RUN}.*cos(Phi_data{RUN}+EstPose(RUN,3))+EstPose(RUN,1);
        y_LIDAR = R_data{RUN}.*sin(Phi_data{RUN}+EstPose(RUN,3))+EstPose(RUN,2);
        plot (x_LIDAR,y_LIDAR,'.','linewidth',1,'color',ax.ColorOrder(1,:));
        drawnow
        % Plotting uncorrected next Lidar scan
        if RUN < length(R_data) % so the the last t will not execute
            x_LIDAR = R_data{RUN+1}.*cos(Phi_data{RUN+1}+NextPose(3))+NextPose(1);
            y_LIDAR = R_data{RUN+1}.*sin(Phi_data{RUN+1}+NextPose(3))+NextPose(2);
            plot (x_LIDAR,y_LIDAR,'.','linewidth',1,'color',ax.ColorOrder(3,:));
            drawnow
        end
    end
    
end
%% This will save the output file for later plotting
FileNameMatlab=[InputFile,'_V7'];
save(FileNameMatlab);

%% Functions
function PlotSLAM(h_fig,MainMap,Pose_est, ax)
%global
figure(h_fig)
%subplot(1,2,1);
hold off;
ShowMap(MainMap,h_fig)
grid on; box on;  hold on;

X_Arrow = 0.15*cos(Pose_est(end,3)) +Pose_est(end,1);
Y_Arrow = 0.15*sin(Pose_est(end,3)) +Pose_est(end,2);

plot (X_Arrow,Y_Arrow,'o','markersize',2,'color',ax.ColorOrder(2,:),'linewidth',1);
figEst=plot (Pose_est(:,1),Pose_est(:,2),'+:','linewidth',1,'color',ax.ColorOrder(1,:),'markersize',5');
plot (Pose_est(:,1),Pose_est(:,2),'+','markersize',5','linewidth',1,'color',ax.ColorOrder(1,:));
line ([Pose_est(end,1),X_Arrow],[Pose_est(end,2),Y_Arrow],'color',ax.ColorOrder(2,:),'linewidth',1);

legend(figEst,'Esitmated pose','AutoUpdate','off')
end

