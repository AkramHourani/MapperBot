function [ x,fval ] = LaserMatchOpt( m,R_data, Phi_data, Centre,alg,Bot )
%Obtain the distance between two point clouds
% First it will find the closest point, then caluclate the RMS od the
% distance vector
% m is the model (reference) matrix (two column vector)

% Apply the distance Filter here for faster performance
disFilter=sqrt((Centre(1)-m(:,1)).^2 + (Centre(2)-m(:,2)).^2 );
m(disFilter>Bot.rho_Max ,:)=[];% remove the points that frther than Rmax



mu     =Centre;
sig    = alg.SmallSig;
eps    = 0;
SIGMA = [sig^2, eps , eps;
    eps, sig^2, eps;
    eps, eps  , sig^2];

options = optimoptions('fminunc');
options.MaxFunctionEvaluations=800;
options.Algorithm='quasi-newton';
options.Display='off';


[x,fval] = fminunc(@nestedfun,Centre,options);
%sclose

%% Objective Function
    function Objt = nestedfun(P_pose)
        q(:,1)=R_data.*cos(Phi_data+P_pose(3))+P_pose(1);
        q(:,2)=R_data.*sin(Phi_data+P_pose(3))+P_pose(2);
       
        xx_q = repmat(q(:,1)',size(m,1),1);
        yy_q = repmat(q(:,2)',size(m,1),1);
        xx_m = repmat(m(:,1),1,size(q,1));
        yy_m = repmat(m(:,2),1,size(q,1));
        d_mat  = sqrt ( (xx_q-xx_m).^2 + (yy_q-yy_m).^2);
        d_vector = min(d_mat);
        d_vector (d_vector>alg.rho_Search)=[]; % Remove the samples that have very far neighbour
        LIDAR   = sqrt(sum(d_vector.^2)/numel(d_vector));
        
        
        w  =  mvnpdf(P_pose,mu,SIGMA); % The wieght is the pdf value of the point
        w0 =  mvnpdf(mu,mu,SIGMA);    % The wieght of the highest point
        w=w/w0;
        %w  = 1-w/w0; % Normalize the weight
        
        Objt = LIDAR * 1/w;
    end

%%


end

