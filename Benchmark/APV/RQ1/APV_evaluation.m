clear;
close all;
bdclose('all');

addpath('F:\CPS\CPS_git\Benchmarks\benchmark_RL\APV_RL');

%% setup parameters
T = 12;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;
isTerminate = 0;       % never terminate sim in middle

max_x_error = 3;        % maximum position error on x-axis
max_y_error = 3;        % maximum position error on y-axis
max_theta_error = 2;    % maximum orientation error on theta

max_steer = 0.785;      % maximum steering angle in rad
min_steer = -0.785;     % minumun steering angle in rad

max_vel = 6;            % maximum velocity in m/s
min_vel = -6;           % minimum velocity in m/s

x_error_offset = 4;     % x-axis error reward offset on denominator
y_error_offset = 4;     % x-axis error reward offset on denominator
theta_error_offset = 3; % theta-axis error reward offset on denominator

vdims = vehicleDimensions;
egoWheelbase = vdims.Wheelbase;
distToCenter = 0.5*egoWheelbase;

% Ego initial pose: x(m), y(m) and yaw angle (rad)
egoInitialPose = [4,12,0];

parkNorth = true;
if parkNorth
    egoTargetPose = [36,45,pi/2];
else
    egoTargetPose = [27.2,4.7,-pi/2];
end

costmap = helperSLCreateCostmap();
centerToFront = distToCenter;
centerToRear = distToCenter;
helperSLCreateUtilityBus;
costmapStruct = helperSLCreateUtilityStruct(costmap);

if parkNorth
    midPoint = [4,34,pi/2];
else
    midPoint = [27,12,0];
end

% Prediction horizon
p = 100;
% Control horizon
c = 100;
% Weight matrices for terminal cost
Qt = 0.5*diag([10 5 20]); 
Rt = 0.1*diag([1 2]);
% Weight matrices for tracking cost
if parkNorth
    Qp = 1e-6*diag([2 2 0]);
    Rp = 1e-4*diag([1 15]);
else
    Qp = 0*diag([2 2 0]);
    Rp = 1e-2*diag([1 5]);
end
% Safety distance to obstacles (m)
safetyDistance = 0.1;
% Maximum iteration number
maxIter = 70;
% Disable message display
mpcverbosity('off');

% Create the NLMPC controller using the specified parameters.
[nlobj,opt,paras] = createMPCForParkingValet(p,c,Ts,egoInitialPose,egoTargetPose,...
    maxIter,Qp,Rp,Qt,Rt,distToCenter,safetyDistance,midPoint);

% Set the initial conditions for the ego vehicle.
x0 = egoInitialPose';
u0 = [0;0];

% Generate the reference trajectory using the nlmpcmove function.
tic;
[mv,nloptions,info] = nlmpcmove(nlobj,x0,u0,[],[],opt);

timeVal = toc;
xRef = info.Xopt;
uRef = info.MVopt;

analyzeParkingValetResults(nlobj,info,egoTargetPose,Qp,Rp,Qt,Rt,...
    distToCenter,safetyDistance,timeVal)

% set the simulation duration and update the reference trajectory based on the duration
Duration = T;
Tsteps = Duration/Ts;
Xref = [xRef(2:p+1,:);repmat(xRef(end,:),Tsteps-p,1)];

% Create an NLMPC controller with a tracking prediction horizon (pTracking) of 10.
pTracking = 10;
nlobjTracking = createMPCForTrackingParkingValet(pTracking,Xref);

mdl = 'APV_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
% agent = load('APV_DDPG_Agent_9_19.mat');
agent = load('APV_TD3_Agent_9_27.mat');
agent = agent.agent;

% file_name = 'APV_eval_T_result';
% file_name = 'APV_eval_DDPG_result';
file_name = 'APV_eval_TD3_result';

control_switch = 0;         % 0: RL controller, 1: original controller

x_steady_threshold = 0.5;
y_steady_threshold = 0.5;
theta_steady_threshold = 0.2;

S1_x = ones(sim_num,1);
S1_y = ones(sim_num,1);
S2_x_avg = ones(sim_num,1);
S2_x_max = ones(sim_num,1);
S2_y_avg = ones(sim_num,1);
S2_y_max = ones(sim_num,1);
S2_theta_avg = ones(sim_num,1);
S2_theta_max = ones(sim_num,1);
S3_x = ones(sim_num,1);
S3_y = ones(sim_num,1);
S3_theta = ones(sim_num,1);
S4_x = ones(sim_num,1);
S4_y = ones(sim_num,1);
S4_theta = ones(sim_num,1);

for index = 1:sim_num

    sim(mdl);
    x_error = logsout.getElement('x_error').Values.Data;
    step_num = length(x_error);                  % number of samples
    x_error = reshape(x_error, [step_num,1]);
    y_error = logsout.getElement('y_error').Values.Data;
    y_error = reshape(y_error, [step_num,1]);
    theta_error = logsout.getElement('theta_error').Values.Data;
    theta_error = reshape(theta_error, [step_num,1]);

    % T1: average & maximum error 
    S2_x_avg(index) = mean(x_error(1/Ts : length(x_error)));               % average lateral deviation value
    S2_x_max(index) = max(x_error(1/Ts : length(x_error)));                % maximum lateral deviation value
    S2_y_avg(index) = mean(y_error(1/Ts : length(y_error)));               % average lateral deviation value
    S2_y_max(index) = max(y_error(1/Ts : length(y_error)));                % maximum lateral deviation value
    S2_theta_avg(index) = mean(theta_error(1/Ts : length(theta_error)));               % average lateral deviation value
    S2_theta_max(index) = max(theta_error(1/Ts : length(theta_error)));                % maximum lateral deviation value
    S1_x = sum(x_error(1/Ts : length(x_error)) > 1);
    S1_y = sum(y_error(1/Ts : length(y_error)) > 1);

    % T2: steady-state
    x_steady_error = (x_error-x_steady_threshold)<0;                % mu under steady-state
    x_steady_perct = sum(x_steady_error)/length(x_steady_error);    % percentage of samples under steady-state
    S3_x(index) = x_steady_perct;
    
    y_steady_error = (y_error-y_steady_threshold)<0;                % mu under steady-state
    y_steady_perct = sum(y_steady_error)/length(y_steady_error);    % percentage of samples under steady-state
    S3_y(index) = y_steady_perct;
    
    theta_steady_error = (theta_error-theta_steady_threshold)<0;                % mu under steady-state
    theta_steady_perct = sum(theta_steady_error)/length(theta_steady_error);    % percentage of samples under steady-state
    S3_theta(index) = theta_steady_perct;

    % T3: Resilience
    S4_x(index) = nan;
    S4_y(index) = nan;
    
    if x_steady_perct ~= 1
        x_unsteady_error = (x_error-x_steady_threshold)>0;          % mu beyond steady-state, boolean array
        x_unsteady_num = sum(x_unsteady_error);                % num of times mu beyond from unsteady-state
        x_recovery = 0;                                   % num of times mu recovery from unsteady-state
        x_unsteady_index = find(x_unsteady_error == 1);
        
        for i = 1:length(x_unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if x_unsteady_index(i) < maxsteps-1/Ts
                x_recovery = x_recovery + any(x_unsteady_error(x_unsteady_index(i):x_unsteady_index(i)+1/Ts)<1);
            end
        end
        x_recovery_rate = x_recovery/x_unsteady_num;          % percentage of recovery
        S4_x(index) = x_recovery_rate;
    end
    
     if y_steady_perct ~= 1
        y_unsteady_error = (y_error-y_steady_threshold)>0;          % mu beyond steady-state, boolean array
        y_unsteady_num = sum(y_unsteady_error);                % num of times mu beyond from unsteady-state
        y_recovery = 0;                                   % num of times mu recovery from unsteady-state
        y_unsteady_index = find(y_unsteady_error == 1);
        
        for i = 1:length(y_unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if y_unsteady_index(i) < maxsteps-1/Ts
                y_recovery = y_recovery + any(y_unsteady_error(y_unsteady_index(i):y_unsteady_index(i)+1/Ts)<1);
            end
        end
        y_recovery_rate = y_recovery/y_unsteady_num;          % percentage of recovery
        S4_y(index) = y_recovery_rate;
     end   
    
     if theta_steady_perct ~= 1
        theta_unsteady_error = (theta_error-theta_steady_threshold)>0;          % mu beyond steady-state, boolean array
        theta_unsteady_num = sum(theta_unsteady_error);                % num of times mu beyond from unsteady-state
        theta_recovery = 0;                                   % num of times mu recovery from unsteady-state
        theta_unsteady_index = find(theta_unsteady_error == 1);
        
        for i = 1:length(theta_unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if theta_unsteady_index(i) < maxsteps-1/Ts
                theta_recovery = theta_recovery + any(theta_unsteady_error(theta_unsteady_index(i):theta_unsteady_index(i)+1/Ts)<1);
            end
        end
        theta_recovery_rate = theta_recovery/theta_unsteady_num;          % percentage of recovery
        S4_theta(index) = theta_recovery_rate;
    end  
     
    if rem(index,25)== 0
        disp(index);
    end
end

detail_result = struct('S1_x',S1_x,'S1_y',S1_y,'T1_x_avg',S2_x_avg, 'T1_x_max',S2_x_max, 'T2_x_percent',S3_x,'T3_x',S4_x,...
                        'T1_y_avg',S2_y_avg, 'T1_y_max',S2_y_max, 'T2_y_percent',S3_y,'T3_y',S4_y,...
                        'T1_theta_avg',S2_theta_avg, 'T1_theta_max',S2_theta_max, 'T2_theta_percent',S3_theta,'T3_theta',S4_theta);        
                    
save(append(file_name,'_detailed.mat'),"detail_result");

S4_x = mean(S4_x(~isnan(S4_x)));
S4_y = mean(S4_y(~isnan(S4_y)));
S4_theta = mean(S4_theta(~isnan(S4_theta)));

final_result = struct('S1_x',sum(S1_x>0),'S1_y',sum(S1_y>0), 'T1_x_avg',mean(S2_x_avg), 'T1_x_max',mean(S2_x_max),'T2_x_num',sum(S3_x~=1),'T2_x_percent',mean(S3_x),'T3_x',S4_x,...
                'T1_y_avg',mean(S2_y_avg), 'T1_y_max',mean(S2_y_max),'T2_y_num',sum(S3_y~=1),'T2_y_percent',mean(S3_y),'T3_y',S4_y,...
                'T1_thetaavg',mean(S2_theta_avg), 'T1_theta_max',mean(S2_theta_max),'T2_theta_num',sum(S3_theta~=1),'T2_theta_percent',mean(S3_theta),'T3_theta',S4_theta);
            
save(append(file_name,'_.mat'),"final_result");