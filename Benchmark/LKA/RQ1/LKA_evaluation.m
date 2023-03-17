clear;
close all;
bdclose('all');

addpath('F:\CPS\CPS_git\Benchmarks\benchmark_RL\LKA_RL');

%% setup parameters
T = 15;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;

m = 1575;   % total vehicle mass (kg)
Iz = 2875;  % yaw moment of inertia (mNs^2)
lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000; % cornering stiffness of front tires (N/rad)
Cr = 33000; % cornering stiffness of rear tires (N/rad)
Vx = 15;    % longitudinal velocity (m/s)
u_min = -0.5;   % maximum steering angle
u_max = 0.5;    % minimum steering angle

line_width = 3.7;   % highway lane width
avg_car_width = 2;  % average car width 
max_late_dev = (line_width-avg_car_width)/2-0.1;
max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
terminate_error = 1.5;
isTerminate = 0;

rho = 0.001;        %  curvature of the road
e1_initial = 0.4;   % initial lateral deviation
e2_initial = 0.1;   % initial yaw angle

PredictionHorizon = 10;     % MPC prediction horizon
time = 0:Ts:T;
md = getCurvature(Vx,time); % previewed curvature

control_switch = 0;     % 0:RL, 1:MPC

mdl = 'LKA_eval';


%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% agent = load('LKA_DDPG_Agent_9_2.mat');
% agent = load('LKA_PPO_Agent_9_18.mat');
% agent = load('LKA_A2C_Agent_9_18.mat');
agent = load('LKA_SAC_Agent_9_18.mat');

agent = agent.agent;

% file_name = 'LKA_eval_T_result';
% file_name = 'LKA_eval_DDPG_result';
% file_name = 'LKA_eval_PPO_result';
% file_name = 'LKA_eval_A2C_result';
file_name = 'LKA_eval_SAC_result';

lat_steady_threshold = 0.5;    % threshold to determine steady-state
yaw_steady_threshold = 0.1;

S1 = ones(sim_num,1);
S2_lat_avg = ones(sim_num,1);
S2_lat_max = ones(sim_num,1);
S2_yaw_avg = ones(sim_num,1);
S2_yaw_max = ones(sim_num,1);
S3_lat = ones(sim_num,1);
S3_yaw = ones(sim_num,1);
S4_lat = ones(sim_num,1);
S4_yaw = ones(sim_num,1);


for index = 1:sim_num

    sim(mdl);
    lat_error = logsout.getElement('lat_error').Values.Data;
    yaw_error = logsout.getElement('yaw_error').Values.Data;
    step_num = length(lat_error);                  % number of samples

    % T1: average & maximum error 
    
    S1_violate_num = (lat_error(1/Ts : length(lat_error))) > 0.85;
    S1(index) = sum(S1_violate_num);  
    
    S2_lat_avg(index) = mean(lat_error(1/Ts : length(lat_error)));               % average lateral deviation value
    S2_lat_max(index) = max(lat_error(1/Ts : length(lat_error)));                % maximum lateral deviation value
    S2_yaw_avg(index) = mean(yaw_error(1/Ts : length(yaw_error)));               % average lateral deviation value
    S2_yaw_max(index) = max(yaw_error(1/Ts : length(yaw_error)));                % maximum lateral deviation value

    % T2: steady-state
    lat_steady_error = (lat_error-lat_steady_threshold)<0;                % mu under steady-state
    lat_steady_perct = sum(lat_steady_error)/length(lat_steady_error);    % percentage of samples under steady-state
    S3_lat(index) = lat_steady_perct;
    yaw_steady_error = (yaw_error-yaw_steady_threshold)<0;                % mu under steady-state
    yaw_steady_perct = sum(yaw_steady_error)/length(yaw_steady_error);    % percentage of samples under steady-state
    S3_yaw(index) = yaw_steady_perct;

    % T3: Resilience
    S4_lat(index) = nan;
    S4_yaw(index) = nan;
    
    if lat_steady_perct ~= 1
        lat_unsteady_error = (lat_error-lat_steady_threshold)>0;          % mu beyond steady-state, boolean array
        lat_unsteady_num = sum(lat_unsteady_error);                % num of times mu beyond from unsteady-state
        lat_recovery = 0;                                   % num of times mu recovery from unsteady-state
        lat_unsteady_index = find(lat_unsteady_error == 1);
        
        for i = 1:length(lat_unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if lat_unsteady_index(i) < maxsteps-1/Ts
                lat_recovery = lat_recovery + any(lat_unsteady_error(lat_unsteady_index(i):lat_unsteady_index(i)+1/Ts)<1);
            end
        end
        lat_recovery_rate = lat_recovery/lat_unsteady_num;          % percentage of recovery
        S4_lat(index) = lat_recovery_rate;
    end
    
     if yaw_steady_perct ~= 1
        yaw_unsteady_error = (yaw_error-yaw_steady_threshold)>0;          % mu beyond steady-state, boolean array
        yaw_unsteady_num = sum(yaw_unsteady_error);                % num of times mu beyond from unsteady-state
        yaw_recovery = 0;                                   % num of times mu recovery from unsteady-state
        yaw_unsteady_index = find(yaw_unsteady_error == 1);
        
        for i = 1:length(yaw_unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if yaw_unsteady_index(i) < maxsteps-1/Ts
                yaw_recovery = yaw_recovery + any(yaw_unsteady_error(yaw_unsteady_index(i):yaw_unsteady_index(i)+1/Ts)<1);
            end
        end
        yaw_recovery_rate = yaw_recovery/yaw_unsteady_num;          % percentage of recovery
        S4_yaw(index) = yaw_recovery_rate;
    end   
    
    if rem(index,25)== 0
        disp(index);
    end
end

detail_result = struct('S1',S1, 'S2_lat_avg',S2_lat_avg, 'S2_lat_max',S2_lat_max, 'S3_lat_percent',S3_lat,'S4_lat',S4_lat,...
                        'S2_yaw_avg',S2_yaw_avg, 'S2_yaw_max',S2_yaw_max, 'S3_yaw_percent',S3_yaw,'S4',S4_yaw);        
                    
save(append(file_name,'_detailed.mat'),"detail_result");

S4_lat = mean(S4_lat(~isnan(S4_lat)));
S4_yaw = mean(S4_yaw(~isnan(S4_yaw)));

final_result = struct('S1',sum(S1>0), 'S2_lat_avg',mean(S2_lat_avg), 'S2_lat_max',mean(S2_lat_max),'S3_lat_num',sum(S3_lat~=1),'S3_lat_percent',mean(S3_lat),'S4_lat',S4_lat,...
                'S2_yaw_avg',mean(S2_yaw_avg), 'S2_yaw_max',mean(S2_yaw_max),'S3_yaw_num',sum(S3_yaw~=1),'S3_yaw_percent',mean(S3_yaw),'S4_yaw',S4_yaw);
            
save(append(file_name,'_.mat'),"final_result");


