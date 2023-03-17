clear;
close all;
bdclose('all');

addpath('F:\CPS\CPS_git\Benchmarks\SC\RL\agent');

%% setup parameters
T = 35;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts);
isTerminate = 0; 

P_step = 2;         % ref pressure step time
P_init = 90;        % initial ref pressure
P_final = 87;       % final ref pressure
P_terminate = 100;    % error to terminate episode
time_tol = 3;

% mdl = 'SC_PID_eval';
mdl = 'SC_RL_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
% agent = load('SC_DDPG_Agent_9_20.mat');
% agent = load('SC_PPO_Agent_9_20.mat');
agent = load('SC_A2C_Agent_9_20.mat');
agent = agent.agent;

% file_name = 'SC_eval_T_result';
% file_name = 'SC_eval_DDPG_result';
% file_name = 'SC_eval_PPO_result';
file_name = 'SC_eval_A2C_result';

control_switch = 0;         % 0: RL controller, 1: original controller
steady_threshold = 0.8;     % threshold to determine steady-state
steady_time = 5;            % time to determine a steady-state is reached

S1 = ones(sim_num,1);
S2_avg = ones(sim_num,1);
S2_max = ones(sim_num,1);
S3_result = ones(sim_num,1);

for index = 1:sim_num

    sim(mdl);
    error = logsout.getElement('p_error').Values.Data;
    step_num = length(error);                  % number of samples

    % T1: v_ego <= v_set
    error(isinf(error)|isnan(error)) = 0;      % remove +-inf & NaN
    error_last = error(30/Ts:step_num);
    
    S1_violate_num = sum(error_last > 0.5);
    S1(index) = S1_violate_num;
    
    S2_avg(index) = mean(error_last);                  % average error value
    S2_max(index) = max(error_last);                   % maximum error value

    % T2: steady-state
    S3_result(index) = nan;
    for i = 1:step_num
        if sum(error(i:step_num) > steady_threshold) == 0
            S3_result(index) = i*Ts;
            break
        end
    end
    
end

detail_result = struct('S1',S1,'S2_avg',S2_avg, 'S2_max',S2_max, 'S2',S3_result);
save(append(file_name,'_detailed.mat'),"detail_result");

final_result = struct('S1',sum(S1>0),'S2_avg',mean(S2_avg), 'S2_max',mean(S2_max), 'S3_num',sum(~isnan(S3_result)),'S3_time',mean(S3_result(~isnan(S3_result))));
save(append(file_name,'_.mat'),"final_result");