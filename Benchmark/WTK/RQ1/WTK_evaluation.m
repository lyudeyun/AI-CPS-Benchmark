clear;
close all;
bdclose('all');

addpath('F:\CPS\CPS_git\Benchmarks\WTK\RL\agent');

%% setup parameters
T = 24;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts);
isTerminate = 0; 
change_freq = 8;
random_freq = 0.1;

% mdl = 'WTK_PID_eval';
% mdl = 'WTK_RL_eval';
% mdl = 'WTK_RL_hybrid_eval';
% mdl = 'WTK_RL_hybrid_random_eval';
mdl = 'WTK_RL_hybrid_average_eval';


%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
agent = load('WTK_DDPG_Agent_9_15.mat');
% agent = load('WTK_TD3_Agent_9_20.mat');
agent_2 = load('WTK_TD3_Agent_9_20.mat');

agent = agent.agent;
agent_2 = agent_2.agent;

% file_name = 'WTK_eval_T_result';
% file_name = 'WTK_eval_DDPG_result';
% file_name = 'WTK_eval_TD3_result';
% file_name = 'WTK_eval_hybrid_DDPG_TD3_result';
% file_name = 'WTK_eval_hybrid_random_result';
file_name = 'WTK_eval_hybrid_average_result';

steady_threshold = 0.8;     % threshold to determine steady-state
steady_time = 5;            % time to determine a steady-state is reached
switch_error = 1;

S1 = ones(sim_num,1);
S2_avg = ones(sim_num,1);
S2_max = ones(sim_num,1);
S3_result = ones(sim_num,1);

for index = 1:sim_num

    sim(mdl);
    error = logsout.getElement('error').Values.Data;
    step_num = length(error);                  % number of samples

    % T1: v_ego <= v_set
    error(isinf(error)|isnan(error)) = 0;      % remove +-inf & NaN
    steady_error = cat(1,error(40:80),error(120:160),error(200:240));
    
    S2_avg(index) = mean(steady_error);                  % average error value
    S2_max(index) = max(steady_error);                   % maximum error value
    S1_violate_num = sum(steady_error > 0.5);
    S1(index) = S1_violate_num;

    % T2: steady-state
    S3_result(index) = nan;
    start_point = [0,80,160];
    time_to_steady = zeros(3,1);
    
    for i = 1:3
        for j = 1:79
            k = start_point(i);
            if sum(error(k+j:k+79) > steady_threshold) == 0
                 time_to_steady(i) = (j)*Ts;
                break
            end
        end
    end
    S3_result(index) = mean(time_to_steady);
end

detail_result = struct('S1',S1, 'S2_avg',S2_avg, 'S2_max',S2_max, 'S3',S3_result);
save(append(file_name,'_detailed.mat'),"detail_result");

final_result = struct('S1',sum(S1>0),'S2_avg',mean(S2_avg), 'S2_max',mean(S2_max), 'S3_num',sum(~isnan(S3_result)),'S3_time',mean(S3_result(~isnan(S3_result))));
save(append(file_name,'_.mat'),"final_result");