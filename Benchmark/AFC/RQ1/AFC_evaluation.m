clear;
close all;
bdclose('all');

% addpath('F:\CPS\CPS_git\Benchmarks\AFC\RL\agent');
% addpath('F:\CPS\CPS_git\Benchmarks\AFC\RL\model');

%% setup parameters
T = 30;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;

fuel_inj_tol=1.0;
MAF_sensor_tol=1.0;% MAF sensor error factor
AF_sensor_tol=1.0; 
pump_tol=1;
kappa_tol=1;
tau_ww_tol=1;
fault_time=50;
kp=0.04;
ki=0.14;

%The engine speed is constrained to the range [900,1100].
min_Engine_Speed = 900;
max_Engine_Speed = 1100;
%The pedal angle is constrained to the range [8.8,61.1].
min_Pedal_Angle = 8.8;
max_Pedal_Angle = 69;

% input to simulink: take a_lead from a uniform distribution
change_freq_PA = 2;    % input signal change value every change_freq sec
change_freq_ES = 2;

max_mu = 0.03;         % mu from STL
mu_tol = 0.1;          % mu value to terminate the episode
time_tol = 5;          % time tolerence, to avoid the initial large mu value
isTerminate = 0;       % never terminate sim in middle
control_switch = 0;         % 0: RL controller, 1: original controller
switch_mu = 0.12;      % hybrid: if mu > threshold -> switch to RL controller
random_freq = 1;

% mdl = 'AFC_RL_DDPG_eval';
% mdl = 'AFC_RL_eval';
% mdl = 'AFC_hybrid_eval';
% mdl = 'AFC_hybrid_AI_eval';
% mdl = 'AFC_NN_6_20_eval';
% mdl = 'AFC_NN_5_10_eval';
% mdl = 'AFC_NN_4_10_eval';
% mdl = 'AFC_NN_4_15_2_eval';
% mdl = 'AFC_NN_3_20_eval';
% mdl = 'AFC_NN_3_15_1_eval';
% mdl = 'AFC_hybrid_random_eval';
mdl = 'AFC_hybrid_average_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
agent = load('AFC_DDPG_Agent_8_28.mat');
% agent = load('AFC_PPO_Agent_9_18.mat');
% agent = load('AFC_A2C_Agent_9_18.mat');
agent = agent.agent;

% file_name = 'AFC_eval_T_result';
% file_name = 'AFC_eval_DDPG_result';
% file_name = 'AFC_eval_PPO_result';
% file_name = 'AFC_eval_A2C_result';
% file_name = 'AFC_eval_hybrid_T_DDPG_result';
% file_name = 'AFC_eval_hybrid_NN_4_15_2_DDPG_result';
% file_name = 'AFC_eval_NN_6_20_result';
% file_name = 'AFC_eval_NN_5_10_result';
% file_name = 'AFC_eval_NN_4_10_result';
% file_name = 'AFC_eval_NN_4_15_2_result';
% file_name = 'AFC_eval_NN_3_20_result';
% file_name = 'AFC_eval_NN_3_15_1_result';
% file_name = 'AFC_eval_hybrid_random_0.1_DDPG_T_result';
% file_name = 'AFC_eval_hybrid_random_1_DDPG_T_result';
file_name = 'AFC_eval_hybrid_average_DDPG_T_result';

steady_threshold = 0.1;    % threshold to determine steady-state

S1_result = ones(sim_num,1);
S2_avg = ones(sim_num,1);
S2_max = ones(sim_num,1);
S3_result = ones(sim_num,1);
S4_result = ones(sim_num,1);

for index = 1:sim_num

    sim(mdl);
    mu = logsout.getElement('sampled_mu').Values.Data;
    step_num = length(mu);                  % number of samples

    % T1: maximum mu violation
    mu(isinf(mu)|isnan(mu)) = 0;            % remove +-inf & NaN
    mu_violate = mu(mu > 0.2);              % get samples violate mu threshold
    violate_num = length(mu_violate);       % number of violations
    avg_viol_mu = mean(mu((mu-0.2)>0));     % average of violation over 0.2
    
    S2_avg(index) = mean(mu);               % average mu value
    S2_max(index) = max(mu);                % maximum mu value
    
    S1_violate_num = sum(mu > 0.2);
    S1_result(index) = S1_violate_num;

    % T2: steady-state
    steady_mu = (mu-steady_threshold)<0;                % mu under steady-state
    steady_perct = sum(steady_mu)/length(steady_mu);    % percentage of samples under steady-state
    S3_result(index) = steady_perct;

    % T3: Resilience
    S4_result(index) = nan;
    if steady_perct ~= 1
        unsteady_mu = (mu-steady_threshold)>0;          % mu beyond steady-state, boolean array
        unsteady_num = sum(unsteady_mu);                % num of times mu beyond from unsteady-state
        recovery = 0;                                   % num of times mu recovery from unsteady-state
        unsteady_index = find(unsteady_mu == 1);
        
        for i = 1:length(unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if unsteady_index(i) < maxsteps-10
                recovery = recovery + any(unsteady_mu(unsteady_index(i):unsteady_index(i)+10)<1);
            end
        end
        recovery_rate = recovery/unsteady_num;          % percentage of recovery
        S4_result(index) = recovery_rate;
    end
end

% detail_result = struct('S1',S1_result, 'S2_avg',S2_avg, 'S2_max',S2_max, 'S3_percent',S3_result,'S4',S4_result);
% save(append(file_name,'_detailed.mat'),"detail_result");
% 
% S4_result = mean(S4_result(~isnan(S4_result)));
% final_result = struct('S1',sum(S1_result>0), 'S2_avg',mean(S2_avg), 'S2_max',mean(S2_max), 'S3_percent',mean(S3_result), 'S3_num',sum(S3_result~=1),  'S4',S4_result);
% save(append(file_name,'_.mat'),"final_result");




