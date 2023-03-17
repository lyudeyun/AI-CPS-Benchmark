clear;
close all;
bdclose('all');

addpath('F:\CPS\CPS_git\Benchmarks\\ACC\RL\agent');
% addpath('F:\CPS\CPS_git\Benchmarks\AFC\RL\model');

%% setup parameters
T = 30;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;

t_gap = 1.4;        % time gap to calculate safe distance
D_default = 10;     % default distance
v_set = 30;         % driver-set velocity for ego car
vmax_ego = 50;      % ego car minimum/maximum velocity
vmin_ego = -50;
amin_ego = -3;      % ego car minimum/maximum acceleration
amax_ego = 2;       
x0_lead = 70;       % lead car initial velocity and position
v0_lead = 25;
random_freq = 1;  % time period to randomly switch controllers

x0_ego = 10;        % ego car initial velocity and position
v0_ego = 25;
amin_lead = -1;     % lead car minimum/maximum acceleration
amax_lead = 1;
isTerminate = 0;       % never terminate sim in middle
use_MPC = false;
change_freq = 2;
switch_distance = 5;

% mdl = 'RL_ACC_eval';
% mdl = 'RL_ACC_hybrid_eval';
% mdl = 'RL_ACC_hybrid_random_eval';
mdl = 'RL_ACC_hybrid_average_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
agent = load('ACC_DDPG_Agent_9_22.mat');
% agent = load('ACC_TD3_Agent_9_11.mat');
% agent = load('ACC_SAC_Agent_9_11.mat');
agent = agent.agent;

% file_name = 'ACC_eval_T_result';
% file_name = 'ACC_eval_DDPG_result';
% file_name = 'ACC_eval_TD3_result';
% file_name = 'ACC_eval_SAC_result';
% file_name = 'ACC_eval_hybrid_DDPG_T_result';
% file_name = 'ACC_eval_hybrid_random_DDPG_T_result';
file_name = 'ACC_eval_hybrid_average_DDPG_T_result';

control_switch = 0;         % 0: RL controller, 1: original controller
steady_threshold = 0.2;     % threshold to determine steady-state

S1_result = ones(sim_num,1);
S2_result = ones(sim_num,1);
S3_result = ones(sim_num,1);
S4_result = ones(sim_num,1);
S5_result = ones(sim_num,1);

for index = 1:sim_num

    sim(mdl);
    v_ego = logsout.getElement('v_ego').Values.Data;
    step_num = length(v_ego);                  % number of samples
    v_ego = reshape(v_ego, [step_num,1]);
    d_rel = logsout.getElement('d_rel').Values.Data;
    d_rel = reshape(d_rel, [step_num,1]);
    d_safe = logsout.getElement('d_safe').Values.Data;
    d_safe = reshape(d_safe, [step_num,1]);
    

    % T1: v_ego <= v_set
    v_ego(isinf(v_ego)|isnan(v_ego)) = 0;            % remove +-inf & NaN
    violate_num = sum(v_ego>v_set);       
    S2_result(index) = violate_num;         % number of violations
    
    S1_violate_num = sum(d_rel<d_safe);       
    S1_result(index) = S1_violate_num;

    % T2: steady-state
    steady_num = d_rel > (d_safe + steady_threshold);      % mu under steady-state
    steady_perct = sum(steady_num)/length(steady_num);    % percentage of samples under steady-state
    S3_result(index) = steady_perct;

    % T3: Resilience
    S4_result(index) = nan;
    if steady_perct ~= 1
        unsteady = d_rel < (d_safe + steady_threshold);          % mu beyond steady-state, boolean array
        unsteady_num = sum(unsteady);    % num of times mu beyond from unsteady-state
        recovery = 0;                       % num of times mu recovery from unsteady-state
        unsteady_index = find(unsteady == 1);
        
        for i = 1:length(unsteady_index)
            % how many times mu can recovery from unsteady-sate
            % in following 1 sec (10 samples)
            if unsteady_index(i) < maxsteps-10
                recovery = recovery + any(unsteady(unsteady_index(i):unsteady_index(i)+10)<1);
            end
        end
        recovery_rate = recovery/unsteady_num;          % percentage of recovery
        S4_result(index) = recovery_rate;
    end
    
    % T4: Liveness
    live = v_ego < 1;
    S5_result(index) = sum(live);
    
end

detail_result = struct('S1',S1_result, 'S2',S2_result,'S3',S3_result,'S4',S4_result,'S5',S5_result);
save(append(file_name,'_detailed.mat'),"detail_result");
% 
S4_result = mean(S4_result(~isnan(S4_result)));
S5_result = sum(S5_result~=0);
final_result = struct('S1',sum(S1_result>0), 'S2',sum(S2_result>0),'S3_num',sum(S3_result~=1),'S3_percent',mean(S3_result),'S4',S4_result,'S5',S5_result);
save(append(file_name,'_.mat'),"final_result");