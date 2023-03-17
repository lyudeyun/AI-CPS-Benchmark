clear;
close all;
bdclose('all');

%% setup parameters
T = 30;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts);
isTerminate = 0; 

x0 = [311.2639; 8.5698; 0];     % initial concentration
u0 = [10; 298.15; 298.15];
out_max = 500;
out_min = 300;
change_freq = 5;
nlobj = nlmpc(3, 1,'MV',3,'MD',[1 2],'UD',4);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 6; 
nlobj.ControlHorizon = [2 2 2];
nlobj.MV(1).ScaleFactor = 300;
nlobj.MV(1).RateMin = -5;
nlobj.MV(1).RateMax = 5;
nlobj.States(1).ScaleFactor = 300;
nlobj.States(2).ScaleFactor = 10;
nlobj.Model.StateFcn = 'exocstrStateFcnCT';
nlobj.Model.OutputFcn = 'exocstrOutputFcn';

random_freq = 1;

mdl = 'CSTR_MPC_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
% agent = load('CSTR_DDPG_Agent_9_24.mat');
% agent = load('CSTR_PPO_Agent_9_24.mat');
% agent = load('CSTR_A2C_Agent_9_24.mat');
agent = load('CSTR_TD3_Agent_9_24.mat');
agent = agent.agent;

% file_name = 'CSTR_eval_T_result';
% file_name = 'CSTR_eval_DDPG_result';
% file_name = 'CSTR_eval_PPO_result';
% file_name = 'CSTR_eval_A2C_result';
% file_name = 'CSTR_eval_TD3_result';
file_name = 'CSTR_eval_hybrid_TD3_T_result';
% file_name = 'CSTR_eval_hybrid_random_TD3_T_result';
% file_name = 'CSTR_eval_hybrid_average_TD3_T_result';

control_switch = 0;         % 0: RL controller, 1: original controller
steady_threshold = 0.8;     % threshold to determine steady-state
steady_time = 5;            % time to determine a steady-state is reached
switch_error = 0.1;

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
    error_last = error(25/Ts:step_num);
    
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

detail_result = struct('S1',S1,'S2_avg',S2_avg, 'S2_max',S2_max, 'S3',S3_result);
save(append(file_name,'_detailed.mat'),"detail_result");

final_result = struct('S1',sum(S1>0),'S2_avg',mean(S2_avg), 'S2_max',mean(S2_max), 'S3_num',sum(~isnan(S3_result)),'S3_time',mean(S3_result(~isnan(S3_result))));
save(append(file_name,'_.mat'),"final_result");