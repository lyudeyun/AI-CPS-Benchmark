clear;
close all;
bdclose('all');
%%
addpath('F:\CPS\CPS_git\Benchmarks\WT');
addpath('F:\CPS\CPS_git\Benchmarks\WT\wind');
addpath('F:\CPS\CPS_git\Benchmarks\WT\tools');
addpath('F:\CPS\CPS_git\Benchmarks\WT\wafo\wafo');
addpath('F:\CPS\CPS_git\Benchmarks\benchmark_RL\WT_RL');

%% setup parameters
T = 630;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;
isTerminate = 0;

SimplifiedTurbine_Config;
% addpath(config.wafo_path)
load('ClassA.mat')
load('ClassA_config.mat')
load('aeromaps3.mat');
Parameter.InitialConditions = load('InitialConditions');
% remove all unnecessary fields (otherwise Simulink will throw an error)
cT_modelrm = rmfield(cT_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});
cP_modelrm = rmfield(cP_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});

% initialize WAFO
initwafo 
iBin = find(URefVector==Parameter.URef);
iRandSeed = 1;
config.iBin                         = iBin;
config.iRandSeed                    = iRandSeed;
Parameter.v0                        = v0_cell{iBin,iRandSeed};
Parameter.v0.signals.values         = Parameter.v0.signals.values';
Parameter.TMax                      = v0_cell{iBin,iRandSeed}.time(end);
config.WindFieldName                = FileNames{iBin,iRandSeed};
% Time
Parameter.Time.TMax                 = T;              % [s]       duration of simulation
Parameter.Time.dt                   = Ts;             % [s]       time step of simulation
Parameter.Time.cut_in               = 30;
Parameter.Time.cut_out              = Parameter.Time.TMax;
Parameter.v0_0 = Parameter.v0.signals.values(1);
Parameter = SimplifiedTurbine_ParamterFile(Parameter);
max_vin = 16;       % maximum input wind speed
min_vin = 8;
change_freq = 60;   % input signal change value every change_freq sec

max_theta = Parameter.Pitch.Max;   % maximum angle controller output
min_theta = Parameter.Pitch.Min;
max_torque = Parameter.VSControl.VS_MaxTq;
min_torque = 2.1e+4;
omega_term_error = 60;   % omega error to terminate episode
theta_term_error = 16;  % theta error to terminate episode
time_tol = 180;           % error exemption time

% mdl = 'WT_T_eval';
mdl = 'WT_RL_eval';

%% simulation & plotting 
% simulation option
sim_num = 100;               % simulation numbers

% load agent 
agent = load('WT_DDPG_Agent_9_27.mat');
agent = agent.agent;

% file_name = 'WT_eval_T_result';
file_name = 'WT_eval_DDPG_result';

control_switch = 0;         % 0: RL controller, 1: original controller
steady_threshold = 0.2;    % threshold to determine steady-state

T1_result = cell(sim_num,1);
T2_result = cell(sim_num,1);
T3_result = ones(sim_num,1);

% steady-state thresholds for 4 outputs
theta_thres = 14;
omega_thres = 14;
mg_d_thres = [20500, 47000];
theta_error_thres = 1.6;

for index = 1:sim_num

    sim(mdl);
    theta_error = logsout.getElement('theta_error').Values.Data;
    mg_d = logsout.getElement('mg_d').Values.Data;
    omega = logsout.getElement('omege').Values.Data;
    theta = logsout.getElement('theta').Values.Data;

    % T1: soft-safety
    thete_viol = sum(theta>14.2);           % violation num of theta
    mg_d_viol = sum(mg_d>47500) + sum(mg_d<21000);
    omega_viol = sum(omega>14.3);
    theta_error_avg = mean(theta_error);
    theta_error_max = max(theta_error);
    
    T1_result{index} = [thete_viol,mg_d_viol,omega_viol,theta_error_avg,theta_error_max];       
    
    % T2: steady-state
    thete_steady = sum(theta<theta_thres)/maxsteps;
    omega_steady = sum(omega<omega_thres)/maxsteps;
    mg_d_steady = 1- (sum(mg_d<mg_d_thres(1)) + sum(mg_d>mg_d_thres(2)))/maxsteps;
    theta_error_steady = sum(theta_error<theta_error_thres)/maxsteps;
    T2_result{index} = [thete_steady, omega_steady,mg_d_steady,theta_error_steady];

    % T3: Resilience
    T3_result(index) = nan;
    
    if theta_error_steady ~= 1
        unsteady = theta_error > theta_error_thres;          % mu beyond steady-state, boolean array
        unsteady_num = sum(unsteady);    % num of times mu beyond from unsteady-state
        recovery = 0;                       % num of times mu recovery from unsteady-state
        unsteady_index = find(unsteady == 1);
        
        for i = 1:length(unsteady_index)
            % how many times system recovers from unsteady-sate
            % in following 5 sec (50 samples)
            if unsteady_index(i) < maxsteps-50
                recovery = recovery + any(unsteady(unsteady_index(i):unsteady_index(i)+50)<1);
            end
        end
        recovery_rate = recovery/unsteady_num;          % percentage of recovery
        T3_result(index) = recovery_rate;
    end

    
end

T1_result = cell2mat(T1_result);
T2_result = cell2mat(T2_result);

detail_result = struct('T1',T1_result,'T2',T2_result,'T3',T3_result);
save(append(file_name,'_detailed.mat'),"detail_result");

T1_final = [sum(T1_result(:,1)>0); sum(T1_result(:,2)>0);sum(T1_result(:,3)>0);mean(T1_result(:,4));mean(T1_result(:,5))];
T2_final = [mean(T2_result(:,1));mean(T2_result(:,2));mean(T2_result(:,3));mean(T2_result(:,4))];

final_result = struct('T1',T1_final,'T2',T2_final,'T3',mean(T3_result));
save(append(file_name,'_.mat'),"final_result");