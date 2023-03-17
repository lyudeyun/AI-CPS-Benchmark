clear all;
close all;

addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
addpath(genpath('/home/ubuntu/git/Benchmarks/WT'));

% addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
% addpath(genpath('/Users/ldy/git/Benchmarks/WT/'));

InitBreach;
tool_name = 'breach';
% model name
mdl_name = 'WT';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'WT_T','WT_RL_torque'};
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'WT_TD3_Agent_10_6'};  % 'WT_DDPG_Agent_9_27'
%%%%%%%%%%%%%%%%%%%%
global min_vin max_vin;
% model parameters
SimplifiedTurbine_Config;

addpath('tools/')
addpath('wind/')
addpath(config.wafo_path)

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
Parameter.Time.TMax                 = 630;            % [s]       duration of simulation
Parameter.Time.dt                   = 0.1;            % [s]       time step of simulation
Parameter.Time.cut_in               = 30;
Parameter.Time.cut_out              = Parameter.Time.TMax;
Parameter.v0_0 = Parameter.v0.signals.values(1);

Parameter = SimplifiedTurbine_ParamterFile(Parameter);

min_vin = 8;
max_vin = 16;       % maximum input wind speed
% RL parameters
max_theta = Parameter.Pitch.Max;   % maximum angle controller output
min_theta = Parameter.Pitch.Min;
max_torque = Parameter.VSControl.VS_MaxTq;
min_torque = 2.1e+4;

omega_term_error = 60;   % omega error to terminate episode
theta_term_error = 16;  % theta error to terminate episode
time_tol = 180;           % error exemption time

% falsification parameters
trials = 1;
solver_list = {'cmaes', 'GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1','always_[30, 630](Out6[t] <= 14.2)');
phi2 = STL_Formula('phi2','always_[30, 630]((Out4[t] >= 21000) and (Out4[t] <= 47500))');
phi3 = STL_Formula('phi3','always_[30, 630](Out5[t] <= 14.3)');
phi4 = STL_Formula('phi4','always_[30, 630](ev_[0,5](abs(Out6[t] - Out1[t]) <= 1.6))');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1,phi2,phi3,phi4};

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'WT_T')
            for spec_index = 1: numel(spec_list)
                % falsification
                [obj_best,num_sim,time] = breach_fal(cur_mdl,spec_list{1,spec_index},trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
            end
            
        elseif strcmp(cur_mdl,'WT_RL_torque')
            for agent_index = 1:numel(agent_list)
                for spec_index = 1:numel(spec_list)
                    % load agent
                    agent_name = agent_list{1,agent_index};
                    agent = load([agent_name,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,spec_list{1,spec_index},trials,cur_solver);
                    % save
                    new_agent_name = replace(agent_name, mdl_name, '');
                    file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            end
        else
            disp("Check your model.");
        end
    end
end

function [obj_best,num_sim,time] = breach_fal(mdl, phi, trials, solver_name)
global min_vin max_vin;

Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:0.1:630;

% 70 * 9 = 630
cp_num = 9;
input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    ws_sig = strcat('vin_u', num2str(cpi));
    Br.SetParamRanges({ws_sig},[min_vin max_vin]);
end


falsified = [];
time = [];
obj_best = [];
num_sim = [];
for n = 1:trials
    falsif_pb = FalsificationProblem(Br,phi);
    falsif_pb.max_obj_eval = 1;
    if strcmp(solver_name, 'cmaes')
        falsif_pb.setup_solver(solver_name);
    elseif strcmp(solver_name, 'GNM')
    end
    falsif_pb.solve();
    if falsif_pb.obj_best < 0
        falsified = [falsified;1];
    else
        falsified = [falsified;0];
    end
    num_sim = [num_sim;falsif_pb.nb_obj_eval];
    time = [time;falsif_pb.time_spent];
    obj_best = [obj_best;falsif_pb.obj_best];
end
end    



 



