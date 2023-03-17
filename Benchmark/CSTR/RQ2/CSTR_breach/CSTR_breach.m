clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
    addpath(genpath('/Users/ldy/git/Benchmarks/CSTR/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/CSTR'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% initial config
global T Ts nlobj dist_min dist_max out_min out_max agent;
InitBreach;
tool_name = 'breach';
mdl_name = 'CSTR';
% general
mdl_list = {'CSTR_MPC'};
agent_list = {'CSTR_DDPG_Agent_9_23','CSTR_DDPG_Agent_9_24','CSTR_TD3_Agent_9_24','CSTR_PPO_Agent_9_24','CSTR_A2C_Agent_9_24'};
% hybrid
hybrid_mdl_list = {'CSTR_hybrid_random_01','CSTR_hybrid_random_1' ,'CSTR_hybrid','CSTR_hybrid_average'};
hybrid_agent_list = {'CSTR_TD3_Agent_9_24'};
% model parameters
T = 30;
Ts = 0.1;
nlobj = nlmpc(3, 1,'MV',3,'MD',[1 2],'UD',4);
% The prediction model sample time is the same as the controller sample
% time.
nlobj.Ts = Ts;
% To reduce computational effort, use a short prediction horizon of 3
% seconds (6 steps). Also, to increase robustness, use block moves in the
% control horizon.
nlobj.PredictionHorizon = 6;
nlobj.ControlHorizon = [2 2 2];

% Since the magnitude of the MV is of order 300 and that of the OV is order
% 1, scale the MV to make them compatible such that default tuning
% weights can be used.
nlobj.MV(1).ScaleFactor = 300;

% Constrain the coolant temperature adjustment rate, which can only
% increase or decrease 5 degrees between two successive intervals.
nlobj.MV(1).RateMin = -5;
nlobj.MV(1).RateMax = 5;

% It is good practice to scale the state to be of unit order. Doing so has
% no effect on the control strategy, but it can improve numerical behavior.
nlobj.States(1).ScaleFactor = 300;
nlobj.States(2).ScaleFactor = 10;

% Specify the nonlinear state and output functions.
nlobj.Model.StateFcn = 'exocstrStateFcnCT';
nlobj.Model.OutputFcn = 'exocstrOutputFcn';

x0 = [311.2639; 8.5698; 0];     % initial concentration
u0 = [10; 298.15; 298.15];
dist_min = 0;
dist_max= 0.5;
out_min = 300;
out_max = 500;
% RL parameters
use_MPC = false;
% hybrid parameters
control_switch = 0;         % 0: RL controller, 1: original controller
steady_threshold = 0.8;     % threshold to determine steady-state
steady_time = 5;            % time to determine a steady-state is reached
switch_error = 0.1;
% random_freq = 0.1;
isTerminate = 0;
% falsification parameters
trials = 30;
max_sim = 300;
cp_num = 6;
solver_list = {'GNM'}; % default: global_nelder_mead
%
phi1 = STL_Formula('phi1','alw_[25,30](abs(error[t]) < 0.5)');
% phi list
spec_list = {phi1};

hybrid_model(hybrid_mdl_list, hybrid_agent_list, solver_list, spec_list);



for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for spec_index =1: numel(spec_list)
        cur_spec = spec_list(1,spec_index);
        for mdl_index = 1:numel(mdl_list)
            cur_mdl = mdl_list{1,mdl_index};
            if strcmp(cur_mdl,'CSTR_MPC')
                for spec_index = 1: numel(spec_list)
                    % falsification
                    [obj_best,num_sim,time] =  breach_fal(cur_mdl, cur_spec, cur_solver, cp_num, trials, max_sim);
                    % save
                    file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_Result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
                
            elseif strcmp(cur_mdl,'CSTR_RL')
                for agent_index = 1:numel(agent_list)
                    for spec_index = 1: numel(spec_list)
                        % load agent
                        agent_name = agent_list{1,agent_index};
                        agent = load([agent_name,'.mat']);
                        agent = agent.agent;
                        % falsification
                       [obj_best,num_sim,time] = breach_fal(cur_mdl, cur_spec, cur_solver, cp_num, trials, max_sim);
                        % save
                        new_agent_name = replace(agent_name, mdl_name, '');
                        file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_Result.mat'];
                        save(file_name, 'num_sim', 'time', 'obj_best');
                    end
                end
            else
                disp("Check your model.");
            end
        end
    end
end

function hybrid_model(hybrid_mdl_list, hybrid_agent_list, solver_list, spec_list)
global agent tool_name;
    for solver_index = 1: numel(solver_list)
        for spec_index =1: numel(spec_list)
            for hybrid_mdl_index = 1: numel(hybrid_mdl_list)
                for hybrid_agent_index = 1: numel(hybrid_agent_list)
                   
                    cur_solver = solver_list{1, solver_index};
                    cur_spec = spec_list{1, spec_index};
                    cur_hybrid_mdl = hybrid_mdl_list{1, hybrid_mdl_index};
                    cur_hybrid_agent = hybrid_agent_list{1, hybrid_agent_index};
                    % load agent for hybrid control
                    agent = load([cur_hybrid_agent,'.mat']);
                    agent = agent.agent;
                     % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_hybrid_mdl, cur_spec, cur_solver, 6, 30, 300);
                    % save result
                    file_name = [cur_hybrid_mdl,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            end
        end
end
end

function [obj_best,num_sim,time] = breach_fal(mdl, phi, solver, cp_num, trials, max_sim)
global T Ts dist_min dist_max;

Br = BreachSimulinkSystem(mdl);
Br.Sys.tspan = 0:Ts:T;

input_gen.type = 'UniStep';
input_gen.cp = cp_num;

Br.SetInputGen(input_gen);

for cpi = 0:input_gen.cp-1
    disturb_sig = strcat('disturb_u', num2str(cpi));
    Br.SetParamRanges({disturb_sig},[dist_min dist_max]);
end

falsified = [];
time = [];
obj_best = [];
num_sim = [];
for n = 1:trials
    
    disp(n);
    disp(mdl);
    disp(solver);
    
    
    falsif_pb = FalsificationProblem(Br,phi);
    falsif_pb.max_obj_eval = max_sim;
    if strcmp(solver, 'cmaes')
        falsif_pb.setup_solver(solver);
    elseif strcmp(solver, 'GNM')
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