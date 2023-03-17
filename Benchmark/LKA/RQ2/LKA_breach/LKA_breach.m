clear all;
close all;

% addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
% addpath(genpath('/home/ubuntu/git/Benchmarks/LKA'));

addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
addpath(genpath('/Users/ldy/git/Benchmarks/LKA/'));

InitBreach;
tool_name = 'breach';
% model name
mdl_name = 'LKA';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'LKA_RL'};
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'LKA_DDPG_Agent_9_2','LKA_SAC_Agent_9_18','LKA_PPO_Agent_9_18','LKA_A2C_Agent_9_18'};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts Vx e1_initial e2_initial;
T = 15;
Ts = 0.1;

m = 1575;   % total vehicle mass (kg)
Iz = 2875;  % yaw moment of inertia (mNs^2)
lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000; % cornering stiffness of front tires (N/rad)
Cr = 33000; % cornering stiffness of rear tires (N/rad)

Vx = 15;    % longitudinal velocity (m/s)
e1_initial = 0;   % initial lateral deviation
e2_initial = 0;   % initial yaw angle

u_min = -0.5;   % maximum steering angle
u_max = 0.5;    % minimum steering angle

line_width = 3.7;   % highway lane width
avg_car_width = 2;  % average car width
max_late_dev = (line_width-avg_car_width)/2-0.1;
max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
terminate_error = 1.5;

rho = 0.001;        %  curvature of the road

time = 0:Ts:T;
md = getCurvature(Vx,time);
% MPC parameters
PredictionHorizon = 10;     % MPC prediction horizon
% RL parameters

% falsification parameters
trials = 10;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1', 'alw_[0,15](abs(lateral_deviation[t]) < 0.85)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'LKA_T')
            for spec_index = 1: numel(spec_list)
                % falsification
                [obj_best,num_sim,time] = breach_fal(cur_mdl, 0, spec_list{1,spec_index},trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_Result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
            end
            
        elseif strcmp(cur_mdl,'LKA_RL')
            for agent_index = 1:numel(agent_list)
                for spec_index = 1:numel(spec_list)
                    % load agent
                    agent_name = agent_list{1,agent_index};
                    agent = load([agent_name,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,agent,spec_list{1,spec_index},trials,cur_solver);
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

function [obj_best,num_sim,time] = breach_fal(model,agent, phi, trials, solver_name)
global T Ts Vx e1_initial e2_initial;

sg = LKA_signal_gen(model,agent);
Br = BreachSignalGen(sg);
Br.SetTime(0:Ts:T);

Br.SetParamRanges( {'Vx', 'e1_initial','e2_initial'}, [10 25;-0.5 0.5;-0.1 0.1]);

falsified = [];
time = [];
obj_best = [];
num_sim = [];
for n = 1:trials
    falsif_pb = FalsificationProblem(Br,phi);
    falsif_pb.max_obj_eval = 300;
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

