clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
    addpath(genpath('/Users/ldy/git/Benchmarks/LR/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/LR'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
tool_name = 'breach';
% model name
mdl_name = 'LR';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'LR_RL','LR_T'};
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'LR_DDPG_Agent_10_4','LR_DDPG_Agent_10_5','LR_DDPG_single_Agent_10_4'};
hybrid_agent_list = {};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts xpos0 references;

T = 15;             % simulation time
Ts = 0.2;           % simulation duration in sec

max_x_error = 10;        % maximum position error on x-axis
max_y_error = 10;        % maximum position error on y-axis
max_theta_error = 5;     % maximum orientation error on theta

max_thrust  = 8;         % maximum steering angle in rad
min_thrust  = 0;         % minumun steering angle in rad

xpos0 = -5;
% x0 = [xpos0;ypos0;0;0;0;0];   % initial position(x,y,theta, dx/dt, dy/dt, dtheta/dt)
u0 = [0;0];              % target position (x,y)
pPlanner = 50;
planner = nlmpcMultistage(pPlanner,6,2);
planner.Ts = Ts;
planner.Model.StateFcn = 'RocketStateFcn';
planner.Model.StateJacFcn = 'RocketStateJacobianFcn';
planner.MV(1).Min = 0;
planner.MV(1).Max = 8;
planner.MV(2).Min = 0;
planner.MV(2).Max = 8;
planner.States(2).Min = 10;
for ct=1:pPlanner
    planner.Stages(ct).CostFcn = 'RocketPlannerCostFcn';
    planner.Stages(ct).CostJacFcn = 'RocketPlannerCostGradientFcn';
end

planner.Model.TerminalState = [0;10;0;0;0;0];
planner.Optimization.SolverOptions.MaxIterations = 1000;
[~,~,info] = nlmpcmove(planner,[xpos0;60;0;0;0;0],u0);
pLander = 10;
lander = nlmpcMultistage(pLander,6,2);
lander.Ts = Ts;
lander.Model.StateFcn = 'RocketStateFcn';
lander.Model.StateJacFcn = 'RocketStateJacobianFcn';
lander.MV(1).Min = 0;
lander.MV(1).Max = 8;
lander.MV(2).Min = 0;
lander.MV(2).Max = 8;
lander.States(2).Min = 10;
for ct=1:pLander+1
    lander.Stages(ct).CostFcn = 'RocketLanderCostFcn';
    lander.Stages(ct).CostJacFcn = 'RocketLanderCostGradientFcn';
    lander.Stages(ct).ParameterLength = 6;
end
lander.UseMVRate = true;
% x = x0;
% u = u0;
% k = 1;
references = reshape(info.Xopt',(pPlanner+1)*6,1); % Extract reference signal as column vector.
% RL parameters
isTerminate = 0;         % never terminate episode in middle
% hybrid control parameters
% falsification parameters
trials = 30;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1', 'alw_[14.8,15](x_error[t] < 1 and y_error[t] < 1)');

%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

InitBreach;
for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for spec_index = 1:numel(spec_list)
        cur_spec = spec_list{1,spec_index};
        for mdl_index = 1:numel(mdl_list)
            cur_mdl = mdl_list{1,mdl_index};
            if strcmp(cur_mdl,'LR_T')
                % falsification
                cur_spec = spec_list{1,spec_index};
                [obj_best,num_sim,time] = breach_fal(cur_mdl,0 ,cur_spec,trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
                
            elseif strcmp(cur_mdl,'LR_RL')
                for agent_index = 1:numel(agent_list)
                    % load agent
                    cur_agent = agent_list{1,agent_index};
                    agent = load([cur_agent,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,agent, cur_spec,trials,cur_solver);
                    % save
                    new_agent_name = replace(cur_agent, mdl_name, '');
                    file_name = [mdl_name,'_',num2str(spec_index),new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            elseif contains(cur_mdl, 'hybrid')
                for hybrid_agent_index = 1:numel(hybrid_agent_list)
                    % load agent for hybrid control
                    cur_hybrid_agent = hybrid_agent_list{1,hybrid_agent_index};
                    agent = load([cur_hybrid_agent,'.mat']);
                    agent = agent.agent;
                    % falsification
                    [obj_best,num_sim,time] = breach_fal(cur_mdl,agent, cur_spec,trials,cur_solver);
                    % save
                    new_hybrid_agent_name = replace(cur_hybrid_agent, mdl_name, '');
                    file_name = [cur_mdl,'_',num2str(spec_index),new_hybrid_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                    save(file_name, 'num_sim', 'time', 'obj_best');
                end
            else
                disp("Check your model.");
            end
        end
    end
end

function [obj_best,num_sim,time] = breach_fal(model,agent, phi, trials, solver_name)
global T Ts  xpos0   references;

sg = LR_signal_gen(model,agent);
Br = BreachSignalGen(sg);
Br.SetTime(0:Ts:T);


Br.SetParamRanges( {'xpos0'}, [-5 5]);

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

