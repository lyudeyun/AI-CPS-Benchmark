clear;
close all;
bdclose('all');
%% add path
if ismac() == 1
    % MacOS
    addpath(genpath('/Users/ldy/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/Users/ldy/git/Benchmarks/LKA/'));
elseif isunix() == 1
    % Linux
    addpath(genpath('/home/ubuntu/git/Benchmarks/tools/Copy_of_S-TaLiRo_new/root/'));
    addpath(genpath('/home/ubuntu/git/Benchmarks/LKA'));
elseif ispc() == 1
    % Windows
    % addpath(genpath(''));
end
%% set parameters
global cur_mdl agent ;
global opt Ts trials max_obj_eval;
global T Ts Vx e1_initial e2_initial;
mdl_name = 'LKA';
tool_name = 'staliro';
mdl_list = {'LKA_T_staliro','LKA_RL_staliro'};
agent_list = {'LKA_DDPG_Agent_9_2','LKA_SAC_Agent_9_18','LKA_PPO_Agent_9_18','LKA_A2C_Agent_9_18'};
% solver list
solver_list = {'SA'};
% model parameters
T = 15;
Ts = 0.1;

m = 1575;   % total vehicle mass (kg)
Iz = 2875;  % yaw moment of inertia (mNs^2)
lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000; % cornering stiffness of front tires (N/rad)
Cr = 33000; % cornering stiffness of rear tires (N/rad)

% Vx = 15;    % longitudinal velocity (m/s)
% e1_initial = 0;   % initial lateral deviation
% e2_initial = 0;   % initial yaw angle

u_min = -0.5;   % maximum steering angle
u_max = 0.5;    % minimum steering angle

line_width = 3.7;   % highway lane width
avg_car_width = 2;  % average car width
max_late_dev = (line_width-avg_car_width)/2-0.1;
max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
terminate_error = 1.5;

rho = 0.001;        %  curvature of the road

time = 0:Ts:T;
Vx = 15;
e1_initial = 0;
e2_initial = 0;
md = getCurvature(Vx,time);
% MPC parameters
PredictionHorizon = 10;     % MPC prediction horizon
% RL parameters

% falsification parameters
trials = 1;            % # of Runs
max_obj_eval = 300;     % Max # of simulations
model = @BlackBoxLKA;
%% Simulation time & Requirements (run " help staliro" for more information)
SP.t0 = 0;
SP.tf = T;
SP.pred(1).str = 'p1';
SP.pred(1).A =  1;
SP.pred(1).b =  0.85;
SP.pred(2).str = 'p2';
SP.pred(2).A =  -1;
SP.pred(2).b =  0.85;
SP.phi = '[]_[0,15] (p1 /\ p2)';

%%
rng('default')

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    % select algorithm
    if strcmp(cur_solver,'SA')
        SA_config();
    elseif strcmp(cur_solver,'SOAR')
        SOAR_config();
    end
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'LKA_T_staliro')
            % falsification
            [results, history] = staliro(model,[10 25;-0.5 0.5;-0.1 0.1],[],[],SP.phi,SP.pred,SP.tf,opt);
            % save
            file_name = [mdl_name,'_','T','_',tool_name,'_',cur_solver,'_result.mat'];
            save(file_name, 'results');
        elseif strcmp(cur_mdl,'LKA_RL_staliro')
            for agent_index = 1:numel(agent_list)
                % load agent
                agent_name = agent_list{1,agent_index};
                agent = load([agent_name,'.mat']);
                agent = agent.agent;
                % falsification
                 [results, history] = staliro(model,[10 25;-0.5 0.5;-0.1 0.1],[],[],SP.phi,SP.pred,SP.tf,opt);
                % save
                new_agent_name = replace(agent_name, mdl_name, '');
                file_name = [mdl_name,new_agent_name,'_',tool_name,'_',cur_solver,'_result.mat'];
                save(file_name, 'results');
            end
        else
            disp("Check your model.");
        end
    end
end

%% SA config
function SA_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();        %(See staliro_options help file)
opt.optimization_solver = 'SA_Taliro';
opt.black_box = 1;
opt.falsification = 1;
opt.SampTime = Ts;
opt.interpolationtype = {'pconst'};   % {'const', 'pconst'}
opt.runs = trials;
opt.optim_params.n_tests = max_obj_eval;
opt.seed = 1;
% opt.varying_cp_times = 1; 
% opt.optim_params.dispStart= 0.9;  ??
% opt.optim_params.dispAdap = 2;
end
%% SOAR config
function SOAR_config()
global opt Ts trials max_obj_eval;
opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_LocalGPs'; % 'SOAR_Taliro_FiniteDiff'; % 'SOAR_Taliro_LocalGPs';
opt.black_box = 1;
opt.falsification = 1;
opt.SampTime = Ts;
opt.interpolationtype = {'pconst'};   % {'const', 'pconst'}
opt.runs = trials;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = max_obj_eval; % Set the length of each marco-replicaitons
opt.seed = 1;               % Set Optimization seed for repeatability

% opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'
% opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
% opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;

% opt.optim_params.crowded_EI_flag = 0;       % Default value is 1,such that croweded EI is used
% opt.optim_params.crowding_threshold = 0.10; % Default value is 0.05
% opt.optim_params.TR_lowpass_thresh = 0.05;  % Default value is 0.25
% opt.optim_params.TR_highpass_thresh = 0.50; % Default value is 0.75
% opt.optim_params.TR_delta = 0.7;            % Default value is 0.75
% opt.optim_params.TR_gamma = 1.3;            % Default value is 1.25
end
