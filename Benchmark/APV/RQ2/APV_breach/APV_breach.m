clear all;
close all;

% addpath(genpath('/home/ubuntu/git/Benchmarks/tools/breach'));
% addpath(genpath('/home/ubuntu/git/Benchmarks/APV'));

addpath(genpath('/Users/ldy/git/Benchmarks/tools/breach'));
addpath(genpath('/Users/ldy/git/Benchmarks/APV/'));

InitBreach;
tool_name = 'breach';
% model name
mdl_name = 'APV';
%%%%%%%%%%%%%%%%%%%% current model
mdl_list = {'APV_RL', 'APV_T'};
%%%%%%%%%%%%%%%%%%%% agent list
agent_list = {'APV_TD3_Agent_9_27','APV_DDPG_Agent_9_20','APV_DDPG_Agent_9_19'};
%%%%%%%%%%%%%%%%%%%%
% model parameters
global T Ts xpos ypos;

Ts = 0.1;           % simulation duration in sec
T = 12;             % simulation time


max_x_error = 3;        % maximum position error on x-axis
max_y_error = 3;        % maximum position error on y-axis
max_theta_error = 2;    % maximum orientation error on theta

max_steer = 0.785;      % maximum steering angle in rad
min_steer = -0.785;     % minumun steering angle in rad

max_vel = 6;            % maximum velocity in m/s
min_vel = -6;           % minimum velocity in m/s

x_error_offset = 4;     % x-axis error reward offset on denominator
y_error_offset = 4;     % x-axis error reward offset on denominator
theta_error_offset = 3; % theta-axis error reward offset on denominator

vdims = vehicleDimensions;
egoWheelbase = vdims.Wheelbase;
distToCenter = 0.5*egoWheelbase;

xpos = 4;
ypos = 12;

% Ego initial pose: x(m), y(m) and yaw angle (rad)
egoInitialPose = [xpos,ypos,0];

parkNorth = true;
if parkNorth
    egoTargetPose = [36,45,pi/2];
else
    egoTargetPose = [27.2,4.7,-pi/2];
end

costmap = helperSLCreateCostmap();
centerToFront = distToCenter;
centerToRear = distToCenter;
helperSLCreateUtilityBus;
costmapStruct = helperSLCreateUtilityStruct(costmap);

if parkNorth
    midPoint = [4,34,pi/2];
else
    midPoint = [27,12,0];
end

% Prediction horizon
p = 100;
% Control horizon
c = 100;
% Weight matrices for terminal cost
Qt = 0.5*diag([10 5 20]); 
Rt = 0.1*diag([1 2]);
% Weight matrices for tracking cost
if parkNorth
    Qp = 1e-6*diag([2 2 0]);
    Rp = 1e-4*diag([1 15]);
else
    Qp = 0*diag([2 2 0]);
    Rp = 1e-2*diag([1 5]);
end
% Safety distance to obstacles (m)
safetyDistance = 0.1;
% Maximum iteration number
maxIter = 70;
% Disable message display
mpcverbosity('off');

% Create the NLMPC controller using the specified parameters.
[nlobj,opt,paras] = createMPCForParkingValet(p,c,Ts,egoInitialPose,egoTargetPose,...
    maxIter,Qp,Rp,Qt,Rt,distToCenter,safetyDistance,midPoint);

% Set the initial conditions for the ego vehicle.
x0 = egoInitialPose';
u0 = [0;0];

% Generate the reference trajectory using the nlmpcmove function.
tic;
[mv,nloptions,info] = nlmpcmove(nlobj,x0,u0,[],[],opt);

timeVal = toc;
xRef = info.Xopt;
uRef = info.MVopt;

analyzeParkingValetResults(nlobj,info,egoTargetPose,Qp,Rp,Qt,Rt,...
    distToCenter,safetyDistance,timeVal)

% set the simulation duration and update the reference trajectory based on the duration
Duration = T;
Tsteps = Duration/Ts;
Xref = [xRef(2:p+1,:);repmat(xRef(end,:),Tsteps-p,1)];

% Create an NLMPC controller with a tracking prediction horizon (pTracking) of 10.
pTracking = 10;
nlobjTracking = createMPCForTrackingParkingValet(pTracking,Xref);

% MPC parameters
PredictionHorizon = 10;     % MPC prediction horizon
% RL parameters

% falsification parameters
trials = 1;
solver_list = {'cmaes','GNM'}; % default: global_nelder_mead
%%%%%%%%%%%%%%%%%%%%
phi1 = STL_Formula('phi1', 'alw_[0,12](x_error[t] < 1 and y_error[t] < 1)');
%%%%%%%%%%%%%%%%%%%% phi list
spec_list = {phi1};

for solver_index = 1:numel(solver_list)
    cur_solver = solver_list{1, solver_index};
    for mdl_index = 1:numel(mdl_list)
        cur_mdl = mdl_list{1,mdl_index};
        if strcmp(cur_mdl,'APV_T')
            for spec_index = 1: numel(spec_list)
                % falsification
                [obj_best,num_sim,time] = breach_fal(cur_mdl, 0, spec_list{1,spec_index},trials,cur_solver);
                % save
                file_name = [mdl_name,'_',num2str(spec_index),'_','T','_',tool_name,'_',cur_solver,'_Result.mat'];
                save(file_name, 'num_sim', 'time', 'obj_best');
            end
            
        elseif strcmp(cur_mdl,'APV_RL')
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
global T Ts xpos ypos;

sg = APV_signal_gen(model,agent);
Br = BreachSignalGen(sg);
Br.SetTime(0:Ts:T);

Br.SetParamRanges({'xpos','ypos'}, [3 5;11 13]);

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








