clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.2;           % simulation duration in sec
T = 15;             % simulation time
mdl = 'RocketLander_RL_test';
agentblk = [mdl, '/RL Agent'];

max_x_error = 10;        % maximum position error on x-axis
max_y_error = 10;        % maximum position error on y-axis
max_theta_error = 5;     % maximum orientation error on theta

max_thrust  = 8;         % maximum steering angle in rad
min_thrust  = 0;         % minumun steering angle in rad

sim_num = 0;             % count the num of simulations
change_freq = 50;        % num og episodes to change initial conditions
isTerminate = 0;         % never terminate episode in middle

x0 = [-10;60;0;0;0;0];   % initial position(x,y,theta, dx/dt, dy/dt, dtheta/dt)
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
[~,~,info] = nlmpcmove(planner,x0,u0);
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
x = x0;
u = u0;
k = 1;
references = reshape(info.Xopt',(pPlanner+1)*6,1); % Extract reference signal as column vector.

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([14 1],...
    'LowerLimit', -inf*ones(14,1),...
    'UpperLimit', inf*ones(14,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'states';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([2 1]);
actInfo.LowerLimit = [min_thrust; min_thrust];
actInfo.UpperLimit = [max_thrust; max_thrust];
actInfo.Name = 'Speed; Steer';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo,'UseFastRestart','off');


% Set a custom reset function that randomizes the reference values for the model.
% env.ResetFcn = @(in)localResetFcn(in);

% Fix the random generator seed for reproducibility
rng(0);

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
% create a network to be used as underlying critic approximator
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(32,'Name','CriticStateFC2')];

actionPath = [
    featureInputLayer(2,'Normalization','none','Name','action')
    fullyConnectedLayer(32,'Name','CriticActionFC2')];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(16,'Name','CriticCommonFC1')
    reluLayer('Name','CriticCommonRelu2')
%     fullyConnectedLayer(64,'Name','CriticCommonFC2')
%     reluLayer('Name','CriticCommonRelu3')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC2','add/in2');

% set some options for the critic
criticOptions = rlRepresentationOptions(...
    'LearnRate',1e-3,...
    'GradientThreshold',1,...
    'L2RegularizationFactor',1e-4);

% create the critic based on the network approximator
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

% create a network to be used as underlying actor approximator
actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(64,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(32,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(16,'Name','ActorFC3')
    reluLayer('Name','ActorRelu3')
%     fullyConnectedLayer(64,'Name','ActorFC4')
%     reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(2,'Name','ActorFC5')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',[4; 4],'Bias',[4;4])
%     fullyConnectedLayer(2,'Name','ActorScaling')
    ];

% set some options for the actor
actorOptions = rlRepresentationOptions(...
    'LearnRate',1e-4,...
    'GradientThreshold',1,...
    'L2RegularizationFactor',1e-4);

% create the actor based on the network approximator
actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

% agent options
agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'NumStepsToLookAhead',20,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',64);

% agent creation
agent = rlDDPGAgent(actor,critic,agentOptions);

%% train agent
% stop option: EpisodeCount AverageSteps AverageReward
maxepisodes = 80000; 
maxsteps = ceil(T/Ts);
window_length = 20;

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', maxepisodes, ...
    'MaxStepsPerEpisode', maxsteps, ...
    'ScoreAveragingWindowLength',window_length, ...
    'StopTrainingCriteria','EpisodeCount',...
    'StopTrainingValue', maxepisodes,...
    'Verbose', false,...
    'Plots','training-progress',...
    'UseParallel',false);

doTraining = true;
if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
%     save("preTrainedAgent.mat","agent")
else
%     agent = load('LR_DDPG_Agent.mat');
%     agent = agent.agent;
end
%% simulation & plotting 
% control_switch = 0;
% simulation option
simOpts = rlSimulationOptions('MaxSteps',maxsteps,'StopOnError','on');
% get results
experiences = sim(env,agent,simOpts);
totalReward = sum(experiences.Reward);

% extract signals from simulation results
time = experiences.SimulationInfo.tout;
steps = size(time);
steps = steps(1);
x_error = experiences.SimulationInfo.logsout.getElement('x_error').Values.Data;
y_error = experiences.SimulationInfo.logsout.getElement('y_error').Values.Data;
theta_error = experiences.SimulationInfo.logsout.getElement('theta_error').Values.Data;
dx_error = experiences.SimulationInfo.logsout.getElement('dx_error').Values.Data;
dy_error = experiences.SimulationInfo.logsout.getElement('dy_error').Values.Data;
dtheta_error = experiences.SimulationInfo.logsout.getElement('dtheta_error').Values.Data;
thrust_1 = experiences.SimulationInfo.logsout.getElement('thrust_1').Values.Data;
thrust_1 = reshape(thrust_1, [steps,1]);
thrust_2 = experiences.SimulationInfo.logsout.getElement('thrust_2').Values.Data;
thrust_2 = reshape(thrust_2, [steps,1]);

% plotting
figure(1);
subplot(5,1,1)
plot(time,x_error)
title('Position error in x-axis')
xlabel('time (s)') 
ylabel('Distance (m)') 
grid on

subplot(5,1,2)
plot(time,y_error)
title('Position error in y-axis')
xlabel('time (s)') 
ylabel('Distance (m)') 
grid on

subplot(5,1,3)
plot(time,theta_error)
title('Direction error (theta)')
xlabel('time (s)') 
ylabel('Degree') 
grid on

subplot(5,1,4)
plot(time,dx_error,time,dy_error,time,dtheta_error)
title('Velocity error')
legend('dx','dy','dtheta')
xlabel('time (s)') 
ylabel('Velocity') 
grid on

subplot(5,1,5)
plot(time,thrust_1, time,thrust_2)
title('RL thrust')
legend('thrust 1','thrust 2')
xlabel('time (s)') 
ylabel('Thrust') 
grid on


%% reset function
% function in = localResetFcn(in)
%     x_1 = randi([-50,50]);
%     y_1 = randi([60,100]);
%     x0 = [x_1;y_1;0;0;0;0];
%     in = setVariable(in,'x0', x0);
%     x = x0;
%     in = setVariable(in,'x', x);
%     [~,~,info] = nlmpcmove(planner,x0,u0);
%     references = reshape(info.Xopt',(pPlanner+1)*6,1);
%     in = setVariable(in,'references', references);
% end