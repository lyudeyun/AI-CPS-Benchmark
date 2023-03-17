clear;
close all;
bdclose('all');

%% setup parameters
m = 1575;   % total vehicle mass (kg)
Iz = 2875;  % yaw moment of inertia (mNs^2)
lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000; % cornering stiffness of front tires (N/rad)
Cr = 33000; % cornering stiffness of rear tires (N/rad)

Vx = 15;    % longitudinal velocity (m/s)

Ts = 0.1;   % sample time
T = 15;     % simulation time

u_min = -0.5;   % maximum steering angle
u_max = 0.5;    % minimum steering angle

line_width = 3.7;   % highway lane width
avg_car_width = 2;  % average car width 
max_late_dev = (line_width-avg_car_width)/2-0.1;
max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
terminate_error = 1.5;

rho = 0.001;        %  curvature of the road
e1_initial = 0.2;   % initial lateral deviation
e2_initial = 0.1;   % initial yaw angle

PredictionHorizon = 10;     % MPC prediction horizon
time = 0:Ts:T;
md = getCurvature(Vx,time); % previewed curvature

mdl = 'LKA_RL_model';       % simulink model
agentblk = [mdl '/RL Agent'];

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([6 1],...
    'LowerLimit', [-inf, -inf, -inf, -inf, -inf,-inf]',...
    'UpperLimit', [inf, inf, inf, inf, inf, inf]');
obIsnfo.Name = 'observations';
obsInfo.Description = 'lateral deviation and yaw angle';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlFiniteSetSpec(u_min:0.1:u_max);
% actInfo.LowerLimit = u_min;
% actInfo.UpperLimit = u_max;
actInfo.Name = 'steering angle';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo);

% Set a custom reset function that randomizes the reference values for the model.
env.ResetFcn = @(in)localResetFcn(in);

% Fix the random generator seed for reproducibility
rng(0);

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
% create a network to be used as underlying critic approximator
criticNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(1,'Name','CriticFC')];

% set options for the critic
criticOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',1);

% create the critic (actor-critic agents use a value function representation)
critic = rlValueRepresentation(criticNetwork,obsInfo,'Observation',{'observation'},criticOpts);

% create the network to use as approximator in the actor
actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128, 'Name','actorFC')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(10,'Name','action')
    softmaxLayer('Name','actionProb')
    ];

% set options for the actor
actorOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',1);

% create the actor (actor-critic agents use a stochastic actor representation)
actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},actorOpts);

agentOpts = rlACAgentOptions(...
    'SampleTime',Ts,...
    'DiscountFactor',0.5, ...
    'EntropyLossWeight',0.1,...
    'NumStepsToLookAhead',10); 

agent = rlACAgent(actor,critic,agentOpts);

%% train agent
% stop option: EpisodeCount AverageSteps AverageReward
maxepisodes = 8000; 
maxsteps = ceil(T/Ts);
window_length = 20;

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', maxepisodes, ...
    'MaxStepsPerEpisode', maxsteps, ...
    'ScoreAveragingWindowLength',window_length, ...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue', 2*maxsteps*0.95,...
    'Verbose', false,...
    'Plots','training-progress',...
    'UseParallel',false);

% doTraining = true;
doTraining = true;
if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
%     save("LKA_Agent.mat","agent")
else
    % Load the pretrained agent for the example.
%     agent = load('LKA_Agent.mat');
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
lateral_deviation = experiences.SimulationInfo.logsout.getElement('lateral_deviation').Values.Data;
relative_yaw_angle =  experiences.SimulationInfo.logsout.getElement('relative_yaw_angle').Values.Data;
RL_steer_angle =  experiences.SimulationInfo.logsout.getElement('RL_steer_angle').Values.Data;
RL_steer_angle = reshape(RL_steer_angle, [steps,1]);
MPC_steering_angle =  experiences.SimulationInfo.logsout.getElement('MPC_steering_angle').Values.Data;
velocity =  experiences.SimulationInfo.logsout.getElement('Vx').Values.Data;
velocity = velocity(1);
% initial_lat =  experiences.SimulationInfo.logsout.getElement('lateral_deviation').Values.Data;
% initial_lat =  experiences.SimulationInfo.logsout.getElement('lateral_deviation').Values.Data;
% initial_yaw =  experiences.SimulationInfo.logsout.getElement('lateral_deviation').Values.Data;
% initial_yaw =  experiences.SimulationInfo.logsout.getElement('lateral_deviation').Values.Data;

%% plotting
figure(1);
subplot(3,1,1)
plot(time,lateral_deviation)
title('Lateral deviation VS Time')
xlabel('Time (s)') 
ylabel('Lateral deviation') 
grid on

subplot(3,1,2)
plot(time,relative_yaw_angle)
title('Relative yaw angle VS Time')
xlabel('Time (s)') 
ylabel('Relative yaw angle') 
grid on

subplot(3,1,3)
plot(time,RL_steer_angle, time,MPC_steering_angle)
title('RL, MPC steering angle VS Time, Vx = ',velocity)
legend('RL', 'MPC')
xlabel('Time (s)') 
ylabel('Steering angle') 
grid on

% % training progress info
% episodes = trainingStats.EpisodeIndex;
% episode_avg_reward = trainingStats.AverageReward;
% episode_avg_step = trainingStats.AverageSteps;
% 
% figure(2);
% subplot(2,1,1)
% plot(episodes, episode_avg_reward)
% title('Episode Average Reward')
% xlabel('Episode') 
% ylabel('Reward') 
% grid on
% 
% subplot(2,1,2)
% plot(episodes, episode_avg_step)
% title('Episode Average Step')
% xlabel('Episode') 
% ylabel('Step') 
% grid on

% save("train_info.mat","trainingStats")
%% reset function
function in = localResetFcn(in)
    % reset
    in = setVariable(in,'e1_initial', 0.5*(-1+2*rand)); % random value for lateral deviation
    in = setVariable(in,'e2_initial', 0.1*(-1+2*rand)); % random value for relative yaw angle
    in = setVariable(in,'Vx', 10+20*rand);              % random value for longitudinal velocity
end