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
actInfo = rlNumericSpec([1 1]);
actInfo.LowerLimit = u_min;
actInfo.UpperLimit = u_max;
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
statePath1 = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
%     fullyConnectedLayer(64,'Name','CriticStateFC1')
%     reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(128,'Name','CriticStateFC2')
    reluLayer('Name','CriticStateRelu2')
    fullyConnectedLayer(64,'Name','CriticStateFC3')
    ];

actionPath1 = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
%     fullyConnectedLayer(64,'Name','CriticActionFC1')
%     reluLayer('Name','CriticActionRelu1')
    fullyConnectedLayer(128,'Name','CriticActionFC2')
    reluLayer('Name','CriticActionRelu2')
    fullyConnectedLayer(64,'Name','CriticActionFC3')
    ];

commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(32,'Name','CriticCommonFC1')
    reluLayer('Name','CriticCommonRelu2')
%     fullyConnectedLayer(128,'Name','CriticCommonFC2')
%     reluLayer('Name','CriticCommonRelu3')   
%     fullyConnectedLayer(64,'Name','CriticCommonFC3')
%     reluLayer('Name','CriticCommonRelu4')
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet = layerGraph(statePath1);
criticNet = addLayers(criticNet,actionPath1);
criticNet = addLayers(criticNet,commonPath1);
criticNet = connectLayers(criticNet,'CriticStateFC3','add/in1');
criticNet = connectLayers(criticNet,'CriticActionFC3','add/in2');

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
                                    
critic1 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

critic2 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);


% create a network to be used as underlying actor approximator
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128, 'Name','commonFC1')
    reluLayer('Name','CommonRelu1')
%     fullyConnectedLayer(64, 'Name','commonFC2')
%     reluLayer('Name','CommonRelu2')
%     fullyConnectedLayer(64, 'Name','commonFC3')
%     reluLayer('Name','CommonRelu3')
    ];

meanPath = [
    fullyConnectedLayer(64,'Name','MeanFC1')
    reluLayer('Name','MeanRelu1')
%     fullyConnectedLayer(128,'Name','MeanFC2')
%     reluLayer('Name','MeanRelu2')
    fullyConnectedLayer(32,'Name','MeanFC3')
    reluLayer('Name','MeanRelu3')
    fullyConnectedLayer(numActions,'Name','Mean')
    ];

stdPath = [
    fullyConnectedLayer(64,'Name','StdFC1')
    reluLayer('Name','StdRelu1')
%     fullyConnectedLayer(128,'Name','StdFC2')
%     reluLayer('Name','StdRelu2')
    fullyConnectedLayer(32,'Name','StdFC3')
    reluLayer('Name','StdRelu3')
    fullyConnectedLayer(numActions,'Name','StdFC4')
    softplusLayer('Name','StandardDeviation')];

concatPath = concatenationLayer(1,2,'Name','GaussianParameters');

actorNetwork = layerGraph(statePath);
actorNetwork = addLayers(actorNetwork,meanPath);
actorNetwork = addLayers(actorNetwork,stdPath);
actorNetwork = addLayers(actorNetwork,concatPath);
actorNetwork = connectLayers(actorNetwork,'CommonRelu1','MeanFC1/in');
actorNetwork = connectLayers(actorNetwork,'CommonRelu1','StdFC1/in');
actorNetwork = connectLayers(actorNetwork,'Mean','GaussianParameters/in1');
actorNetwork = connectLayers(actorNetwork,'StandardDeviation','GaussianParameters/in2');

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);

actor = rlStochasticActorRepresentation(actorNetwork,obsInfo,actInfo,actorOptions,...
    'Observation',{'observation'});

% agent creation
agentOptions = rlSACAgentOptions(...
    'SampleTime',Ts,...
    'ExperienceBufferLength',1e6,...
    'DiscountFactor',0.5,...
    'UseDeterministicExploitation',false,...
    'NumStepsToLookAhead',40,...
    'MiniBatchSize',256);

agent = rlSACAgent(actor,[critic1 critic2],agentOptions);
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
