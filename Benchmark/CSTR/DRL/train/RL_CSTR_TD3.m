clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;           % simulation duration in sec
T = 30;             % simulation time

x0 = [311.2639; 8.5698; 0];     % initial concentration
u0 = [10; 298.15; 298.15];
out_max = 500;
out_min = 300;

mdl = 'CSTR_RL';
agentblk = [mdl, '/RL Agent'];

% input to simulink: take a_lead from a uniform distribution
change_freq = 5;    % input signal change value every change_freq sec
use_MPC = false;    % use mpc output as complement in reward function

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([7 1],...
    'LowerLimit', -inf*ones(7,1),...
    'UpperLimit', inf*ones(7,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'state, ref, C,T,RC';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([1 1]);
actInfo.LowerLimit = out_min;
actInfo.UpperLimit = out_max;
actInfo.Name = 'RC';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo);

% Set a custom reset function that randomizes the reference values for the model.
% env.ResetFcn = @(in)localResetFcn(in);

% Fix the random generator seed for reproducibility
rng(0);

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
% create a network to be used as underlying critic approximator
statePath1 = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
%     layerNormalizationLayer('Name','CriticNormal')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(32,'Name','CriticStateFC2')
%     reluLayer('Name','CriticStateRelu2')
%     fullyConnectedLayer(64,'Name','CriticStateFC3')
    ];
actionPath1 = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(64,'Name','CriticActionFC1')
    reluLayer('Name','CriticActionRelu1')
    fullyConnectedLayer(32,'Name','CriticActionFC2')
%     reluLayer('Name','CriticActionRelu2')
%     fullyConnectedLayer(64,'Name','CriticActionFC3')
    ];

commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(16,'Name','CriticCommonFC1')
    reluLayer('Name','CriticCommonRelu2')
%     fullyConnectedLayer(128,'Name','CriticCommonFC2')
%     reluLayer('Name','CriticCommonRelu3')
%     fullyConnectedLayer(64,'Name','CriticCommonFC3')
%     reluLayer('Name','CriticCommonRelu4')
    fullyConnectedLayer(1,'Name','CriticOutput')
%     regressionLayer('Name','CriticRegression')
    ];

criticNet = layerGraph(statePath1);
criticNet = addLayers(criticNet,actionPath1);
criticNet = addLayers(criticNet,commonPath1);
criticNet = connectLayers(criticNet,'CriticStateFC2','add/in1');
criticNet = connectLayers(criticNet,'CriticActionFC2','add/in2');

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
                                    
critic1 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

critic2 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);


% create a network to be used as underlying actor approximator
actorNet = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
%     layerNormalizationLayer('Name','ActorNormal')
%     fullyConnectedLayer(128,'Name','ActorFC1')
%     reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(64,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(32,'Name','ActorFC3')
    reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(numActions,'Name','ActorFC4')                       
    tanhLayer('Name','ActorTanh1')
%     fullyConnectedLayer(numActions,'Name','Action')
    scalingLayer('Name','ActorScaling','Scale',100, 'Bias',400)
%     regressionLayer('Name','ActorRegression')
    ];

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);
                                   
actor  = rlDeterministicActorRepresentation(actorNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

% agent creation
agentOptions = rlTD3AgentOptions(...
    'SampleTime',Ts,...
    'DiscountFactor',0.95,...
    'PolicyUpdateFrequency',1,...
    'TargetUpdateFrequency',2,...
    'TargetSmoothFactor',0.005,...
    'MiniBatchSize',64,...
    'NumStepsToLookAhead',2);


agent = rlTD3Agent(actor,[critic1 critic2],agentOptions);

%% train agent
% stop option: EpisodeCount AverageSteps AverageReward
maxepisodes = 5000; 
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
%     agent = load('preTrainedAgent.mat');
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
RC = experiences.SimulationInfo.logsout.getElement('RC').Values.Data;
ref = experiences.SimulationInfo.logsout.getElement('ref').Values.Data;
error = experiences.SimulationInfo.logsout.getElement('error').Values.Data;
RL_out = experiences.SimulationInfo.logsout.getElement('RL_out').Values.Data;
RL_out = reshape(RL_out, [steps,1]);
RL_CT = experiences.SimulationInfo.logsout.getElement('RL_CT').Values.Data;
RL_CT = reshape(RL_CT, [steps,1]);
dist = experiences.SimulationInfo.logsout.getElement('dist').Values.Data;
dist = reshape(dist, [steps,1]);


% plotting
figure(1);
subplot(3,1,1)
plot(time,ref, time, RC)
title('Reactor Concentration')
legend('ref','RC')
xlabel('time (s)') 
ylabel('Concentration') 
grid on

subplot(3,1,2)
plot(time,RL_CT)
title('RL output Coolant Temperature')
xlabel('time (s)') 
ylabel('CT') 
grid on

subplot(3,1,3)
plot(time,dist, time, error)
title('Disturbance & CT error')
legend('dist','error')
xlabel('time (s)') 
ylabel('CT') 
grid on

%% training progress info
episodes = trainingStats.EpisodeIndex;
episode_reward = trainingStats.EpisodeReward;
episode_step = trainingStats.EpisodeSteps;
episode_avg_reward = trainingStats.AverageReward;
episode_avg_step = trainingStats.AverageSteps;

% figure(2);
% subplot(2,1,1)
% plot(episodes,episode_avg_reward)
% title('Average Episode Reward ')
% xlabel('Episodes') 
% ylabel('Reward') 
% grid on
% 
% subplot(2,1,2)
% plot(episodes,episode_avg_step)
% title('Average Episode Step')
% xlabel('Episodes') 
% ylabel('Step') 
% grid on