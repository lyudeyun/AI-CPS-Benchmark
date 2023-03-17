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
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(32,'Name','CriticStateFC2')];

actionPath = [
    featureInputLayer(1,'Normalization','none','Name','action')
    fullyConnectedLayer(32,'Name','CriticActionFC2')];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(16,'Name','CriticCommonFC1')
    reluLayer('Name','CriticCommonRelu2')
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
    fullyConnectedLayer(32,'Name','ActorFC4')
    reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(1,'Name','ActorFC5')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',100, 'Bias',400)];

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
    'MiniBatchSize',64);

agentOptions.NoiseOptions.StandardDeviation = 0.6;
agentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-5;

% agent creation
agent = rlDDPGAgent(actor,critic,agentOptions);

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
%     agent = load('CSTR_DDPG_Agent_9_23.mat');
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
% episodes = trainingStats.EpisodeIndex;
% episode_reward = trainingStats.EpisodeReward;
% episode_step = trainingStats.EpisodeSteps;
% episode_avg_reward = trainingStats.AverageReward;
% episode_avg_step = trainingStats.AverageSteps;

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


%% reset function
function in = localResetFcn(in)
    % reset
    in = setVariable(in,'v_set', 25+randi(10,1,1));           % random value for user-set velocity
%     in = setVariable(in,'a_lead_seed', randi(10000));   % random value for a_lead generator seed
%     in = setVariable(in,'x0_lead', 60+randi(40,1,1));    % random value for lead car initial position
%     in = setVariable(in,'v0_lead', 20+randi(15,1,1));    % random value for lead car initial velocity
end