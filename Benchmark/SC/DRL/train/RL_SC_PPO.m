clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;           % simulation duration in sec
T = 40;             % simulation time

max_Fs = 4.01;      % maximum steam flowrate
min_Fs = 3.99;
change_freq = 5;    % input signal change value every change_freq sec

P_step = 2;         % ref pressure step time
P_init = 90;        % initial ref pressure
P_final = 87;       % final ref pressure
P_terminate = 100;    % error to terminate episode
time_tol = 3;

mdl = 'SC_RL';
agentblk = [mdl, '/RL Agent'];

seed = 0;


%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([5 1],...
    'LowerLimit', -inf*ones(5,1),...
    'UpperLimit', inf*ones(5,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'T, Tcw, Q, P, P_error';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlFiniteSetSpec(3.99260: 0.00001: 3.99284);
% actInfo.LowerLimit = -1200;
% actInfo.UpperLimit = 500;
actInfo.Name = 'Fcw';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo);

% Set a custom reset function that randomizes the reference values for the model.
env.ResetFcn = @(in)localResetFcn(in);

% Fix the random generator seed for reproducibility
rng(0);

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
% create Deep deterministic policy gradient reinforcement learning agent
criticNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(256,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(128,'Name','CriticStateFC2')
    reluLayer('Name','CriticRelu2')
    fullyConnectedLayer(64,'Name','CriticStateFC3')
    reluLayer('Name','CriticRelu3')
    fullyConnectedLayer(1,'Name','CriticFC4')];

% set options for the critic
criticOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',1);

% create the critic (actor-critic agents use a value function representation)
critic = rlValueRepresentation(criticNetwork,obsInfo,'Observation',{'observation'},criticOpts);


actorNet = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(256,'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(128, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(64, 'Name','actorFC3')
    reluLayer('Name','actorRelu3')
    fullyConnectedLayer(25, 'Name','action')
    softmaxLayer('Name','actionProb')
    ];

% set some training options for the actor
actorOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',100);

% create the actor using the network
actor = rlStochasticActorRepresentation(actorNet,obsInfo,actInfo,...
    'Observation',{'observation'},actorOpts);

% agent options
agentOpts = rlPPOAgentOptions(...
    'SampleTime',Ts,...
    'ExperienceHorizon',512,...
    'DiscountFactor',0.5,...
    'MiniBatchSize',256,...
    'NumEpoch',1,...
    'AdvantageEstimateMethod','gae',...
    'UseDeterministicExploitation',false);

agent = rlPPOAgent(actor,critic,agentOpts);


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
%     save("SC_DDPG_Agent.mat","agent")
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

p_ref = experiences.SimulationInfo.logsout.getElement('p_ref').Values.Data;
p_ref = reshape(p_ref, [steps,1]);
P = experiences.SimulationInfo.logsout.getElement('<P>').Values.Data;
p_error = experiences.SimulationInfo.logsout.getElement('p_error').Values.Data;
RL_action = experiences.SimulationInfo.logsout.getElement('RL_action').Values.Data;
RL_action = reshape(RL_action, [steps,1]);

% plotting
figure(1);
subplot(3,1,1)
plot(time,p_ref, time, P)
title('Pressure level')
legend('P_{ref}','P_{out}')
xlabel('time (s)') 
ylabel('Pressure') 
grid on

subplot(3,1,2)
plot(time,p_error)
title('Pressure error')
xlabel('time (s)') 
ylabel('Error Pressure') 
grid on

subplot(3,1,3)
plot(time,RL_action)
title('RL agent action')
xlabel('time (s)') 
ylabel('Fcw: Coolong Water Flowrate') 
grid on

%% reset function
function in = localResetFcn(in)

in = setVariable(in,'seed', randi(1000,1,1));  % seed to generate random input Fs

end