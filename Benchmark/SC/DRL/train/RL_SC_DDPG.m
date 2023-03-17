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
actInfo = rlNumericSpec([1 1]);
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
% create a network to be used as underlying critic approximator
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','State')
    fullyConnectedLayer(64,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(32,'Name','CriticStateFC2')];

actionPath = [
    featureInputLayer(numActions,'Normalization','none','Name','Action')
    fullyConnectedLayer(32,'Name','CriticActionFC1')];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,'Observation',...
    {'State'},'Action',{'Action'},criticOpts);

% create a network to be used as underlying actor approximator
actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','State')
    fullyConnectedLayer(64, 'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(32,'Name','ActorFC2')
    reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(1,'Name','ActorFC3')
    tanhLayer('Name','actorTanh')
    fullyConnectedLayer(numActions,'Name','Action')
%     scalingLayer('Name','Action','Scale',30, 'Bias',10)
    ];

actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'State'},'Action',{'Action'},actorOptions);

agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',0.85, ...
    'MiniBatchSize',64, ...
    'ExperienceBufferLength',1e6); 

% agentOpts.NoiseOptions.StandardDeviation = 0.3;
% agentOpts.NoiseOptions.StandardDeviationDecayRate = 1e-5;

agent = rlDDPGAgent(actor,critic,agentOpts);

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
