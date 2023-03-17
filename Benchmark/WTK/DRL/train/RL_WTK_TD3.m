clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;
T = 30;
desire_WL = 10;
change_freq = 4;

%% Environment setup
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([3 1],...
    'LowerLimit',[-inf -inf 0  ]',...
    'UpperLimit',[ inf  inf inf]');
obsInfo.Name = 'observations';
obsInfo.Description = 'integrated error, error, and measured height';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([1 1]);
actInfo.Name = 'flow';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv('rlwatertank','rlwatertank/RL Agent',...
    obsInfo,actInfo);

% Set a custom reset function that randomizes parameters
env.ResetFcn = @(in)localResetFcn(in);

% Fix the random generator seed for reproducibility
rng(0)

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(50,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(25,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(25,'Name','CriticActionFC1')];
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

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
                                    
critic1 = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

critic2 = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);


% create a network to be used as underlying actor approximator
actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(3, 'Name','actorFC')
    tanhLayer('Name','actorTanh')
    fullyConnectedLayer(numActions,'Name','Action')
    ];


actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-4,...
                                       'GradientThreshold',1,'L2RegularizationFactor',1e-5);
                                   
actor  = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'Action'},actorOptions);

% agent creation
agentOptions = rlTD3AgentOptions(...
    'SampleTime',Ts,...
    'DiscountFactor',0.95,...
    'PolicyUpdateFrequency',1,...
    'TargetUpdateFrequency',1,...
    'TargetSmoothFactor',0.001,...
    'MiniBatchSize',64,...
    'NumStepsToLookAhead',20);


agent = rlTD3Agent(actor,[critic1 critic2],agentOptions);


%% train agent
% stop option: EpisodeCount AverageSteps AverageReward
maxepisodes = 5000; 
maxsteps = ceil(T/Ts);

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', maxepisodes, ...
    'MaxStepsPerEpisode', maxsteps, ...
    'ScoreAveragingWindowLength',20, ...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue', 1500,...
    'Verbose', false,...
    'Plots','training-progress',...
    'UseParallel',false);

doTraining = true;
if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
%     save("preTrainedAgent.mat","agent")
else
    % Load the pretrained agent for the example.
%     load('initialAgent.mat','agent');
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
h_ref = experiences.SimulationInfo.logsOut.getElement('h_ref').Values.Data;
h_ref = reshape(h_ref, [steps,1]);
h_out = experiences.SimulationInfo.logsOut.getElement('h_out').Values.Data;
h_error = experiences.SimulationInfo.logsOut.getElement('h_error').Values.Data;

rl_action = experiences.SimulationInfo.logsOut.getElement('action').Values.Data;
rl_action = reshape(rl_action, [steps,1]);

% plotting
figure(1);
subplot(3,1,1)
plot(time,h_ref, time, h_out)
title('Water level')
legend('H_{ref}','H_{out}')
xlabel('time (s)') 
ylabel('Height (m)') 
grid on

subplot(3,1,2)
plot(time,h_error)
title('Height error')
xlabel('time (s)') 
ylabel('Height error') 
grid on

subplot(3,1,3)
plot(time,rl_action)
title('RL agent action')
xlabel('time (s)') 
ylabel('') 
grid on

%% reset function
function in = localResetFcn(in)

% randomize initial height
h = 3*randn + 10;
while h <= 0 || h >= 20
    h = 3*randn + 10;
end
blk = 'rlwatertank/Water-Tank System/H';
in = setBlockParameter(in,blk,'InitialCondition',num2str(h));

end