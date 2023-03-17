clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;           % simulation duration in sec
T = 30;             % sample time
fuel_inj_tol=1.0;
MAF_sensor_tol=1.0;% MAF sensor error factor
AF_sensor_tol=1.0; 
pump_tol=1;
kappa_tol=1;
tau_ww_tol=1;
fault_time=50;
kp=0.04;
ki=0.14;

%The engine speed is constrained to the range [900,1100].
min_Engine_Speed = 900;
max_Engine_Speed = 1100;
%The pedal angle is constrained to the range [8.8,61.1].
min_Pedal_Angle = 8.8;
max_Pedal_Angle = 69;

% input to simulink: take a_lead from a uniform distribution
change_freq_PA = 2;    % input signal change value every change_freq sec
change_freq_ES = 2;

max_mu = 0.03;         % mu from STL
mu_tol = 0.08;          % mu value to terminate the episode
time_tol = 3;          % time tolerence, to avoid the initial large mu value
control_switch = 0;    % 0: RL controller, 1: original controller

mdl = 'AFC_RL';
agentblk = [mdl, '/Air Fuel Control Model 1/RL Agent'];

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([6 1],...
    'LowerLimit', [-inf, -inf, -inf, -inf, -inf,-inf]',...
    'UpperLimit', [inf, inf, inf, inf, inf, inf]');
obIsnfo.Name = 'observations';
obsInfo.Description = 'est_air_flow, af_ref, control_mode, af_meas, af_actual';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([1 1]);
actInfo.LowerLimit = 0.13;
actInfo.UpperLimit = 1.66;
actInfo.Name = 'commanded_fuel';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk, obsInfo,actInfo);

% Set a custom reset function that randomizes the reference values for the model.
% env.ResetFcn = @(in) setVariable(in,'seed',randi(1000),'Workspace','RL_ACCsystem');

% Fix the random generator seed for reproducibility
rng(0);

%% create Deep deterministic policy gradient reinforcement learning agent (DDPG)
% create a network to be used as underlying critic approximator
statePath1 = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','CriticStateFC1')
    reluLayer('Name','CriticStateRelu1')
    fullyConnectedLayer(300,'Name','CriticStateFC2')
    ];
actionPath1 = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(300,'Name','CriticActionFC1')
    ];
commonPath1 = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(1,'Name','CriticOutput')
    ];

criticNet = layerGraph(statePath1);
criticNet = addLayers(criticNet,actionPath1);
criticNet = addLayers(criticNet,commonPath1);
criticNet = connectLayers(criticNet,'CriticStateFC2','add/in1');
criticNet = connectLayers(criticNet,'CriticActionFC1','add/in2');

criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1,'L2RegularizationFactor',2e-4);
critic1 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
critic2 = rlQValueRepresentation(criticNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);


% create a network to be used as underlying actor approximator
actorNet = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(400,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(300,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(numActions,'Name','ActorFC3')                       
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',0.765, 'Bias',0.895)
    ];

actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-3,...
                                       'GradientThreshold',10,'L2RegularizationFactor',1e-5);
                                   
actor  = rlDeterministicActorRepresentation(actorNet,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);

% agent creation
agentOptions = rlTD3AgentOptions(...
    'SampleTime',Ts,...
    'DiscountFactor',0.99,...
    'NumStepsToLookAhead',20);

agent = rlTD3Agent(actor,[critic1 critic2],agentOptions);


%% train agent
% stop option: EpisodeCount AverageSteps AverageReward
maxepisodes = 8000; 
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

% doTraining = true;
doTraining = true;
if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
%     save("AFC_Agent.mat","agent")
else
    % Load the pretrained agent for the example.
%     agent = load('AFC_Agent.mat');
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
time = experiences.SimulationInfo.yout.time;
signals = experiences.SimulationInfo.yout.signals;
labels = {signals.label};

af = signals(contains(labels,'af')).values;
af_ref = signals(contains(labels,'ref')).values;
mu = signals(contains(labels,'mu')).values;
rms = signals(contains(labels,'rms')).values;
action_Fc = signals(contains(labels,'Fc')).values;
action_Fc = squeeze(action_Fc(1,1,:));
pedal_angle = signals(contains(labels,'PA')).values;
pedal_angle = squeeze(pedal_angle(1,1,:));
engine_speed = signals(contains(labels,'ES')).values;
engine_speed = squeeze(engine_speed(1,1,:));
org_Fc = signals(contains(labels,'org')).values;
% org_Fc = squeeze(org_Fc(1,1,:));

% plotting
figure(1);
subplot(3,1,1)
plot(time,mu)
title('Normalized error sognal (\mu) VS Time')
legend('\mu')
xlabel('Time (s)') 
ylabel('Normalized error sognal') 
grid on

subplot(3,1,2)
plot(time(10:length(time)),org_Fc(10:length(org_Fc)), time(10:length(time)), action_Fc(10:length(action_Fc)))
title('Original Commanded Fuel VS RL Commanded Fuel')
legend('Fc_{orginal}','a_{RL}')
xlabel('Time (s)') 
ylabel('Commanded Fuel (gps)') 
grid on

subplot(3,1,3)
plot(time(10:length(time)),af(10:length(af)), time(10:length(time)), af_ref(10:length(af_ref)))
title('Actual Air-Fuel ratio & Referenece Air-Fuel ratio')
legend('AF_{actual}','AF_{ref}')
xlabel('Time (s)') 
ylabel('Air-Fuelratio') 
grid on