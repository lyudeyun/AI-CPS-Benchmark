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

max_mu = 0.1;         % mu from STL
mu_tol = 0.2;          % mu value to terminate the episode
time_tol = 3;          % time tolerence, to avoid the initial large mu value
control_switch = 0;    % 0: RL controller, 1: original controller

mdl = 'AFC_RL';
agentblk = [mdl, '/Air Fuel Control Model 1/RL Agent'];

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([7 1],...
    'LowerLimit', [-inf, -inf, -inf, -inf, -inf,-inf,-inf]',...
    'UpperLimit', [inf, inf, inf, inf, inf, inf, inf]');
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

% create Deep deterministic policy gradient reinforcement learning agent
criticNet = [
    imageInputLayer([obsInfo.Dimension 1],'Normalization','none','Name','state')
    fullyConnectedLayer(64,'Name', 'fc_in')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(128,'Name', 'fc_in2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(64,'Name', 'fc_in3')
    reluLayer('Name', 'relu3')
    fullyConnectedLayer(1,'Name','out')];

% set some training options for the critic
criticOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',100);

% create the critic representation from the network
critic = rlValueRepresentation(criticNet,obsInfo,'Observation',{'state'},criticOpts);

% input path layers (2 by 1 input and a 1 by 1 output)
inPath = [ 
    imageInputLayer([obsInfo.Dimension 1], 'Normalization','none','Name','state')
    fullyConnectedLayer(64,'Name', 'ip_fc')  % 10 by 1 output
    reluLayer('Name', 'ip_relu1')             % nonlinearity
    fullyConnectedLayer(128,'Name', 'ip_fc1')  
    reluLayer('Name', 'ip_relu2')             
    fullyConnectedLayer(64,'Name', 'ip_fc2')  
    reluLayer('Name', 'ip_relu3')             
    fullyConnectedLayer(1,'Name','ip_out') ];% 1 by 1 output

% path layers for mean value (1 by 1 input and 1 by 1 output)
% using scalingLayer to scale the range
meanPath = [
    fullyConnectedLayer(64,'Name', 'mp_fc1') % 15 by 1 output
    reluLayer('Name', 'mp_relu1')       % nonlinearity
    fullyConnectedLayer(128,'Name', 'mp_fc2')  
    reluLayer('Name', 'mp_relu2')             
    fullyConnectedLayer(64,'Name', 'mp_fc3')  
    reluLayer('Name', 'mp_relu3')     
    fullyConnectedLayer(1,'Name','mp_fc4');  % 1 by 1 output
    tanhLayer('Name','tanh');                % output range: (-1,1)
    scalingLayer('Name','mp_out','Scale',0.765, 'Bias',0.895) ];
%     scalingLayer('Name','mp_out','Scale',actInfo.UpperLimit) ]; % output range: (-2N,2N)

% path layers for standard deviation (1 by 1 input and output)
% using softplus layer to make it non negative
sdevPath = [
    fullyConnectedLayer(64,'Name', 'vp_fc1') % 15 by 1 output
    reluLayer('Name', 'vp_relu1')             % nonlinearity
    fullyConnectedLayer(128,'Name', 'vp_fc2') % 15 by 1 output
    reluLayer('Name', 'vp_relu2') 
    fullyConnectedLayer(64,'Name', 'vp_fc3') % 15 by 1 output
    reluLayer('Name', 'vp_relu3') 
    fullyConnectedLayer(1,'Name','vp_fc4');  % 1 by 1 output
    softplusLayer('Name', 'vp_out') ];       % output range: (0,+Inf)

% conctatenate two inputs (along dimension #3) to form a single (2 by 1) output layer
outLayer = concatenationLayer(3,2,'Name','mean&sdev');

% add layers to layerGraph network object
actorNet = layerGraph(inPath);
actorNet = addLayers(actorNet,meanPath);
actorNet = addLayers(actorNet,sdevPath);
actorNet = addLayers(actorNet,outLayer);

% connect layers: the mean value path output MUST be connected to the FIRST input of the concatenation layer
actorNet = connectLayers(actorNet,'ip_out','mp_fc1/in');   % connect output of inPath to meanPath input
actorNet = connectLayers(actorNet,'ip_out','vp_fc1/in');   % connect output of inPath to sdevPath input
actorNet = connectLayers(actorNet,'mp_out','mean&sdev/in1');% connect output of meanPath to mean&sdev input #1
actorNet = connectLayers(actorNet,'vp_out','mean&sdev/in2');% connect output of sdevPath to mean&sdev input #2


% set some training options for the actor
actorOpts = rlRepresentationOptions('LearnRate',8e-3,'GradientThreshold',100);

% create the actor using the network
actor = rlStochasticActorRepresentation(actorNet,obsInfo,actInfo,...
    'Observation',{'state'},actorOpts);

% agent options
agentOpts = rlPPOAgentOptions(...
    'SampleTime',Ts,...
    'ExperienceHorizon',32,...
    'DiscountFactor',0.95,...
    'MiniBatchSize',32,...
    'NumEpoch',2,...
    'AdvantageEstimateMethod','gae',...
    'UseDeterministicExploitation',false);

agent = rlPPOAgent(actor,critic,agentOpts);

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
action_Fc = signals(contains(labels,'action_Fc')).values;
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