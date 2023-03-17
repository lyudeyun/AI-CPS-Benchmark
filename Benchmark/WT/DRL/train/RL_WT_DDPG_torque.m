% Wind-Turbin
clear;
close all;
bdclose('all');

%% Parameter Setup
SimplifiedTurbine_Config;
addpath('tools/')
addpath('wind/')
addpath('wafo/wafo')
% addpath(config.wafo_path)
load('ClassA.mat')
load('ClassA_config.mat')
load('aeromaps3.mat');
Parameter.InitialConditions = load('InitialConditions');
% remove all unnecessary fields (otherwise Simulink will throw an error)
cT_modelrm = rmfield(cT_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});
cP_modelrm = rmfield(cP_model,{'VarNames'});%,'RMSE','ParameterVar','ParameterStd','R2','AdjustedR2'});

% initialize WAFO
initwafo 

%%
iBin = find(URefVector==Parameter.URef);
iRandSeed = 1;

T = 630;     % simulation duration
Ts = 0.1;   % simuation step size

config.iBin                         = iBin;
config.iRandSeed                    = iRandSeed;
Parameter.v0                        = v0_cell{iBin,iRandSeed};
Parameter.v0.signals.values         = Parameter.v0.signals.values';
Parameter.TMax                      = v0_cell{iBin,iRandSeed}.time(end);
config.WindFieldName                = FileNames{iBin,iRandSeed};
% Time
Parameter.Time.TMax                 = T;              % [s]       duration of simulation
Parameter.Time.dt                   = Ts;             % [s]       time step of simulation
Parameter.Time.cut_in               = 30;
Parameter.Time.cut_out              = Parameter.Time.TMax;
Parameter.v0_0 = Parameter.v0.signals.values(1);

Parameter = SimplifiedTurbine_ParamterFile(Parameter);

mdl = 'RL_WT_torque';
agentblk = [mdl, '/WT_model/RL Agent'];


max_vin = 16;       % maximum input wind speed
min_vin = 8;
change_freq = 60;   % input signal change value every change_freq sec

max_theta = Parameter.Pitch.Max;   % maximum angle controller output
min_theta = Parameter.Pitch.Min;
max_torque = Parameter.VSControl.VS_MaxTq;
min_torque = 2.1e+4;

omega_term_error = 60;   % omega error to terminate episode
theta_term_error = 16;  % theta error to terminate episode
time_tol = 180;           % error exemption time

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([4 1],...
    'LowerLimit', -inf*ones(4,1),...
    'UpperLimit', inf*ones(4,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'omega_g, theta, omega_g_rated, theta_d';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([1 1]);
actInfo.LowerLimit = min_torque/1000;
actInfo.UpperLimit = max_torque/1000;
actInfo.Name = 'Fcw';
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
%     scalingLayer('Name','Action','Scale',max_theta/2, 'Bias',max_theta/2)
    ];

actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'State'},'Action',{'Action'},actorOptions);

agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',0.95, ...
    'MiniBatchSize',256, ...
    'NumStepsToLookAhead', 40,...
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

% theta_d = experiences.SimulationInfo.logsout.getElement('theta_d').Values.Data;
% theta_d = reshape(theta_d, [steps,1]);
theta = experiences.SimulationInfo.logsout.getElement('theta').Values.Data;
theta_error = experiences.SimulationInfo.logsout.getElement('theta_error').Values.Data;
theta_error = reshape(theta_error, [steps,1]);
% RL_mg_d = experiences.SimulationInfo.logsout.getElement('RL_mg_d').Values.Data;
% RL_mg_d = reshape(RL_mg_d, [steps,1]);
RL_mg_d = experiences.SimulationInfo.logsout.getElement('RL_mg_d').Values.Data;
RL_mg_d = reshape(RL_mg_d, [steps,1]);
v0 = experiences.SimulationInfo.logsout.getElement('v0').Values.Data;
% mg_d = experiences.SimulationInfo.logsout.getElement('mg_d').Values.Data;
omege = experiences.SimulationInfo.logsout.getElement('omege').Values.Data;
omege_gen = experiences.SimulationInfo.logsout.getElement('omege_gen').Values.Data;
oemga_g_error = experiences.SimulationInfo.logsout.getElement('oemga_g_error').Values.Data;


% plotting
figure(1);
subplot(3,1,1)
plot(time,theta_error)
title('Blade Pitch Angle Error')
xlabel('time (s)') 
ylabel('Degree') 
grid on

subplot(3,1,2)
plot( time,oemga_g_error)
title('Generator speed error')
% legend('omega','omega gen')
xlabel('time (s)') 
ylabel('rpm') 
grid on

subplot(3,1,3)
plot(time,RL_mg_d)
title('RL Demanded theta')
xlabel('time (s)') 
ylabel('RL theta_d ') 
grid on


% subplot(3,1,3)
% plot(time,theta, time,theta_d, time,v0)
% title('Blade Pitch angle compare')
% legend('theta','theta_d', 'v0')
% xlabel('time (s)') 
% ylabel('Degree') 
% grid on




