clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;           % simulation duration in sec
T = 30;             % simulation time
t_gap = 1.4;        % time gap to calculate safe distance
D_default = 10;     % default distance
v_set = 30;         % driver-set velocity for ego car

vmax_ego = 50;      % ego car minimum/maximum velocity
vmin_ego = -50;

amin_ego = -3;      % ego car minimum/maximum acceleration
amax_ego = 2;       

x0_lead = 70;       % lead car initial velocity and position
v0_lead = 35;

x0_ego = 10;        % ego car initial velocity and position
v0_ego = 25;

amin_lead = -1;     % lead car minimum/maximum acceleration
amax_lead = 1;

mdl = 'RL_ACC';
agentblk = [mdl, '/RL Agent'];

% input to simulink: take a_lead from a uniform distribution
change_freq = 2;    % input signal change value every change_freq sec
use_MPC = false;    % use mpc output as complement in reward function

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([5 1],...
    'LowerLimit', -inf*ones(5,1),...
    'UpperLimit', inf*ones(5,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'v_set, v_ego, d_rel, v_rel, d_safe';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([1 1]);
actInfo.LowerLimit = amin_ego;
actInfo.UpperLimit = amax_ego;
actInfo.Name = 'a_ego';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo);

% Set a custom reset function that randomizes the reference values for the model.
env.ResetFcn = @(in)localResetFcn(in);
% env.ResetFcn = @(in) setVariable(in,'v_set',25+randi(10,1,1)');

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
    scalingLayer('Name','mp_out','Scale',actInfo.UpperLimit) ]; % output range: (-2N,2N)

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
    'ExperienceHorizon',65,...
    'DiscountFactor',0.95,...
    'MiniBatchSize',32,...
    'NumEpoch',2,...
    'AdvantageEstimateMethod','gae',...
    'UseDeterministicExploitation',false);

% initOpts = rlAgentInitializationOptions('NumHiddenUnit',128);
% agent = rlPPOAgent(obsInfo,actInfo,initOpts, agentOpts);
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
d_safe = experiences.SimulationInfo.logsout.getElement('d_safe').Values.Data;
d_safe = reshape(d_safe, [steps,1]);
d_rel = experiences.SimulationInfo.logsout.getElement('d_rel').Values.Data;
d_rel = reshape(d_rel, [steps,1]);
MPC_a_ego = experiences.SimulationInfo.logsout.getElement('MPC_a_ego').Values.Data;
action_a_ego = experiences.SimulationInfo.logsout.getElement('action_a_ego').Values.Data;
action_a_ego = reshape(action_a_ego, [steps,1]);
a_lead = experiences.SimulationInfo.logsout.getElement('a_lead').Values.Data;
a_lead = reshape(a_lead, [steps,1]);
v_lead = experiences.SimulationInfo.logsout.getElement('v_lead').Values.Data;
v_lead = reshape(v_lead, [steps,1]);
v_ego = experiences.SimulationInfo.logsout.getElement('v_ego').Values.Data;
v_ego = reshape(v_ego, [steps,1]);
v_set_sim =  experiences.SimulationInfo.logsout.getElement('v_set').Values.Data;
v_set_sim = v_set_sim(1);
v0_lead_sim =  experiences.SimulationInfo.logsout.getElement('v0_lead').Values.Data;
v0_lead_sim = v0_lead_sim(1);
x0_lead_sim =  experiences.SimulationInfo.logsout.getElement('x0_lead').Values.Data;
x0_lead_sim = x0_lead_sim(1);

% plotting
figure(1);
subplot(3,1,1)
plot(time,d_rel, time, d_safe)
title('Relative distance Vs Safe distance')
legend('d_{rel}','d_{safe}')
xlabel('time (s)') 
ylabel('Distance (m)') 
grid on

subplot(3,1,2)
plot(time,a_lead, time, MPC_a_ego, time,action_a_ego)
title('Leading car, MPC and Agent acceleration')
legend('a_{lead}','a_{MPC}', 'a_{agent}')
xlabel('time (s)') 
ylabel('Acceleration (m/s^{2})') 
grid on

subplot(3,1,3)
plot(time,v_lead, time, v_ego)
title(['Velocity v_{set}= ', num2str(v_set_sim)])
legend('v_{lead}','v_{ego}')
xlabel('time (s)') 
ylabel('Velocity (m/s') 
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


%% reset function
function in = localResetFcn(in)
    % reset
    in = setVariable(in,'v_set', 25+randi(10,1,1));           % random value for user-set velocity
%     in = setVariable(in,'a_lead_seed', randi(10000));   % random value for a_lead generator seed
%     in = setVariable(in,'x0_lead', 60+randi(40,1,1));    % random value for lead car initial position
%     in = setVariable(in,'v0_lead', 20+randi(15,1,1));    % random value for lead car initial velocity
end