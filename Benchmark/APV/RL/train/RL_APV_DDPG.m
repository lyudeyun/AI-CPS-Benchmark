clear;
close all;
bdclose('all');

%% setup parameters
Ts = 0.1;           % simulation duration in sec
T = 12;             % simulation time
mdl = 'APV_RL_model';
agentblk = [mdl, '/controller/RL Agent'];

max_x_error = 3;        % maximum position error on x-axis
max_y_error = 3;        % maximum position error on y-axis
max_theta_error = 2;    % maximum orientation error on theta

max_steer = 0.785;      % maximum steering angle in rad
min_steer = -0.785;     % minumun steering angle in rad

max_vel = 6;            % maximum velocity in m/s
min_vel = -6;           % minimum velocity in m/s

x_error_offset = 4;     % x-axis error reward offset on denominator
y_error_offset = 4;     % x-axis error reward offset on denominator
theta_error_offset = 3; % theta-axis error reward offset on denominator

vdims = vehicleDimensions;
egoWheelbase = vdims.Wheelbase;
distToCenter = 0.5*egoWheelbase;

% Ego initial pose: x(m), y(m) and yaw angle (rad)
egoInitialPose = [4,12,0];

parkNorth = true;
if parkNorth
    egoTargetPose = [36,45,pi/2];
else
    egoTargetPose = [27.2,4.7,-pi/2];
end

costmap = helperSLCreateCostmap();
centerToFront = distToCenter;
centerToRear = distToCenter;
helperSLCreateUtilityBus;
costmapStruct = helperSLCreateUtilityStruct(costmap);

if parkNorth
    midPoint = [4,34,pi/2];
else
    midPoint = [27,12,0];
end

% Prediction horizon
p = 100;
% Control horizon
c = 100;
% Weight matrices for terminal cost
Qt = 0.5*diag([10 5 20]); 
Rt = 0.1*diag([1 2]);
% Weight matrices for tracking cost
if parkNorth
    Qp = 1e-6*diag([2 2 0]);
    Rp = 1e-4*diag([1 15]);
else
    Qp = 0*diag([2 2 0]);
    Rp = 1e-2*diag([1 5]);
end
% Safety distance to obstacles (m)
safetyDistance = 0.1;
% Maximum iteration number
maxIter = 70;
% Disable message display
mpcverbosity('off');

% Create the NLMPC controller using the specified parameters.
[nlobj,opt,paras] = createMPCForParkingValet(p,c,Ts,egoInitialPose,egoTargetPose,...
    maxIter,Qp,Rp,Qt,Rt,distToCenter,safetyDistance,midPoint);

% Set the initial conditions for the ego vehicle.
x0 = egoInitialPose';
u0 = [0;0];

% Generate the reference trajectory using the nlmpcmove function.
tic;
[mv,nloptions,info] = nlmpcmove(nlobj,x0,u0,[],[],opt);

timeVal = toc;
xRef = info.Xopt;
uRef = info.MVopt;

analyzeParkingValetResults(nlobj,info,egoTargetPose,Qp,Rp,Qt,Rt,...
    distToCenter,safetyDistance,timeVal)

% set the simulation duration and update the reference trajectory based on the duration
Duration = T;
Tsteps = Duration/Ts;
Xref = [xRef(2:p+1,:);repmat(xRef(end,:),Tsteps-p,1)];

% Create an NLMPC controller with a tracking prediction horizon (pTracking) of 10.
pTracking = 10;
nlobjTracking = createMPCForTrackingParkingValet(pTracking,Xref);

%% setup environment
% Creating an environment model includes action, obervation
% observation
obsInfo = rlNumericSpec([5 1],...
    'LowerLimit', -inf*ones(5,1),...
    'UpperLimit', inf*ones(5,1));
obIsnfo.Name = 'observations';
obsInfo.Description = 'x_error,y_error,theta_error,CurrVelocity,CurrSteer';
numObservations = obsInfo.Dimension(1);

% action
actInfo = rlNumericSpec([2 1]);
actInfo.LowerLimit = [min_vel; min_steer];
actInfo.UpperLimit = [max_vel; max_steer];
actInfo.Name = 'Speed; Steer';
numActions = actInfo.Dimension(1);

% Build the environment interface object.
env = rlSimulinkEnv(mdl,agentblk,obsInfo,actInfo,'UseFastRestart','off');


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
    fullyConnectedLayer(128,'Name','CriticStateFC2')];

actionPath = [
    featureInputLayer(2,'Normalization','none','Name','action')
    fullyConnectedLayer(128,'Name','CriticActionFC2')];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu1')
    fullyConnectedLayer(256,'Name','CriticCommonFC1')
    reluLayer('Name','CriticCommonRelu2')
    fullyConnectedLayer(64,'Name','CriticCommonFC2')
    reluLayer('Name','CriticCommonRelu3')
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
    fullyConnectedLayer(128,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(256,'Name','ActorFC3')
    reluLayer('Name','ActorRelu3')
    fullyConnectedLayer(64,'Name','ActorFC4')
    reluLayer('Name','ActorRelu4')
    fullyConnectedLayer(2,'Name','ActorFC5')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',[max_vel; max_steer])];

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
    'MiniBatchSize',128);

% StandardDeviation*sqrt(Ts) to a value between 1% and 10% of action range.
agentOptions.NoiseOptions.StandardDeviation = [0.6;0.1];
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
x_error = experiences.SimulationInfo.logsout.getElement('x_error').Values.Data;
x_error = reshape(x_error, [steps,1]);
y_error = experiences.SimulationInfo.logsout.getElement('y_error').Values.Data;
y_error = reshape(y_error, [steps,1]);
theta_error = experiences.SimulationInfo.logsout.getElement('theta_error').Values.Data;
theta_error = reshape(theta_error, [steps,1]);
RL_speed = experiences.SimulationInfo.logsout.getElement('RL_speed').Values.Data;
RL_speed = reshape(RL_speed, [steps,1]);
MPC_speed = experiences.SimulationInfo.logsout.getElement('MPC_speed').Values.Data;
MPC_speed = reshape(MPC_speed, [steps,1]);
RL_steering = experiences.SimulationInfo.logsout.getElement('RL_steering').Values.Data;
RL_steering = reshape(RL_steering, [steps,1]);
MPC_steering = experiences.SimulationInfo.logsout.getElement('MPC_steering').Values.Data;
MPC_steering = reshape(MPC_steering, [steps,1]);

% plotting
figure(1);
subplot(5,1,1)
plot(time,x_error)
title('Position error in x-axis')
xlabel('time (s)') 
ylabel('Distance (m)') 
grid on

subplot(5,1,2)
plot(time,y_error)
title('Position error in y-axis')
xlabel('time (s)') 
ylabel('Distance (m)') 
grid on

subplot(5,1,3)
plot(time,theta_error)
title('Direction error (theta)')
xlabel('time (s)') 
ylabel('Degree') 
grid on

subplot(5,1,4)
plot(time,RL_speed, time,MPC_speed)
title('Speed Command')
legend('Speed_{RL}','Speed_{MPC}')
xlabel('time (s)') 
ylabel('Veocity (m/s)') 
grid on

subplot(5,1,5)
plot(time,RL_steering, time,MPC_steering)
title('Steering Command')
legend('Steer_{RL}','Steer_{MPC}')
xlabel('time (s)') 
ylabel('Degree') 
grid on

%% reset function
function in = localResetFcn(in)
    % reset
    in = setVariable(in,'e1_initial', 0.5*(-1+2*rand)); % random value for lateral deviation
    in = setVariable(in,'e2_initial', 0.1*(-1+2*rand)); % random value for relative yaw angle
    in = setVariable(in,'Vx', 10+20*rand);              % random value for longitudinal velocity
end