clear;
close all;
bdclose('all');

addpath('F:\CPS\Benchmarks\LR');

%% setup parameters
T = 15;             % simulation duration in sec
Ts = 0.1;           % sample time
maxsteps = ceil(T/Ts)+1;
isTerminate = 0;       % never terminate sim in middle

max_x_error = 10;        % maximum position error on x-axis
max_y_error = 10;        % maximum position error on y-axis
max_theta_error = 5;     % maximum orientation error on theta

max_thrust  = 8;         % maximum steering angle in rad
min_thrust  = 0;         % minumun steering angle in rad

x0 = [-5;60;0;0;0;0];   % initial position(x,y,theta, dx/dt, dy/dt, dtheta/dt)
u0 = [0;0];              % target position (x,y)
pPlanner = 50;
planner = nlmpcMultistage(pPlanner,6,2);
planner.Ts = Ts;
planner.Model.StateFcn = 'RocketStateFcn';
planner.Model.StateJacFcn = 'RocketStateJacobianFcn';
planner.MV(1).Min = 0;
planner.MV(1).Max = 8; 
planner.MV(2).Min = 0;
planner.MV(2).Max = 8;
planner.States(2).Min = 10;
for ct=1:pPlanner
    planner.Stages(ct).CostFcn = 'RocketPlannerCostFcn';
    planner.Stages(ct).CostJacFcn = 'RocketPlannerCostGradientFcn';
end

planner.Model.TerminalState = [0;10;0;0;0;0];
planner.Optimization.SolverOptions.MaxIterations = 1000;
[~,~,info] = nlmpcmove(planner,x0,u0);
pLander = 10;
lander = nlmpcMultistage(pLander,6,2);
lander.Ts = Ts;
lander.Model.StateFcn = 'RocketStateFcn';
lander.Model.StateJacFcn = 'RocketStateJacobianFcn';
lander.MV(1).Min = 0;
lander.MV(1).Max = 8;
lander.MV(2).Min = 0;
lander.MV(2).Max = 8;
lander.States(2).Min = 10;
for ct=1:pLander+1
    lander.Stages(ct).CostFcn = 'RocketLanderCostFcn';
    lander.Stages(ct).CostJacFcn = 'RocketLanderCostGradientFcn';
    lander.Stages(ct).ParameterLength = 6;
end
lander.UseMVRate = true;
x = x0;
u = u0;
k = 1;
references = reshape(info.Xopt',(pPlanner+1)*6,1); % Extract reference signal as column vector.

% mdl = 'LR_T_eval';
mdl = 'LR_RL_eval';

%% simulation & plotting 
% simulation option
sim_num = 10;               % simulation numbers

% load agent 
agent = load('LR_DDPG_Agent_10_5.mat');
agent = agent.agent;

% file_name = 'LR_eval_T_result';
file_name = 'LR_eval_DDPG_result';
% file_name = 'LR_eval_TD3_result';

control_switch = 0;         % 0: RL controller, 1: original controller

T1_x = ones(sim_num,1);
T1_y = ones(sim_num,1);
T1_theta = ones(sim_num,1);
T1_dx = ones(sim_num,1);
T1_dy = ones(sim_num,1);

for index = 1:sim_num
    x_1 = randi([-10,10]);
    x0 = [x_1;60;0;0;0;0];
    [~,~,info] = nlmpcmove(planner,x0,u0);
    references = reshape(info.Xopt',(pPlanner+1)*6,1);
    
    result = sim(mdl);
    x = result.logsout.getElement('x').Values.Data;
    step_num = length(x);                  % number of samples
    y = result.logsout.getElement('y').Values.Data;
    theta = result.logsout.getElement('theta').Values.Data;
    dx = result.logsout.getElement('dx').Values.Data;
    dy = result.logsout.getElement('dy').Values.Data;

    % T1: average & maximum error 
    T1_x(index) = abs(x(step_num));               
    T1_y(index) = abs(y(step_num)); 
    T1_theta(index) = abs(theta(step_num)); 
    T1_dx(index) = abs(dx(step_num)); 
    T1_dy(index) = abs(dy(step_num)); 
     
    if rem(index,2)== 0
        disp(index);
    end
end

detail_result = struct('T1_x',T1_x, 'T1_y',T1_y,'T1_theta',T1_theta, 'T2_x',T1_dx, 'T2_y',T1_dy);        
                    
save(append(file_name,'_detailed.mat'),"detail_result");

final_result = struct('T1_x_avg',mean(T1_x), 'T1_y_avg',mean(T1_y), 'T1_theta_avg',mean(T1_theta), 'T1_dx_avg',mean(T1_dx), 'T1_dy_avg',mean(T1_dy));
            
save(append(file_name,'_.mat'),"final_result");