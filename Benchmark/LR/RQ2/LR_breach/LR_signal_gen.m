classdef LR_signal_gen < signal_gen
    % Wrapper class for LKA
    properties
        mdl
        agent
    end
    methods
        function this = LR_signal_gen(mdl,agent)
            this.params = {'xpos0'};  % initial position(x,y,theta, dx/dt, dy/dt, dtheta/dt)
            this.p0 = -4;
            this.signals = {'x_error','y_error'}; 
            this.mdl = mdl;
            this.agent = agent;
        end
        
        function X = computeSignals(this, p, t_vec)
            %% Default constant parameters
            
            global T Ts xpos0 references;
            
            T = 15;             % simulation time
            Ts = 0.2;           % simulation duration in sec
            
            max_x_error = 10;        % maximum position error on x-axis
            max_y_error = 10;        % maximum position error on y-axis
            max_theta_error = 5;     % maximum orientation error on theta
            
            max_thrust  = 8;         % maximum steering angle in rad
            min_thrust  = 0;         % minumun steering angle in rad
            
            
            xpos0 = p;         % initial position x
            
            % x0 = [xpos0;ypos0;0;0;0;0];   % initial position(x,y,theta, dx/dt, dy/dt, dtheta/dt)
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
            [~,~,info] = nlmpcmove(planner,[xpos0;60;0;0;0;0],u0);
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
%             x = x0;
%             u = u0;
            % k = 1;
            references = reshape(info.Xopt',(pPlanner+1)*6,1); % Extract reference signal as column vector.
            
            sim(this.mdl);
            
            x_error = logsout.find('Name', 'x_error');
            y_error = logsout.find('Name', 'y_error');
            X = [x_error{1}.Values.Data'; y_error{1}.Values.Data'];
        end
    end
end






