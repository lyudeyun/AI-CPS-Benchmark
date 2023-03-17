classdef APV_signal_gen < signal_gen
    % Wrapper class for LKA
    properties
        mdl
        agent
    end
    methods
        function this = APV_signal_gen(mdl,agent)
            this.params = {'xpos', ...   % initial lateral deviation
                'ypos'};      % initial yaw angle
            
            this.p0 = [4 12];
            this.signals = {'x_error', 'y_error'}; % starting with bare minimum for benchmark
            this.mdl = mdl;
            this.agent = agent;
        end
        
        function X = computeSignals(this, p, t_vec)
            %% Default constant parameters
            
            global xpos ypos;
            T = 12;
            Ts = 0.1;

            xpos = p(1);
            ypos = p(2);
            
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
            egoInitialPose = [xpos,ypos,0];
            
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
            
            
%             
%             simopt = simget(this.mdl);
%             simopt = simset(simopt,'SaveFormat','Array'); % Replace input outputs with structures
            % [T, XT, YT] = sim(cur_mdl,[0 simT],simopt,[TU U]);
          
            xxx = sim(this.mdl);
            
            
            % sim(this.mdl);
            x_error = xxx.logsout.find('Name', 'x_error');
            new_x_error = x_error{1}.Values.Data(:);
            x_sz = size(new_x_error);
            new_x_error=[new_x_error; zeros(121-length(new_x_error),1)]
            
            
            y_error = xxx.logsout.find('Name', 'y_error');
            new_y_error = y_error{1}.Values.Data(:);
            new_y_error=[new_y_error; zeros(121-length(new_y_error),1)]
            X = [new_x_error';new_y_error'];
            
        end
    end
end
