classdef LKA_signal_gen < signal_gen
    % Wrapper class for LKA
    properties
        mdl
        agent
    end
    methods
        function this = LKA_signal_gen(mdl,agent)
            this.params = {'Vx', ...
                'e1_initial', ...   % initial lateral deviation
                'e2_initial'};      % initial yaw angle
            
            this.p0 = [15; 0; 0];
            this.signals = {'lateral_deviation'}; % starting with bare minimum for benchmark
            this.mdl = mdl;
            this.agent = agent;
        end
        
        function X = computeSignals(this, p, t_vec)
            %% Default constant parameters
            
            global Vx e1_initial e2_initial
            T = 15;
            Ts = 0.1;
            
            m = 1575;   % total vehicle mass (kg)
            Iz = 2875;  % yaw moment of inertia (mNs^2)
            lf = 1.2;   % longitudinal distance from center of gravity to front tires (m)
            lr = 1.6;   % longitudinal distance from center of gravity to rear tires (m)
            Cf = 19000; % cornering stiffness of front tires (N/rad)
            Cr = 33000; % cornering stiffness of rear tires (N/rad)
            
            Vx  = p(1);  % longitudinal velocity (m/s)
            
            u_min = -0.5;   % maximum steering angle
            u_max = 0.5;    % minimum steering angle
            
            line_width = 3.7;   % highway lane width
            avg_car_width = 2;  % average car width
            max_late_dev = (line_width-avg_car_width)/2-0.1;
            max_rel_yaw_ang = 0.261799; % lateral deviation tolerence
            terminate_error = 1.5;
            
            rho = 0.001;        %  curvature of the road
            
            time = 0:Ts:T;
            
            md = getCurvature(Vx,time);
            
            e1_initial = p(2);
            e2_initial = p(3);
            
            sim(this.mdl);
            data = logsout.find('Name', 'lateral_deviation');
            X = data{1}.Values.Data';
        end
    end
end
