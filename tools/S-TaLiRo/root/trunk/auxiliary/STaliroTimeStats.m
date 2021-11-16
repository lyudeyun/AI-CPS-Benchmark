classdef STaliroTimeStats < handle
    % STaliroTimeStats collects and processes time related statistics
    % for S-TaLiRo runs.
    %   
    % Example: 
    %       obj = STaliroTimeStats(3);
    %   Can store time executions for 3 simulations and for 3 robustness 
    %   computations.
	%
	% Note, the object can be instantiated with input 0. E.g. 
    %       obj = STaliroTimeStats(0);
	% However, in this case, the arrays are reallocated with every call of
	% addSimTime and addRobTime. This may slow down Matlab.
    %
    % Use the method:
    %   * addSimTime        to append a simulation time
    %                           Example: obj.addSimTime(1.2)
    %   * addRobTime        to append a robustness computation time
    %                           Example: obj.addRobTime(1.2)
    %   * totalSimTime      to return the total time spent on simulations 
    %                       (this is the sum of all the individual simulation times)
    %   * totalRobTime      to get the total time spent on robustness computations
    %                       (this is the sum of all the individual robustness computation times)
    %   * getSimTimes       to get the vector with the simulation times
    %                       (access the property simTimes to retrieve the
    %                       whole vector with initialized zero entries)
    %   * getRobTimes       to get the vector with the robustness computation times
    %                       (access the property robTimes to retrieve the
    %                       whole vector with initialized zero entries)
    %   * reset             to reset the object for a new data collection
    %   * collectData       to start or stop data collection
    %                           Example: obj.collectData(true) % Start data collection 
    %                           Example: obj.collectData(false) % Stop data collection 
    %   * warnings          to display or not warnings
    %                           Example: obj.warnings(true) % Display warnings
    %                           Example: obj.warnings(false) % Do not display warnings 
    
    properties(SetAccess=private)
        simTimes = [];
        robTimes = [];
        idxSim = 0;
        idxRob = 0;
        vecLength = 0;
        collectingData = false;
        warningDisp = true;
    end
    
    methods
        function obj = STaliroTimeStats(n)
            % STaliroTimeStats Class constructor to store time statistics 
            % for n tests.
            %
            % Example: 
            %       obj = STaliroTimeStats(3);
            %   Can store time executions for 3 simulations and for 3
            %   robustness computations.
			% Note, the object can be instantiated with input 0. E.g. 
			%       obj = STaliroTimeStats(0);
			% However, in this case, the arrays are reallocated with every call of
			% addSimTime and addRobTime. This may slow down Matlab.
			if nargin==0
				n = 0;
			end
            obj.vecLength = n;
            obj.simTimes = zeros(n,1);
            obj.robTimes = zeros(n,1);
        end
        
        function obj = reset(obj)
            % Reset object for a new experiment
            obj.simTimes = zeros(obj.vecLength,1);
            obj.robTimes = zeros(obj.vecLength,1);
            obj.idxSim = 0;
            obj.idxRob = 0;
        end
        
        function obj = addSimTime(obj, simTime)
			% Append a single simulation time
			% Example: obj.addSimTime(1.2)
            if obj.collectingData
                if isscalar(simTime) && simTime>=0
                    obj.idxSim = obj.idxSim+1;
                    obj.simTimes(obj.idxSim) = simTime;
                else
                    if obj.warningDisp
                        warning(' Update ignored. You can only add a scalar value.');
                    end
                end
            else
                if obj.warningDisp
                    warning(' Update ignored. Data collection is turned off.');
                end
            end
        end

        function obj = addRobTime(obj, robTime)
			% Append a single simulation time
			% Example: obj.addRobTime(1.2)
            if obj.collectingData
                if isscalar(robTime) && robTime>=0
                    obj.idxRob = obj.idxRob+1;
                    obj.robTimes(obj.idxRob) = robTime;
                else
                    if obj.warningDisp
                        warning(' Update ignored. You can only add a scalar value.');
                    end
                end
            else
                if obj.warningDisp
                    warning(' Update ignored. Data collection is turned off.');
                end
            end
        end
        
        function out = totalSimTime(obj)
			% Return the total time spent in simulations
            out = sum(obj.simTimes);
        end
        
        function out = totalRobTime(obj)
			% Return the total time spent in robustness computations
            out = sum(obj.robTimes);
        end
        
        function out = getSimTimes(obj)
			% Get the vector of simulation times
            if obj.idxSim==0
                out = [];
            else
                out = obj.simTimes(1:obj.idxSim);
            end
        end
        
        function out = getRobTimes(obj)
			% Get the vector of robustness computation times
            if obj.idxRob==0
                out = [];
            else
                out = obj.robTimes(1:obj.idxRob);
            end
        end
        
        function obj = collectData(obj,inp)
            % Start or stop data collection
            %    Example: obj.collectData(true) % Start data collection 
            %    Example: obj.collectData(false) % Stop data collection 
            assert(isa(inp,'logical'), ' The input value must be Boolean.')
            obj.collectingData = inp;
        end
        
        function obj = warnings(obj,inp)
            % display or not warnings
            %    Example: obj.warnings(true) % Display warnings
            %    Example: obj.warnings(false) % Do not display warnings 
            assert(isa(inp,'logical'), ' The input value must be Boolean.')
            obj.warningDisp = inp;
        end
        
        function disp(obj)
            LogicalStr = {'false', 'true'};
            disp(['   collecting data: ', LogicalStr{obj.collectingData+1}])
            if obj.idxSim == 0
                disp('   simulation times: []')
            else
                disp(['   simulation times: ',mat2str(obj.simTimes(1:obj.idxSim))])
            end
            if obj.idxRob == 0
                disp('   robustness times: []')
            else
                disp(['   robustness times: ',mat2str(obj.robTimes(1:obj.idxRob))])
            end
            disp(['   warning display: ', LogicalStr{obj.warningDisp+1}])
        end
        
    end
end

