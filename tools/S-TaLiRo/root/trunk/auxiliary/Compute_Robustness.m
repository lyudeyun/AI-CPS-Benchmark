% Compute_Robustness - Calls Compute_Robustness_Right and
% retrieves results depending on whether conformance testing, falsification
% or parameter estimation is being conducted
%
% USAGE
% [cost, ind, cur_par, rob] = Compute_Robustness(input, auxData)
%
% INPUTS:
%
%   input: 
%       * An n-dim vector of inputs for which the SUT (system under test) 
%         will be simulated/executed and the temporal logic robustness 
%         will be computed. The length of the vector should be:
%           * if varying_cp_times = 0, then the number of initial 
%             conditions (size(init_cond,1)) plus the total number of 
%             control points (sum(cp_array)).
%   		* if varying_cp_times = 1, then the number of initial 
%             conditions (size(init_cond,1)) plus the total number of 
%             control points (sum(cp_array)) plus the number of time 
%             variables for the control points (max(cp_array)-2).
%         Remark: the time variables for the control points should 
%             satisfy 0 = t0 <= t1 <= ... <= tn-1 <= tn = TotSimTime
%             See SA_Taliro.m and getNewSample.m for this case.
%             In a different implementation, the property TotSimTime of
%             staliro_options can be used for inferring TotSimTime.
%
%       * A n-dim cell vector in case multiple simulations-robustness
%         computations should be performed in parallel. Each cell should
%         contain the input vector for simulating the SUT.
%		  curSample should be a cell vector with length n_workers.
%
%   auxData: Any auxiliary data that the optimizer may need to pass to the
%       model (or SUT), or the robustness computation engine. This is
%       optimizer and application specific and it should only be used for
%       advanced features. Using the auxData means that optimizers are not
%       interchangeable any more.
%
%       Currently, the supported options are declared in the class
%           StaliroRobustnessAuxData
%
% OUTPUTS:
%
%   cost : the temporal logic robustness of the curSample. In case 
%       of parallel execution, this is a cell vector of length n_workers.
%
%       cost and rob are the same for falsification problems. 
%       cost and rob are different for parameter estimation problems.
%
%   ind : (optional) a flag for possible errors
%       = 0 when no problems were detected
%       = -1 when the input signal does not satisfy the constraints
%       (interpolation function does not respect the input signal bounds)
%
%   cur_par : (optional) the parameter value
%
%   rob : (optional) the robustness value in case of parameter estimation
%
%   otherData : (optional) Currently not used. This is a placeholder for
%       future use. 
%
%       For example, it could be modified to return the execution times from
%       simulation and robustness computation along with other statistics
%       in case parallel execution is desired within the optimization
%       algorithm. However, this is not recommended since
%       Compute_Robustness already provides a mechanism for parallel
%       execution.
%
% See also: staliro, Compute_Robustness_Right, StaliroRobustnessAuxData

% (C) 2013, Bardh Hoxha, Arizona State University

function [cost, ind, cur_par, rob, otherData] = Compute_Robustness(input, auxData)

%% Global declarations and I/O
global staliro_InputModel;
global staliro_InputModelType;
global staliro_mtlFormula;
global staliro_Predicate;
global staliro_SimulationTime;
global staliro_InputBounds;
global temp_ControlPoints;
global staliro_dimX;
global staliro_opt;
global staliro_ParameterIndex;
global staliro_Polarity;

% Read-Write global declarations (not used in parallel executions, i.e., within parfor loops)
global staliro_timeStats; 

if nargin<2
    auxData = StaliroRobustnessAuxData;
    auxData.timeStats.warnings(false);
else
    assert(isa(auxData,'StaliroRobustnessAuxData'), '     Compute_Robustness expects a StaliroRobustnessAuxData object for auxiliary data.');
end

% Check whether we need to collect time stats and whether this will be
% placed in a global variable or not. If the parallel toolbox is used, then
% the data cannot be collected in the global variable.
if staliro_opt.TimeStatsCollect
    isOnWorker = ~isempty(getCurrentTask()); % Check for parfor loop
    if isOnWorker 
        % Compute_robustness is called from within a parfor loop
        % We need to collect local data and pass it to the calling optimizer
        % The optimizer has the responsibility of correctly collecting the
        % overall data over each parallel execution.
        assert(auxData.timeStats.CollectingData, fprintf(' S-TaLiRo: Time statistics collection is requested (the property TimeStatsCollect is true) \n within a parfor loop, but the optimizer is not collecting the data. \n See readme.txt in the optimization folder. \n'));
    end
end

warning('off','Simulink:Logging:LegacyModelDataLogsFormat');

%save all the globals in a cell variable
gls = {staliro_mtlFormula, staliro_Predicate , staliro_SimulationTime, staliro_InputBounds, temp_ControlPoints,staliro_dimX, staliro_opt, staliro_ParameterIndex,staliro_Polarity, staliro_InputModel, staliro_InputModelType};


%% Simulate and compute robustness

% If multiple vectors are provided in a cell vector. This is to run multiple inputs in parallel.
if iscell(input)
    
    %initialize cell arrays
    cost = cell(1, length(input));
    ind = cell(1, length(input));
    cur_par = cell(1, length(input));
    rob = cell(1, length(input));    
    
    %check that the size of the input array is the same as the number of
    %workers
    if length(input) ~= staliro_opt.n_workers
        error('S-Taliro (Internal): The number of workers and the size of the cell array do not match.')
    end
    
    if length(input)>1
        %compute the robustness in parallel. The global bridge function is used
        %to initialize the global variables for each workers' workspace.
        parfor ii = 1:length(input)
            warning('off','Simulink:Logging:LegacyModelDataLogsFormat');
            [cost{ii}, ind{ii}, cur_par{ii}, rob{ii}, otherData{ii}] = globalBridge(gls, input{ii}, auxData);
            warning('on','Simulink:Logging:LegacyModelDataLogsFormat');
        end
    else
        warning('off','Simulink:Logging:LegacyModelDataLogsFormat');
        [cost{1}, ind{1}, cur_par{1}, rob{1}, otherData{1}] = Compute_Robustness_Right(staliro_InputModel, staliro_InputModelType, input{1}, auxData);
        warning('on','Simulink:Logging:LegacyModelDataLogsFormat');
    end
    
    % Store timing statistics
    if staliro_opt.TimeStatsCollect
        maxSimTime = otherData{1}.timeStats.simTimes;
        maxRobTime = otherData{1}.timeStats.robTimes;
        for ii = 2:length(input)
            if maxSimTime<otherData{ii}.timeStats.simTimes
                maxSimTime = otherData{ii}.timeStats.simTimes;
            end
            if maxRobTime<otherData{ii}.timeStats.robTimes
                maxRobTime = otherData{ii}.timeStats.robTimes;
            end
        end
        staliro_timeStats.addSimTime(maxSimTime);
        staliro_timeStats.addRobTime(maxRobTime);
    end
    
else
    
    if (staliro_opt.stochastic == 1) && (staliro_opt.n_workers > 1)
        
        % If the system is stochastic, a single input/parameter vector is provided, but
        % multiple simulations will be returned
        
        %initialize cell arrays
        cost = cell(1, size(input,2));
        ind = cell(1, size(input,2));
        cur_par = cell(1, size(input,2));
        rob = cell(1, size(input,2));
        otherData = cell(1, size(input,2));
        
        %compute the robustness in parallel. The global bridge function is used
        %to initialize the global variables for each workers' workspace. In
        %this case the input in an array
        parfor ii = 1:staliro_opt.n_workers
            warning('off','Simulink:Logging:LegacyModelDataLogsFormat');
            [cost{ii}, ind{ii}, cur_par{ii}, rob{ii}, otherData{ii}] = globalBridge(gls, input, auxData);
            warning('on','Simulink:Logging:LegacyModelDataLogsFormat');
        end
        
        % Store timing statistics
        if staliro_opt.TimeStatsCollect
            maxSimTime = otherData{1}.timeStats.simTimes;
            maxRobTime = otherData{1}.timeStats.robTimes;
            for ii = 2:length(input)
                if maxSimTime<otherData{ii}.timeStats.simTimes
                    maxSimTime = otherData{ii}.timeStats.simTimes;
                end
                if maxRobTime<otherData{ii}.timeStats.robTimes
                    maxRobTime = otherData{ii}.timeStats.robTimes;
                end
            end
            staliro_timeStats.addSimTime(maxSimTime);
            staliro_timeStats.addRobTime(maxRobTime);
        end
        
    else
        % this is the case when only one worker is utilized
        
        % if the goal is conformance testing then it is necessary to get the
        % robustness values for both models
        if iscell(staliro_InputModel)
            
            [cost1, ind1, ~, rob1, otherData1] = Compute_Robustness_Right(staliro_InputModel{1},staliro_InputModelType{1}, input, auxData);
            [cost2, ~, ~, rob2, otherData2] = Compute_Robustness_Right(staliro_InputModel{2},staliro_InputModelType{2}, input, auxData);
            % calculate cost by taking the absolute value of the difference of both
            % robustness values and adding a buffer of 5, which was selected
            % arbitrarily
            cost = abs(cost2 - cost1);
            ind = ind1;
            cur_par = inf;
            rob = abs(rob2 - rob1);

            % Store timing statistics
            if staliro_opt.TimeStatsCollect
                staliro_timeStats.addSimTime(otherData1.timeStats.simTimes+otherData2.timeStats.simTimes);
                staliro_timeStats.addRobTime(otherData1.timeStats.robTimes+otherData2.timeStats.robTimes);
            end
            
        else
            
            [cost, ind, cur_par, rob, otherData] = Compute_Robustness_Right(staliro_InputModel, staliro_InputModelType , input, auxData);
            
            % Store timing statistics
            if staliro_opt.TimeStatsCollect
                staliro_timeStats.addSimTime(otherData.timeStats.simTimes);
                staliro_timeStats.addRobTime(otherData.timeStats.robTimes);
            end
        end
                
    end
end

warning('on','Simulink:Logging:LegacyModelDataLogsFormat');

end
