% [Depreciated? Not currenlty used anywhere.]
% 
% Compute_Robustness_Sub - Computes the robustness of a simulation trajectory
% generated by the parameters passed in the input arguments
%
% USAGE
% [cost, ind, cur_par, rob] = Compute_Robustness_Sub((inputModel,inputModelType,inpArray)
%
% INPUTS:
%   inputModel: A string which indicates the name of the model to be used
%
%   inputModelType: The type of the model
%
%   inpArray: An n-by-1 array of inputs for which the robustness value
%       is computed
%
% OUTPUTS:
%   cost: cost and rob are the same for falsification problems. cost
%       and rob are different for parameter estimation problems.
%
%   ind: a flag for possible errors
%       = 0 when no problems were detected
%       = -1 when the input signal does not satisfy the constraints
%       (interpolation function does not respect the input signal bounds)
%
%   cur_par: the parameter value
%
%   rob: the robustness value

% (C) 2010, Yashwanth Annapureddy, Arizona State University
% (C) 2010, Georgios Fainekos, Arizona State University
% (C) 2012, Bardh Hoxha, Arizona State University

% Last update: 02/25/2013 - BH

function [cost, ind, cur_par, rob] = Compute_Robustness_Sub(inputModel,inputModelType,inpArray)

global staliro_mtlFormula;
global staliro_Predicate;
global staliro_SimulationTime;
global staliro_InputBounds;
global temp_ControlPoints;
global staliro_dimX;
global staliro_opt;
global staliro_ParameterIndex;
global staliro_Polarity;

% Check whether a parameter search is running and if so get the current
% parameters from the end position of the input array.
% Since inpArray contains current parameters at the end of the array we do
% not include the parameter values in XPoint and UPoint

XPoint = inpArray(1:staliro_dimX);
if staliro_opt.parameterEstimation > 0
    cur_par = inpArray(end - staliro_opt.parameterEstimation + 1:end);
    UPoint = inpArray(staliro_dimX+1:end - staliro_opt.parameterEstimation);
else
    cur_par = [];
    UPoint = inpArray(staliro_dimX+1:end);
end

ind = 1;
T = []; XT = []; YT = []; LT = []; CLG = []; GRD = [];

if size(XPoint, 2)>1
    XPoint = XPoint';
end

if strcmp(inputModelType, 'function_handle')
    if staliro_opt.black_box == 0
        % Choose ODE solver and simulate the system
        if strcmp(staliro_opt.ode_solver, 'default')
            ode_solver = 'ode45';
        else
            ode_solver = staliro_opt.ode_solver;
        end
        [T, XT] = SimFcnPtr(ode_solver, inputModel, staliro_SimulationTime, XPoint, UPoint, staliro_InputBounds, temp_ControlPoints, staliro_opt.interpolationtype);
        
    else
        % Treat the system as a black box. I.e., provide initial states and
        % input signals and get output signals
        if isempty(UPoint)
            % No inputs to the model
            [T, XT, YT, LT, CLG, GRD] = inputModel(XPoint, staliro_SimulationTime);
            
        else
            steptime = (0:staliro_opt.SampTime:staliro_SimulationTime)';
            InpSignal = ComputeInputSignals(steptime, UPoint,staliro_opt.interpolationtype, temp_ControlPoints, staliro_InputBounds, staliro_SimulationTime);
            % if inpSignal is empty then that means that the interpolation
            % function calculated a value outside of the predefined bounds
            if isempty(InpSignal)
                if staliro_opt.dispinfo == 1
                    warning('S-Taliro issue:');
                    txtmsg =          'An input signal does not satisfy the input constraints and          ';
                    txtmsg = [txtmsg; 'the optimization function is not checking for this. You may want to '];
                    txtmsg = [txtmsg; 'use a different interpolating function.                             '];
                    disp(txtmsg)
                end
                ind = -1;
                cost = inf;
                rob = inf;
                return;
            end
            [T, XT, YT, LT, CLG, GRD] = inputModel(XPoint, staliro_SimulationTime, steptime,InpSignal);
        end
    end
    
elseif strcmp(inputModelType, 'hautomaton') % unsafe set is part of HA specification
    if strcmp(staliro_opt.ode_solver, 'default')
        ode_solver = 'ode45';
    else
        ode_solver = staliro_opt.ode_solver;
    end
    if isfield(inputModel, 'simulator')
        mysimulator = inputModel.simulator;
    else
        mysimulator = @hasimulator;
    end
    hs = mysimulator(inputModel, [inputModel.init.loc 0 XPoint'], staliro_SimulationTime, ode_solver, staliro_opt.hasim_params);
    T =  hs(:, 2); % get time
    XT = hs(:, 3:end); % get continuous state trajecotry
    LT = hs(:, 1); % get location trace
    YT = []; % no output signals
    CLG = inputModel.adjList; % location graph
    GRD = inputModel.guards; % the transition guards
    
elseif strcmp(inputModelType, 'simulink')
    simopt = simget(inputModel);
    simopt = simset(simopt, 'SaveFormat', 'Array'); % Replace input outputs with structures
    if staliro_dimX~=0
        simopt = simset(simopt, 'InitialState', XPoint);
    end
    if ~strcmp(staliro_opt.ode_solver, 'default')
        simopt = simset(simopt, 'Solver', staliro_opt.ode_solver);
    end
    
    if isempty(UPoint) % If no inputs to the model
        [T, XT, YT] = sim(inputModel, [0 staliro_SimulationTime], simopt);
    else
        steptime = (0:staliro_opt.SampTime:staliro_SimulationTime)';
        InpSignal = ComputeInputSignals(steptime, UPoint,staliro_opt.interpolationtype, temp_ControlPoints, staliro_InputBounds, staliro_SimulationTime);
        % if inpSignal is empty then that means that the interpolation
        % function calculated a value outside of the predefined bounds
        if isempty(InpSignal)
            if staliro_opt.dispinfo == 1
                warning('S-Taliro issue:');
                txtmsg =          'An input signal does not satisfy the input constraints and          ';
                txtmsg = [txtmsg; 'the optimization function is not checking for this. You may want to '];
                txtmsg = [txtmsg; 'use a different interpolating function.                             '];
                disp(txtmsg)
            end
            ind = -1;
            cost = inf;
            rob = inf;
            return;
        end
        [T, XT, YT] = sim(inputModel, [0 staliro_SimulationTime], simopt, [steptime InpSignal]);
    end
    % Get location trace
    if ischar(staliro_opt.loc_traj)
        if strcmp(staliro_opt.loc_traj, 'end')
            LT = YT(:, end);
            YT(:,end) = [];
        elseif ~strcmp(staliro_opt.loc_traj, 'none')
            error('S-Taliro: "loc_traj" option not supported');
        end
    else
        LT = YT(:, staliro_opt.loc_traj);
        YT(:, staliro_opt.loc_traj) = [];
    end
else
    error('S-Taliro: The input model must be either a function pointer or a string or an HA struct.');
end

%% Compute robustness of trajectory
if staliro_opt.spec_space=='X' && ~isempty(XT)
    if isempty(staliro_opt.dim_proj)
        STraj = XT;
    else
        STraj = XT(:,staliro_opt.dim_proj);
    end
elseif staliro_opt.spec_space=='Y' && ~isempty(YT)
    if isempty(staliro_opt.dim_proj)
        STraj = YT;
    else
        STraj = YT(:,staliro_opt.dim_proj);
    end
else
    err_msg = sprintf('S-Taliro: The selected specification space (spec_space) is not supported or the signal space is empty.\n If you are using a "white box" m-function as a model, then you must set the "spec_space" to "X".');
    error(err_msg)
end

if isempty(staliro_mtlFormula) && strcmp(inputModelType, 'hautomaton')
    [nearest_point_on_s, tmin, cost] = DistTrajToUnsafe(hs(:, 2:end)', inputModel.unsafe);
    
elseif ~isempty(staliro_mtlFormula)
    
    %% Get polarity and set the fcn_cmp
    % Update pred_tmp which is a temp structure to store the current parameter
    pred_tmp = staliro_Predicate;
    
    if ~isempty(staliro_Polarity)
        if isequal(staliro_Polarity, -1)
            fcn_cmp = @le;
            
        elseif isequal(staliro_Polarity, 1)
            fcn_cmp = @ge;
        end
        pred_tmp(staliro_ParameterIndex).value = cur_par;
    end
    
    % Check whether the staliro_options obj is set to run dp_taliro
    %   or dp_t_taliro
    % It is necessary to handle the different cases since: 1.The feval function
    %   returns a different type of object based on the options set and, 2.For
    %   proper error condition checking
    
    if isequal(staliro_opt.taliro, 'dp_taliro') || isequal(staliro_opt.taliro, 'fw_taliro')
        
        switch staliro_opt.taliro_metric
            case 'none'
                tmp_rob = feval(staliro_opt.taliro, staliro_mtlFormula, pred_tmp, STraj, T);
            case 'hybrid_inf'
                if isempty(LT) || isempty(CLG)
                    error('S-Taliro: The location trace or the Control Location Graph are empty')
                else
                    tmp_rob = feval(staliro_opt.taliro, staliro_mtlFormula, pred_tmp, STraj, T, LT, CLG);
                end
            case 'hybrid'
                if isempty(LT) || isempty(CLG) || isempty(GRD)
                    error('S-TaLiRo: The location trace or the Control Location Graph or the Guard set are empty')
                else
                    tmp_rob = feval(staliro_opt.taliro, staliro_mtlFormula, pred_tmp, STraj, T, LT, CLG, GRD);
                end
            otherwise
                error('S-Taliro: the metric for taliro is not defined')
        end
        
    elseif isequal(staliro_opt.taliro, 'dp_t_taliro')
        
        switch staliro_opt.taliro_metric
            case 'none'
                tmp_rob = feval(staliro_opt.taliro, staliro_mtlFormula, pred_tmp, STraj, T);
            case {'hybrid_inf','hybrid'}
                if isempty(LT) || isempty(CLG)
                    error('S-TaLiRo: The location trace or the Control Location Graph or the Guard set are empty')
                else
                    tmp_rob = feval(staliro_opt.taliro, staliro_mtlFormula, pred_tmp, STraj, T, LT, CLG);
                end
        end
        
        switch staliro_opt.dp_t_taliro_direction
            case 'past'
                tmp_rob = tmp_rob.pt;
            case 'future'
                tmp_rob = tmp_rob.ft;
            case 'both'
                tmp_rob = min(cell2mat(struct2cell(tmp_rob)));
            otherwise
                error('S-Taliro: not defined whether the dp_t_taliro_direction option is set to "past", "future" or "both"')
        end
        
    else
        error('S-Taliro: not specified whether dp_taliro or dp_t_taliro should be used')
    end
    
else
    error('S-Taliro: MTL robustness computation for these input options is not supported. Please read the help file.')
end

% Need to check whether parameter estimation is performed so we can
% check for the error condition

if ~isempty(staliro_Polarity)
    if staliro_opt.map2line == 1
        
        if isa(tmp_rob, 'hydis')
            cost = map2line(tmp_rob, staliro_opt.rob_scale);
        else
            error('S-Taliro: if map2line is used then the rebustness must be a hydis object.')
        end
        
    elseif staliro_opt.map2line == 0 && isa(tmp_rob, 'hydis')
        error('S-Taliro: for parameter estimation, if the robustness is a hydis object, then map2line must be used.')
    else
        cost = tmp_rob;
    end
    rob = cost;
else
    if staliro_opt.map2line == 1
        if isa(tmp_rob, 'hydis')
            cost = map2line(tmp_rob, staliro_opt.rob_scale);
        else
            error('S-Taliro: if map2line is used then the rebustness must be a hydis object.')
        end
    else
        cost = tmp_rob;
    end
    rob = cost;
end

if ~isempty(staliro_Polarity)
    if cost < 0
        cost = cur_par;
        % target parameter value
        if ~isempty(staliro_opt.RobustnessOffset);
            cost = cost - staliro_opt.RobustnessOffset;
        end
        
    else
        if fcn_cmp(1, 0)
            % when the robustness is positive and the lower bound of theta negative
            % then we must make sure that max is below the lower bound of theta
            cost = sum(cur_par) + (staliro_Predicate(staliro_ParameterIndex).range(1) * 1.1) - cost;
        else
            % we can add the upper bound of the range of the parameter in order not
            % to have the robustness value changing the min value
            cost = sum(cur_par) + (staliro_Predicate(staliro_ParameterIndex).range(2) * 1.1) + cost;
        end
    end
end
end


%% Auxiliary functions

% GF 2011.08.21:
%
% A Simulink model simulation with sim cannot be performed while there
% is a nested function in the calling function. It seems that there is a
% problem with the memory space of the nested function and the workspace
% where simulink stores some variables. Thus, the nested function needs to
% be within another function definition. The nested function is necessary
% in order to avoid problems with global variables and parallel execution.

function [T, XT] = SimFcnPtr(odesolver, fcn_ptr, simTime, Xpt, Upt, inpBound, tmpCP, IntType)

[T, XT] = feval(odesolver, @ourmodel_wrapper, [0 simTime], Xpt);

%% Nested functions
    function DX = ourmodel_wrapper(t, X)
        
        if isempty(Upt)
            
            DX = feval(fcn_ptr, t, X);
            
        else
            
            Uin = ComputeInputSignals(t, Upt, IntType, tmpCP, inpBound, simTime);
            if isempty(Uin)
                error('S-Taliro: the bounds of the input signals have been violated. The interpolation function does not respect the input signal bounds.')
            end
            DX = feval(fcn_ptr, t, X, Uin);
            
        end
    end
%% End of nested functions

end
