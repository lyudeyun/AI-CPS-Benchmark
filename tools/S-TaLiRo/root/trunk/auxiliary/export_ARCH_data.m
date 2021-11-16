% export_ARCH_data - Create validation data in CSV format for the ARCH 
% falsification competition.
%
% noMatchList = export_ARCH_data(filename, system, property, results, ...
%       sysModel, InitCond, InputRange, CPArray, TotTime, Opt, exportOpt)
%
% INPUTS
%
%          - filename     :   a string without spaces for the filename for  
%                             the CSV file for exporting the data
%
%          - system       :   the string representing the benchmark model.
%                             See: https://gitlab.com/gernst/ARCH-COMP/-/blob/FALS/2021/FALS/Validation.md
%
%          - property     :   the string representing the property tested 
%                             against the benchamrk model
%                             See: https://gitlab.com/gernst/ARCH-COMP/-/blob/FALS/2021/FALS/Validation.md
%
%          - results      :   the results structure returned by staliro
%
% The rest of the inputs are the same as in the staliro function. In brief,
%
%          - sysModel     :   the model object for simulation
%
%          - InitCond     :   the range of the initial conditions, or
%                             static parameters, i.e., parameters that do 
%                             not change over time.
%
%          - InputRange   :   the range of the inputs.
%
%          - CPArray      :   contains the control points associated with
%                             each input signal. 
%
%          - TotTime      :   Total simulation time
%       
%          - Opt          :   An staliro_options object
%
%          - exportOpt    :   (optional) Export options
%                             See export_ARCH_options
%                    
% OUTPUTS
%          - noMatchList  :   The list of run indexes for which the
%                             computed and saved robustness values do not
%                             match.
%          - robustness   :   An array with the robustness values that did
%                             not match. Each row contains:
%                               [computed robustness value, saved robustness value]
%          - A file containing the required data in CSV format:
%            See: https://gitlab.com/gernst/ARCH-COMP/-/blob/FALS/2021/FALS/Validation.md
%            For reading the file in Matlab, then use the command:
%               T = readtable(file_name,'ReadVariableNames',false,'Delimiter',',')
%          
% See also: staliro, staliro_options, SimulateModel 

function [noMatchList, robustness] = export_ARCH_data(filename, system, property, results, sysModel, InitCond, InputRange, CPArray, TotTime, staliro_opt, exportOpt)

assert(isa(staliro_opt,'staliro_options'), ' staliro_opt must be an staliro_options object.')
if nargin<11
    exportOpt = export_ARCH_options;
end
assert(isa(exportOpt,'export_ARCH_options'), ' exportOpt must be an export_ARCH_options object.')

noMatchList = [];
robustness = [];

% Collect data to write to file
system_T = {'"system"'};                  % benchmark name
property_T = {'"property"'};              % requirement
input_T = {'"input"'};                    % input signals
parameters_T = {'"parameters"'};          % initial conditions 
falsified_T = {'"falsified"'};         
simulations_T = {'"simulations"'};        % how many simulations were needed for falsification? 
robustness_T = {'"robustness"'};          % the numeric robustness value as by the quantitative semantics of STL/MTL formula
stopTime_T = {'"stop time"'};             % time horizon for the validation, assigned to StopTime in Simulink
if exportOpt.output
    output_T = {'"output"'};              % expected output signal, given as time-series analogously to input
end

% For each run
for ii = 1:length(results.run)
    
    % Get best sample, number of tests, robustness 
    Sample = results.run(ii).bestSample;
    
    [T,XT,YT,InpSignal,LT,CLG,GRD,X0] = SimulateModel(sysModel,InitCond,InputRange,CPArray,Sample,TotTime,staliro_opt);
    
    if staliro_opt.spec_space=='X' && ~isempty(XT)
        STraj = XT;
    elseif staliro_opt.spec_space=='Y' && ~isempty(YT)
        STraj = YT;    
    end
    if ~isempty(staliro_opt.dim_proj)
        STraj = STraj(:,staliro_opt.dim_proj);
    end
    
    if exportOpt.validateRobustness
        tmp_rob = CallTaliroWrapper(staliro_opt.taliro, exportOpt.phi, exportOpt.preds, STraj, T, LT, CLG, GRD);
        if exportOpt.tolerance == -1
            if results.run(ii).bestRob~=tmp_rob
                warning([' Potential issue: the saved and computed robustness values for run ',num2str(ii),' do not match.'])
                noMatchList = [noMatchList, ii]; %#ok<AGROW>
                robustness = [robustness; tmp_rob results.run(ii).bestRob];
            end
        else  
            if abs(results.run(ii).bestRob-tmp_rob)>exportOpt.validateRobustness
                warning([' Potential issue: the saved and computed robustness values for run ',num2str(ii),' do not match.'])
                noMatchList = [noMatchList, ii]; %#ok<AGROW>
                robustness = [robustness; tmp_rob results.run(ii).bestRob];
            end
        end
    end

    system_T = [system_T; {['"',system,'"']}]; %#ok<AGROW>
    property_T = [property_T; {['"',property,'"']}]; %#ok<AGROW>
    input_T = [input_T; {mat2str(InpSignal)}]; %#ok<AGROW>
    parameters_T = [parameters_T; {mat2str(X0)}]; %#ok<AGROW>
    falsified_T = [falsified_T; results.run(ii).falsified]; %#ok<AGROW>
    simulations_T = [simulations_T; results.run(ii).nTests]; %#ok<AGROW>
    robustness_T = [robustness_T; results.run(ii).bestRob]; %#ok<AGROW>
    stopTime_T = [stopTime_T; TotTime];  %#ok<AGROW>
    if exportOpt.output
        output_T = [output_T; {mat2str([T, STraj])}]; %#ok<AGROW>
    end
    
end

if exportOpt.output
    T = table(system_T, property_T, input_T, parameters_T, falsified_T, simulations_T, robustness_T, stopTime_T, output_T);
else
    T = table(system_T, property_T, input_T, parameters_T, falsified_T, simulations_T, robustness_T, stopTime_T);
end
writetable(T,[filename,'.csv'],'WriteVariableNames',false);


%% Call Taliro function
function tmp_rob = CallTaliroWrapper(taliro, phi, pred_tmp, STraj, T, LT, CLG, GRD)
    % Check whether the staliro_options obj is set to run dp_taliro or dp_t_taliro
    % It is necessary to handle the different cases since: 
    %   1.The feval function returns a different type of object based on the options set and, 
    %   2.For proper error condition checking
    if isequal(taliro, 'dp_taliro') || isequal(taliro, 'fw_taliro') || isequal(taliro, 'tp_taliro')
        
        switch staliro_opt.taliro_metric
            case 'none'
                tmp_rob = feval(taliro, phi, pred_tmp, STraj, T);
            case 'hybrid_inf'
                if isempty(LT) || isempty(CLG)
                    error('S-Taliro: The location trace or the Control Location Graph are empty')
                else
                    tmp_rob = feval(taliro, phi, pred_tmp, STraj, T, LT, CLG);
                end
            case 'hybrid'
                if isempty(LT) || isempty(CLG) || isempty(GRD)
                    error('S-TaLiRo: The location trace or the Control Location Graph or the Guard set are empty')
                else
                    tmp_rob = feval(taliro, phi, pred_tmp, STraj, T, LT, CLG, GRD);
                end
            otherwise
                error('S-Taliro: the metric for taliro is not defined')
        end
        
    elseif isequal(taliro, 'dp_t_taliro')
        
        switch staliro_opt.taliro_metric
            case 'none'
                tmp_rob = feval(taliro, phi, pred_tmp, STraj, T);
            case {'hybrid_inf','hybrid'}
                if isempty(LT) || isempty(CLG)
                    error('S-TaLiRo: The location trace or the Control Location Graph or the Guard set are empty')
                else
                    tmp_rob = feval(taliro, phi, pred_tmp, STraj, T, LT, CLG);
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
        
        error('S-Taliro: MTL robustness computation for these input options is not supported. Please read the help file.')

    end
end

end

