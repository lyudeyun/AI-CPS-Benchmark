function [run, history] = SOAR_Taliro_SPSA(inpRanges,opt)
% UR_Taliro - Performs random sampling in the state and input spaces.
%
% USAGE:
%   [run, history] = UR_Taliro(inpRanges,opt)
%
% INPUTS:
%
%   inpRanges: n-by-2 lower and upper bounds on initial conditions and
%       input ranges, e.g.,
%           inpRanges(i,1) <= x(i) <= inpRanges(i,2)
%       where n = dimension of the initial conditions vector +
%           the dimension of the input signal vector * # of control points
%
%   opt : staliro options object
%
% OUTPUTS:
%   run: a structure array that contains the results of each run of
%       the stochastic optimization algorithm. The structure has the
%       following fields:
%
%           bestRob : The best (min or max) robustness value found
%
%           bestSample : The sample in the search space that generated
%               the trace with the best robustness value.
%
%           nTests: number of tests performed (this is needed if
%               falsification rather than optimization is performed)
%
%           bestCost: Best cost value. bestCost and bestRob are the
%               same for falsification problems. bestCost and bestRob
%               are different for parameter estimation problems. The
%               best robustness found is always stored in bestRob.
%
%           paramVal: Best parameter value. This is used only in
%               parameter querry problems. This is valid if only if
%               bestRob is negative.
%
%           falsified: Indicates whether a falsification occured. This
%               is used if a stochastic optimization algorithm does not
%               return the minimum robustness value found.
%
%           time: The total running time of each run. This value is set by
%               the calling function.
%
%   history: array of structures containing the following fields
%
%       rob: all the robustness values computed for each test
%
%       samples: all the samples generated for each test
%
%       cost: all the cost function values computed for each test.
%           This is the same with robustness values only in the case
%           of falsification.
%
% See also: staliro, staliro_options, UR_Taliro_parameters

% (C) 2010, Sriram Sankaranarayanan, University of Colorado
% (C) 2010, Georgios Fainekos, Arizona State University
params = opt.optim_params;
nSamples = params.n_tests;
StopCond = opt.falsification;

[nInputs, ~] = size(inpRanges); 

% Initialize outputs
run = struct('bestRob',[],'bestSample',[],'nTests',[],'bestCost',[],'paramVal',[],'falsified',[],'time',[]);
history = struct('rob',[],'samples',[],'cost',[]);

% %initialize curSample vector
% curSample = repmat({0}, 1, opt.n_workers);

% get polarity and set the fcn_cmp
if isequal(opt.parameterEstimation,1)
    if isequal(opt.optimization,'min')
        fcn_cmp = @le;
        minmax = @min;
    elseif isequal(opt.optimization,'max')
        fcn_cmp = @ge;
        minmax = @max;
    end
else
    fcn_cmp = @le;
    minmax = @min;
end

% if rem(nSamples/opt.n_workers,1) ~= 0
%     error('The number of tests (opt.ur_params.n_tests) should be divisible by the number of workers.')
% end

% Start SOAR Here 
crowded_EI_flag = params.crowded_EI_flag; %if equal to 1 then use crowded EI, if 0 use EI
n_0 = 10*nInputs;   %number of initial design points
B = 1;     %number of replications per design point, set to 1 for deterministic problem
B_n0_setting=1;

%Crowded EI level set threshold
alpha_lvl_set = params.crowding_threshold; %i.e. EIs within 5% of maxEI

%%parameters for the TR algorithm, user defined
epsilon= params.finite_diff; %for finite differencing
%for RC test and TR control
eta0= params.TR_lowpass_thresh; 
eta1= params.TR_highpass_thresh;
delta= params.TR_delta;
gamma= params.TR_gamma;

% Instantiate and scale initial design
sim_count = 0;
x_0 = lhsdesign(n_0,nInputs,'criterion','maximin')';
x_0 = x_0.*(inpRanges(:,2) - inpRanges(:,1)) + inpRanges(:,1);


%take first samples, check falsification
for i = 1:n_0  
    curSample{i} = x_0(:,i);
    [curVal{i}, ~, tm_params, rob] = Compute_Robustness(curSample{i});
    sim_count = sim_count + 1;
    
    %instantiate storage/history if first sample
    if nargout>1 && i == 1
        if isa(curVal{1},'hydis')
            history.cost = hydis(zeros(nSamples,1));
            history.rob = hydis(zeros(nSamples,1));
        else
            history.cost = zeros(nSamples,1);
            history.rob = zeros(nSamples,1);   
        end
        history.samples = zeros(nSamples,nInputs);
    end
    
    %store as necessary
    if nargout>1
        if isa(curVal{i},'hydis')
            history.cost(i) = hydisc2m(curVal(i))';
            history.rob(i) = hydisc2m(curVal(i))';
        else
            history.cost(i) = curVal{i}';
            history.rob(i) = curVal{i}'; 
        end
        history.samples(i,:) = curSample{i}'; 
    end
    
    %find and store the best value seen so far
    if isa(curVal{1},'hydis')
        [minmax_val, minmax_idx] = minmax(hydisc2m(curVal));
    else
        [minmax_val, minmax_idx] = minmax(cell2mat(curVal));
    end
    bestCost = minmax_val;
    run.bestCost = minmax_val;
    run.bestSample = curSample{minmax_idx};
    run.bestRob = minmax_val;
    run.falsified = minmax_val<=0;
    run.nTests = sim_count;
    
    %check if best value is falsifying, if so, exit as necessary
    if (fcn_cmp(minmax_val,0) && StopCond)
        if nargout>1
            if isa(minmax_val,'hydis')
                history.cost(i+1:end) = hydis([],[]);
                history.rob(i+1:end) = hydis([],[]);
            else
                history.cost(i+1:end) = [];
                history.rob(i+1:end) = [];
            end
            history.samples(i+1:end,:) = [];
        end
        disp(' SOAR_Taliro: FALSIFIED by initializing samples!');
        return;
    end
end

%set up for surrogate modeling
xTrain = cell2mat(curSample)';
yTrain = cell2mat(curVal)';
all_x = xTrain;
all_y = yTrain;

clear curSample curVal;

% Initialize MOPSO Parameters
        MOparams.Np = 200;        % Population size
        MOparams.Nr = 200;        % Repository size
        MOparams.maxgen = 500;    % Maximum number of generations
        MOparams.W = 0.4;         % Inertia weight
        MOparams.C1 = 2;          % Individual confidence factor
        MOparams.C2 = 2;          % Swarm confidence factor
        MOparams.ngrid = 20;      % Number of grids in each dimension
        MOparams.maxvel = 5;      % Maxmium vel in percentage
        MOparams.u_mut = 0.5;     % Uniform mutation percentage  
        MultiObj.nVar = nInputs;  % Set problem dimension
        MultiObj.var_min = inpRanges(:,1)';
        MultiObj.var_max = inpRanges(:,2)';
    

while(sim_count < nSamples)
    %Fit Gaussian Process Meta Model
    GPmod = OK_Rmodel_kd_nugget(xTrain, yTrain, 0, 2);
       
    % optimize EI and CD with MOPSO    
    MultiObj.fun = @(x)[-EIcalc_kd(x,xTrain,GPmod,yTrain), -CrowdingDist_kd(x,all_x)];
    
    pf = MOPSO(MOparams,MultiObj);
    [minNegEI, index] = min(pf.pos_fit(:,1));
   
   %use crowded EI
   if crowded_EI_flag == 1 
       best_crowd = inf;
       for k = 1:size(pf.pos,1)
           if pf.pos_fit(k,1) <= (minNegEI*(1-alpha_lvl_set))
               if pf.pos_fit(k,2) < best_crowd
                   best_crowd = pf.pos_fit(k,2);
                   x0 = pf.pos(k,:);
               end
           end
       end
   %use standard EI   
   else
        x0 = pf.pos(index,:);
   end
       
   %store acquisition function sample appropriately and sample it
   curSample{1} = x0';
   curVal = Compute_Robustness(curSample);
   f0 = curVal{1};
   sim_count = sim_count + 1;
   
   %store as necessary
    if nargout>1
        if isa(curVal{1},'hydis')
            history.cost(sim_count) = hydisc2m(curVal)';
            history.rob(sim_count) = hydisc2m(curVal)';
        else
            history.cost(sim_count) = curVal{1};
            history.rob(sim_count) = curVal{1}; 
        end
        history.samples(sim_count,:) = curSample{1}'; 
    end
    
    %find and store the best value seen so far
    if isa(curVal{1},'hydis')
        [minmax_val, minmax_idx] = minmax(hydisc2m(curVal));
    else
        [minmax_val, minmax_idx] = minmax(cell2mat(curVal));
    end
    
    if (fcn_cmp(minmax_val,bestCost))
        bestCost = minmax_val;
        run.bestCost = minmax_val;
        run.bestRob = minmax_val;
        run.bestSample = curSample{minmax_idx};
        if opt.dispinfo>0
            if isa(minmax_val,'hydis')
                disp(['Best ==> <',num2str(get(minmax_val,1)),',',num2str(get(minmax_val,2)),'>']);
            else
                disp(['Best ==> ' num2str(minmax_val)]);
            end
        end
    end
    %check if best value is falsifying or if , if so, exit as necessary
    if (fcn_cmp(bestCost,0) && StopCond)
        run.falsified = 1;
        run.nTests = sim_count;
        if nargout>1
            if isa(minmax_val,'hydis')
                history.cost(sim_count+1:end) = hydis([],[]);
                history.rob(sim_count+1:end) = hydis([],[]);
            else
                history.cost(sim_count+1:end) = [];
                history.rob(sim_count+1:end) = [];
            end
            history.samples(sim_count+1:end,:) = [];
        end
        disp(' SOAR_Taliro: FALSIFIED!');
        save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
        return;
    end
    %check if budget has been exhausted
    if sim_count >= nSamples
        run.nTests = sim_count;
        disp(' SOAR_Talro: Samples Exhausted!');
        save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
        return;
    end

   all_x = [all_x; x0];
   all_y = [all_y; f0];
   
   xTrain = [xTrain; x0];
   yTrain = [yTrain; f0];
   
   
   
   %%%%%%%%%%%%%%%%%%%%% LOCAL SEARCH PHASE %%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
   %%%% Initialize SPSA Algorithm %%%%%
   stop_cond = 0;
   % following parameters set based off of Spall (1998a)
   alpha = 0.602;
   gamma = 0.101;
   c = epsilon;     % less important for deterministic problems, in general should be std dev of noise
   k = 0;
   A = nSamples/50; % A <= 10% of max iteration in each local search
   if opt.varying_cp_times == 1
       a0 = min(inpRanges(1:nInputs/2,2)-inpRanges(1:nInputs/2,1));
       a1 = min(inpRanges(nInputs/2+1:end,2)-inpRanges(nInputs/2+1:end,1));
       a = min(inpRanges(:,2)-inpRanges(:,1));
   else
       a = min(inpRanges(:,2)-inpRanges(:,1));
   end
   gk = ones(nInputs,1);
   
   %%% Execute SPSA Algorithm with Adaptive Restart Condition %%%
   while ~stop_cond && norm(gk) > 0.05
       % draw a random simultaneous perturbation vector of -1/1's
       delta_p = binornd(1,0.5,nInputs,1);
       delta_p(delta_p == 0) = -1;
       
       % determine perturbation size
       ck = c/((k+1)^gamma);
       
       % determine perturbation locations
       x0_forward = x0 + ck*delta_p';
       x0_backward = x0 - ck*delta_p';
              
       % sample the perturbation and check/store falsification
       curSample{1} = x0_forward';
       curVal = Compute_Robustness(curSample);
       f0_f = curVal{1};
       sim_count = sim_count + 1;

                %store as necessary
                if nargout>1
                    if isa(curVal{1},'hydis')
                        history.cost(sim_count) = hydisc2m(curVal)';
                        history.rob(sim_count) = hydisc2m(curVal)';
                    else
                        history.cost(sim_count) = curVal{1};
                        history.rob(sim_count) = curVal{1}; 
                    end
                    history.samples(sim_count,:) = curSample{1}'; 
                end

                %find and store the best value seen so far
                if isa(curVal{1},'hydis')
                    [minmax_val, minmax_idx] = minmax(hydisc2m(curVal));
                else
                    [minmax_val, minmax_idx] = minmax(cell2mat(curVal));
                end

                if (fcn_cmp(minmax_val,bestCost))
                    bestCost = minmax_val;
                    run.bestCost = minmax_val;
                    run.bestRob = minmax_val;
                    run.bestSample = curSample{minmax_idx};
                    if opt.dispinfo>0
                        if isa(minmax_val,'hydis')
                            disp(['Best ==> <',num2str(get(minmax_val,1)),',',num2str(get(minmax_val,2)),'>']);
                        else
                            disp(['Best ==> ' num2str(minmax_val)]);
                        end
                    end
                end
                %check if best value is falsifyin , if so, exit as necessary
                if (fcn_cmp(bestCost,0) && StopCond)
                    run.falsified = 1;
                    run.nTests = sim_count;
                    if nargout>1
                        if isa(minmax_val,'hydis')
                            history.cost(sim_count+1:end) = hydis([],[]);
                            history.rob(sim_count+1:end) = hydis([],[]);
                        else
                            history.cost(sim_count+1:end) = [];
                            history.rob(sim_count+1:end) = [];
                        end
                        history.samples(sim_count+1:end,:) = [];
                    end
                    disp(' SOAR_Taliro: FALSIFIED!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
                %check if budget has been exhausted
                if sim_count >= nSamples
                    run.nTests = sim_count;
                    disp(' SOAR_Talro: Samples Exhausted!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
            all_x = [all_x; x0_forward];
            all_y = [all_y; f0_f];
       
       curSample{1} = x0_backward';
       curVal = Compute_Robustness(curSample);
       f0_b = curVal{1};
       sim_count = sim_count + 1;
       
                %store as necessary
                if nargout>1
                    if isa(curVal{1},'hydis')
                        history.cost(sim_count) = hydisc2m(curVal)';
                        history.rob(sim_count) = hydisc2m(curVal)';
                    else
                        history.cost(sim_count) = curVal{1};
                        history.rob(sim_count) = curVal{1}; 
                    end
                    history.samples(sim_count,:) = curSample{1}'; 
                end

                %find and store the best value seen so far
                if isa(curVal{1},'hydis')
                    [minmax_val, minmax_idx] = minmax(hydisc2m(curVal));
                else
                    [minmax_val, minmax_idx] = minmax(cell2mat(curVal));
                end

                if (fcn_cmp(minmax_val,bestCost))
                    bestCost = minmax_val;
                    run.bestCost = minmax_val;
                    run.bestRob = minmax_val;
                    run.bestSample = curSample{minmax_idx};
                    if opt.dispinfo>0
                        if isa(minmax_val,'hydis')
                            disp(['Best ==> <',num2str(get(minmax_val,1)),',',num2str(get(minmax_val,2)),'>']);
                        else
                            disp(['Best ==> ' num2str(minmax_val)]);
                        end
                    end
                end
                %check if best value is falsifying or if , if so, exit as necessary
                if (fcn_cmp(bestCost,0) && StopCond)
                    run.falsified = 1;
                    run.nTests = sim_count;
                    if nargout>1
                        if isa(minmax_val,'hydis')
                            history.cost(sim_count+1:end) = hydis([],[]);
                            history.rob(sim_count+1:end) = hydis([],[]);
                        else
                            history.cost(sim_count+1:end) = [];
                            history.rob(sim_count+1:end) = [];
                        end
                        history.samples(sim_count+1:end,:) = [];
                    end
                    disp(' SOAR_Taliro: FALSIFIED!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
                %check if budget has been exhausted
                if sim_count >= nSamples
                    run.nTests = sim_count;
                    disp(' SOAR_Talro: Samples Exhausted!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
            all_x = [all_x; x0_backward];
            all_y = [all_y; f0_b];
       
       % estimate each dimension of gradient 
       for i = 1:nInputs
           gk(i,1) = (f0_f-f0_b)/(2*ck*delta_p(i));
       end
       
       if k ==0 && any(gk)~=0
           if opt.varying_cp_times == 1
               a0 = 0.3*a0*(((A+1)^alpha)/abs(gk(randi([1,nInputs/2],1))));
               a1 = 0.3*a1*(((A+1)^alpha)/abs(gk(randi([nInputs/2+1,nInputs],1))));
           else
               a = 0.3*a*(((A+1)^alpha)/abs(gk(randi([1,nInputs],1))));
           end
       end
       
       % calculate step size and next step
       if opt.varying_cp_times == 1
           ak(1,1:nInputs/2) = ones(1,nInputs/2)*(a0/((A+k+1)^alpha));
           ak(1,nInputs/2+1:nInputs) = ones(1,nInputs/2)*(a1/((A+k+1)^alpha));
       else
           ak = ones(1,nInputs)*(a/((A+k+1)^alpha));
       end
       x1 = x0 - ak.*gk';
       for i=1:nInputs
           if x1(i) <= inpRanges(i,1)
               x1(i) = inpRanges(i,1) + ck;
           elseif x1(i) >= inpRanges(i,2)
               x1(i) = inpRanges(i,2) - ck;
           end
       end
               
       
       % sample next step 
       curSample{1} = x1';
       curVal = Compute_Robustness(curSample);
       f1 = curVal{1};
       sim_count = sim_count + 1;
       
                %store as necessary
                if nargout>1
                    if isa(curVal{1},'hydis')
                        history.cost(sim_count) = hydisc2m(curVal)';
                        history.rob(sim_count) = hydisc2m(curVal)';
                    else
                        history.cost(sim_count) = curVal{1};
                        history.rob(sim_count) = curVal{1}; 
                    end
                    history.samples(sim_count,:) = curSample{1}'; 
                end

                %find and store the best value seen so far
                if isa(curVal{1},'hydis')
                    [minmax_val, minmax_idx] = minmax(hydisc2m(curVal));
                else
                    [minmax_val, minmax_idx] = minmax(cell2mat(curVal));
                end

                if (fcn_cmp(minmax_val,bestCost))
                    bestCost = minmax_val;
                    run.bestCost = minmax_val;
                    run.bestRob = minmax_val;
                    run.bestSample = curSample{minmax_idx};
                    if opt.dispinfo>0
                        if isa(minmax_val,'hydis')
                            disp(['Best ==> <',num2str(get(minmax_val,1)),',',num2str(get(minmax_val,2)),'>']);
                        else
                            disp(['Best ==> ' num2str(minmax_val)]);
                        end
                    end
                end
                %check if best value is falsifying or if , if so, exit as necessary
                if (fcn_cmp(bestCost,0) && StopCond)
                    run.falsified = 1;
                    run.nTests = sim_count;
                    if nargout>1
                        if isa(minmax_val,'hydis')
                            history.cost(sim_count+1:end) = hydis([],[]);
                            history.rob(sim_count+1:end) = hydis([],[]);
                        else
                            history.cost(sim_count+1:end) = [];
                            history.rob(sim_count+1:end) = [];
                        end
                        history.samples(sim_count+1:end,:) = [];
                    end
                    disp(' SOAR_Taliro: FALSIFIED!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
                %check if budget has been exhausted
                if sim_count >= nSamples
                    run.nTests = sim_count;
                    disp(' SOAR_Talro: Samples Exhausted!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
            all_x = [all_x; x1];
            all_y = [all_y; f1];
       
       % calculate indicator variable and test against random threshold
       improvement = (f0-f1)/f0;
       
       if sigmoid(improvement,0,3)>= rand()
           x0 = x1;
           f0 = f1;
           k = k+1;
       else
           stop_cond = 1;
           if f1<f0
               xTrain = [xTrain; x1];
               yTrain = [yTrain; f1];
           else 
               xTrain = [xTrain; x0];
               yTrain = [yTrain; f0];
           end
           display(sim_count);
       end
       
% % below is an alternative criteria to determine local search restart
%        improvement = (f0-f1);
%        range = max(all_y) - min(all_y);
%        adjustment = norminv(.05,0,range/4);
%        indicator = normcdf(improvement,0-adjustment,range/4);
%        
%        if indicator >= rand()
%            x0 = x1;
%            f0 = f1;
%            k = k+1;
%        else
%            stop_cond = 1;
%            if f1<f0
%                xTrain = [xTrain; x1];
%                yTrain = [yTrain; f1];
%            else 
%                xTrain = [xTrain; x0];
%                yTrain = [yTrain; f0];
%            end
%            display(sim_count);
%        end

   end
end
disp(' SOAR_Talro: Samples Exhausted!');
save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
run.nTests = nSamples; 
end


 






















