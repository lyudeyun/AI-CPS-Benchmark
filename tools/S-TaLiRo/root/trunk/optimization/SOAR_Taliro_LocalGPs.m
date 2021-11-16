function [run, history] = SOAR_Taliro_LocalGPs(inpRanges,opt,repNum)
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
crowded_EI_flag = params.crowded_EI_flag;
n_0 = 10*nInputs;   %number of initial design points
dimOK4budget = 1;
if n_0 >= nSamples
    n_0 = max(nSamples - 10, ceil(nSamples/2));
    dimOK4budget = 0;
end
B = 1;     %number of replications per design point, set to 1 for deterministic problem
B_n0_setting=1;

%Crowded EI level set threshold
alpha_lvl_set = params.crowding_threshold; %i.e. EIs within 5% of maxEI

%%parameters for the TR algorithm, user defined
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
    curVal{i} = Compute_Robustness(curSample{i});
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
   all_y = [all_y; curVal{1}];
   
   xTrain = [xTrain;x0];
   yTrain = [yTrain; curVal{1}];
   
   %%%%%%%%%%%%%%%%%%%%% LOCAL SEARCH PHASE %%%%%%%%%%%%%%%%%%%%%%%%%%
   
   % Initialize TR Bounds
   TR_Bounds = [x0'-inpRanges(:,1); inpRanges(:,2)-x0'; (inpRanges(:,2)-inpRanges(:,1))./10];     %%% ATTN: HARD CODED TR_size0 value @.1 probably need a better criteion based on support range of inputs 
   TR_size = min(TR_Bounds);
   % set up first TR with lbs in col 1 and ubs in col 2
   TR = [x0'-TR_size, x0'+TR_size]; 
   
   psoOpt = optimoptions('particleswarm','MaxStallIteration',100,'FunctionTolerance',1e-7);  %can set new psoOptions here   
   
   all_local_x = x0;
   all_local_y = f0;
   
   xTrain_local(1,:) = x0;
   yTrain_local(1,1) = f0;
   m = 1; % count of how many previous local points are in the local training set
   n_0 = 5*nInputs;
   %%%%%%%%%% Enter TR Meta Model Loop %%%%%%%%%%
   
   while (TR_size>0.01*(min((inpRanges(:,2)-inpRanges(:,1))))) && ((sim_count+n_0-m) < nSamples) && dimOK4budget
       
      if n_0 - m > 0 
           % draw a new lhs over the current TR 
           x0_local = lhsdesign(n_0-m,nInputs,'criterion','maximin')';
           x0_local = x0_local.*(TR(:,2)-TR(:,1)) + TR(:,1);
       
           %take extra local samples, check falsification
            for i = 1:n_0-m 
                curSample_local{i} = x0_local(:,i);
                curVal_local{i} = Compute_Robustness(curSample_local{i});
                sim_count = sim_count + 1;

                %store as necessary
                if nargout>1
                    if isa(curVal_local{i},'hydis')
                        history.cost(sim_count) = hydisc2m(curVal_local(i))';
                        history.rob(sim_count) = hydisc2m(curVal_local(i))';
                    else
                        history.cost(sim_count) = curVal_local{i}';
                        history.rob(sim_count) = curVal_local{i}'; 
                    end
                    history.samples(sim_count,:) = curSample_local{i}'; 
                end

                %find and store the best value seen so far
                if isa(curVal_local{1},'hydis')
                    [minmax_val, minmax_idx] = minmax(hydisc2m(curVal_local));
                else
                    [minmax_val, minmax_idx] = minmax(cell2mat(curVal_local));
                end

                if (fcn_cmp(minmax_val,bestCost))
                    bestCost = minmax_val;
                    run.bestCost = minmax_val;
                    run.bestRob = minmax_val;
                    run.bestSample = curSample_local{minmax_idx};
                    if opt.dispinfo>0
                        if isa(minmax_val,'hydis')
                            disp(['Best ==> <',num2str(get(minmax_val,1)),',',num2str(get(minmax_val,2)),'>']);
                        else
                            disp(['Best ==> ' num2str(minmax_val)]);
                        end
                    end
                end

                %check if best value is falsifying, if so, exit as necessary
                if (fcn_cmp(minmax_val,0) && StopCond)
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
                    disp(' SOAR_Taliro: FALSIFIED by local samples!');
                    save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
                    return;
                end
            end

          % add newly drawn points to list of all local points
          all_local_x = [all_local_x; x0_local'];
          all_local_y = [all_local_y; cell2mat(curVal_local)'];
          all_x = [all_x; x0_local'];
          all_y = [all_y; cell2mat(curVal_local)'];

          xTrain_local = [xTrain_local; x0_local'];
          yTrain_local = [yTrain_local; cell2mat(curVal_local)'];

          clear curSample_local curVal_local;
      end
      %Fit Gaussian Process Meta Model Locally
      GPmod_local = OK_Rmodel_kd_nugget(xTrain_local, yTrain_local, 0, 2);  
      
      [xk, bestEI] = particleswarm(@(x)-EIcalc_kd(x,xTrain_local,GPmod_local,yTrain_local),nInputs, TR(:,1), TR(:,2), psoOpt);
   
      %store acquisition function sample appropriately and sample it
       curSample_local{1} = xk';
       curVal_local = Compute_Robustness(curSample_local);
       fk = curVal_local{1};
       sim_count = sim_count + 1;

       %store as necessary
        if nargout>1
            if isa(curVal{1},'hydis')
                history.cost(sim_count) = hydisc2m(curVal_local)';
                history.rob(sim_count) = hydisc2m(curVal_local)';
            else
                history.cost(sim_count) = curVal_local{1};
                history.rob(sim_count) = curVal_local{1}; 
            end
            history.samples(sim_count,:) = curSample_local{1}'; 
        end

        %find and store the best value seen so far
        if isa(curVal{1},'hydis')
            [minmax_val, minmax_idx] = minmax(hydisc2m(curVal_local));
        else
            [minmax_val, minmax_idx] = minmax(cell2mat(curVal_local));
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

      % currently x0 is not in xTrain/yTrain locally... may need to change
      rho=(f0-fk)/(OK_Rpredict(GPmod_local,x0,0,yTrain_local)-OK_Rpredict(GPmod_local,xk, 0, yTrain_local));  
        
      all_x = [all_x; xk];
      all_y = [all_y; fk];
      
      %add EI point to the global set and local set
      xTrain = [xTrain; xk];
      yTrain = [yTrain; fk];
      
      all_local_x = [all_local_x; xk];
      all_local_y = [all_local_y; fk];      
      
%       % exit by the relative improvemnt
%       improvement = (f0-fk)/f0;
%       if sigmoid(improvement,0,3) < rand()
%          if fk < f0
%              x0 = x_candidate;
%              f0 = fk;
%          end
%          break
%       end
      
      % exit by the norm of the step ratio
%       norm_ind = norm(xk-x0)/norm(TR(:,2)-TR(:,1));
      max_indicator = max(abs(xk-x0))/TR_size;
      test = rand();
      if max_indicator < test
         break
      end
      
      %execute RC testing and TR control 
        if(rho<eta0)
            %reject candidate stepsize/centroid
            x0=x0;
            TR_size=TR_size*delta;
            TR = [x0'-TR_size, x0'+TR_size]; 
        else
            if(eta0<rho && rho<eta1)
                %low pass of RC test
                x0=xk;
                valid_bound = [(x0'-inpRanges(:,1));(inpRanges(:,2)-x0');TR_size];
                TR_size = min(valid_bound);
                f0 = fk;
                TR = [x0'-TR_size, x0'+TR_size]; 
            else
                %high pass of RC test
                x0=xk;
                valid_bound = [(x0'-inpRanges(:,1));(inpRanges(:,2)-x0'); TR_size*gamma];
                TR_size = min(valid_bound);
                f0 = fk;
                TR = [x0'-TR_size, x0'+TR_size]; 
            end
        end
        
      % check old local points in new TR, build local training set
      local_in_TR_idx = all([all(all_local_x >= TR(:,1)',2), all(all_local_x <= TR(:,2)',2)],2);
      m = sum(local_in_TR_idx,1);
      
      clear xTrain_local yTrain_local
      xTrain_local(1:m,:) = all_local_x(local_in_TR_idx,:);
      yTrain_local(1:m,:) = all_local_y(local_in_TR_idx,1);
   end
   
   %%% Update the global model training data w/ last centroid sampled %%%
   display(sim_count);
end
disp(' SOAR_Talro: Samples Exhausted!');
run.nTests = nSamples; 
save(['FinGlobalMod_',num2str(repNum)],'GPmod','yTrain')
end
