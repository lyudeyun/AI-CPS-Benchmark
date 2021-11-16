function [run, history] = SOAR_Taliro_FiniteDiff(inpRanges,opt)
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
   
   %%%% Initialize Trust Region Search %%%%   
    %Create the static structure 'a' which contains constraint structure
    %for trust region optimization soltuion to be within the trust region.
     for i = 1:(2*nInputs)
        if i <= nInputs
            for j = 1:nInputs
                if i == j
                    a(i,j) = 1;
                else
                    a(i,j) = 0;
                end
            end
        else
            for j = 1:nInputs
                if (i-nInputs) == j
                    a(i,j) = -1;
                else 
                    a(i,j) = 0;
                end
            end
        end
     end
    %Initialize TR to delta0
    TR_Bounds = [x0'-inpRanges(:,1); inpRanges(:,2)-x0'; (inpRanges(:,2)-inpRanges(:,1))./10];     %%% ATTN: HARD CODED TR_size0 value @.1 probably need a better criteion based on support range of inputs 
    TR_size=min(TR_Bounds);                
%     TR_size=TR_size0;
    
    x_initial = x0;
    new_xk_flag = 1;
    
    %%%%%%%%%%%% Enter into TR Loop %%%%%%%%%%%%%%%%%%
    
    while (TR_size>0.0005*(min((inpRanges(:,2)-inpRanges(:,1)))))
        
        %%%%%% Gradient Initialization %%%%%% 
        %Creates a sampling plan (x) of locations needed for finite differencing   
        if new_xk_flag == 1
            clear x
            for i = 1:(2*nInputs)
                if i <= nInputs
                    for j = 1:nInputs
                        if i == j
                            x(i,j) = x0(1,j)+epsilon; 
                        else
                            x(i,j) = x0(1,j);
                        end
                    end
                else
                    for j = 1:nInputs
                        if (i-nInputs) == j
                            x(i,j) = x0(1,j)+2*epsilon; 
                        else
                            x(i,j) = x0(1,j);
                        end
                    end
                end
            end
            for i = 1:(((nInputs^2+3*nInputs)/2)-(2*nInputs))
                for j = i+1:nInputs
                    vec=zeros(1,nInputs);
                    positions=[i,j];
                    vec(positions)=1;
                    x(size(x,1)+1,:) = x0+epsilon*vec;
                end
            end

            %'sample' finite differencing plan and record data
            for i = 1:(nInputs^2+3*nInputs)/2
                sim_count = sim_count + 1;
                curSample{1} = x(i,:)';
                curVal = Compute_Robustness(curSample);
                f(i,1) = curVal{1};

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

                if i == nInputs
                    G1 = zeros(1,nInputs);
                        for q = 1:nInputs
                            G1(q) = (f(q)-f0)/epsilon;
                        end
                    if norm(G1)<0.05
                        break
                    end
                end

                y(i,1) = curVal{1}; %just to store the finite difference responses
                all_x = [all_x; x(i,:)];
                all_y = [all_y; curVal{1}];
            end

            %%%%%%%% compute jacobian and hessian %%%%%%%%%%%%

            if norm(G1)<0.05
                break
            end

            count = 0;
            for i = 1:nInputs
                for j = i:nInputs
                    if i == j
                        H_UpTri(i,j) = (f(i+nInputs,1) - 2*f(i) + f0)/epsilon^2;
                    else
                        count = count + 1;
                        H_UpTri(i,j) = (f(2*nInputs+count,1) - f(i,1) - f(j,1) +f0)/epsilon^2;
                    end
                end
            end    
            H = (H_UpTri+H_UpTri') - eye(size(H_UpTri,1)).*diag(H_UpTri);
        end
        
    %construct contraints for step size to be within TR  
        b=(TR_size).*ones(2*nInputs,1);
    %optimize step size through minimization of quadratic model
        sk=fmincon(@(s) quadratic_model(s,f0,G1',H),x0,a,b);
%         sk=fmincon(@(s) linear_model(s,f0,G1'),x0,a,b);
        
    %construct rho as the RC "test statistic"  
        curSample{1} = (x0+sk)';
        curVal = Compute_Robustness(curSample);
        fk = curVal{1};
        rho=(f0-fk)/(quadratic_model(zeros(1,nInputs),f0,G1',H)-quadratic_model(sk,f0,G1',H));
%         rho=(f0-fk)/(linear_model(zeros(1,nInputs),f0,G1')-linear_model(sk,f0,G1'));
            sim_count=sim_count+1;
            
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
        
            all_x = [all_x; (x0+sk)];
            all_y = [all_y; curVal{1}];
    
        %Compare trust region ratio with random number for probability of new centroid
        test = rand; 
        if test>(max(abs(sk))/(TR_size/2))
            if fk < f0
                x0 = x0+sk;
                f0 = fk;
            end
            break;
        end
    
            %execute RC testing and TR control 
        if(rho<eta0)
            %reject candidate stepsize/centroid
            x0=x0;
            TR_size=TR_size*delta;
            new_xk_flag = 0;
        else
            if(eta0<rho && rho<eta1)
                disp(['x0 moved. f0 = ',f0, ' and fk = ', fk]);
                disp(TR_size)
                %low pass of RC test
                x0=x0+sk;
                valid_bound = [(x0'-inpRanges(:,1));(inpRanges(:,2)-x0');TR_size];
                TR_size = min(valid_bound);
                f0 = fk;
                new_xk_flag = 1;
            else
                %high pass of RC test
                x0=x0+sk;
                valid_bound = [(x0'-inpRanges(:,1));(inpRanges(:,2)-x0'); TR_size*gamma];
                TR_size = min(valid_bound);
                f0 = fk;
                new_xk_flag = 1;
                disp(['x0 moved. f0 = ',f0, ' and fk = ', fk]);
                disp(TR_size)
            end
        end  
    end
    %%% Update the global model training data w/ last centroid sampled %%%
    if x0 ~= x_initial
        xTrain = [xTrain; x0];
        yTrain = [yTrain; f0];
    end
    display(sim_count);
end
disp(' SOAR_Talro: Samples Exhausted!');
run.nTests = nSamples; 
end


function m = quadratic_model(s,f0,df,hf)
    m =f0+s*df+.5*s*hf*s';
end

function m = linear_model(s,f0,df)
    m =f0+s*df;
end