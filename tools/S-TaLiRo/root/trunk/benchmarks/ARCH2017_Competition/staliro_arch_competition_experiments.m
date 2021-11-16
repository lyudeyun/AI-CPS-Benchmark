% This is a demo for the benchmarks presented in the ARCH 2017 competition
%
% We present the Powertrain Benchmark 
%
% (C) Adel Dokhanchi, Bardh Hoxha, Shakiba Yaghoubi, 2017 Arizona State University
warning off
clear
close all
%% Display and user feedback
disp(' ')
disp('This is the set of benchmark problems as presented in ')
disp('the ARCH 2017 competition benchmarks paper.')
disp(' ')
disp('Press any key to continue')

pause

disp(' ')
disp (' Select a reported falsification method ')
disp (' 1. Table 1: General Falsification. ')
disp (' 2. Table 2: Vacuity Aware Falsification. ' )
table_id = input ('Select an option (1-2): ');

disp(' ')
disp (' Select the optimization method ')
if table_id==1
    disp (' 1. Table 1: Uniform Random Sampling (UR). ')
    disp (' 2. Table 1: Simulated Annealing (SA). ')
    disp (' 3. Table 1: Pulse Input Signal using Simulated Annealing (P-SA). ')
    disp(' ')
    opt_id= input ('Select an option (1-3): ');
elseif table_id==2
    disp (' 1. Table 2: Uniform Random Sampling (UR). ' )
    disp (' 2. Table 2: Simulated Annealing (SA). ' )
    disp (' ')
    opt_id= input ('Select an option (1-2): ');
else
    error('Select only 1 or 2');
end


if table_id==1
    tic;
    form_id=2;
    load_specs_and_model;
    %% Falsification parameters options
    opt = staliro_options();
    opt.black_box = 1;
    opt.SampTime = 0.05;
    opt.falsification=1;
    outer_runs = 50;
    opt.runs=outer_runs;
%---------------------------------------------------------------------   
    if opt_id==1
        cp_array = [1;10]; 
        opt.interpolationtype = {'const', 'pconst'};
        opt.optimization_solver = 'UR_Taliro'; 
        input_range = [900  1100; 0 61.1]; 
        opt.seed=2016;
    elseif opt_id==2
        cp_array = [1;10]; 
        opt.interpolationtype = {'const', 'pconst'};
        opt.optimization_solver = 'SA_Taliro';
        opt.optim_params.dispStart= 0.9;
        opt.optim_params.dispAdap = 2;        
        input_range = [900  1100; 0 61.1]; 
        opt.seed=2016;
    elseif opt_id==3
        cp_array = []; 
        pulse='mid_high';% 'mid_low'  for -_- first fall, then rise
                         % 'mid_high' for _-_ first rise, then fall
        opt.optimization_solver = 'SA_Taliro'; 
        opt.optim_params.dispStart= 0.9;
        opt.optim_params.dispAdap = 2;        
        % Parameterization:
        % x1 : the value of the signal1,
        % x2, x4, x6: the values of signal 2, and
        % x3, x5: times of steps.
        opt.interpolationtype = {'const', @PulseInputSignal};
        input_range = {[900  1100]; [0 61.1; 0 simTime; 0 61.1; 0 simTime; 0 61.1]}; 

        %% Input signal

        % Add constraints
%         opt.input_space_constrained = 1; 
        opt.search_space_constrained.constrained=true;
        if strcmp(pulse,'mid_high');
        % t2>t1+5: x5-x3>5 
        A = [0 0 1 0 -1 0];
        b = -5;
        %t1>tau_s : x3>tau_s
        A = [A; [0 0 -1 0 0 0]];
        b = [b; -taus];
        %  x2 < low
        A = [A; [0 1 0 0 0 0]];
        b = [b; low];
        % x4 > high
        A = [A; [0 0 0 -1 0 0]];
        b = [b; -high];
        %  x6 < low
        A = [A; [0 0 0 0 0 1]];
        b = [b; low];
        else
        % t2>t1+5: x5-x3>5
        A = [0 0 1 0 -1 0];
        b = -5;
        %  x2 > high
        A = [A; [0 -1 0 0 0 0]];
        b = [b; -high];
        % x4 < low
        A = [A; [0 0 0 1 0 0]];
        b = [b; low];
        %  x6 > high
        A = [A; [0 0 0 0 0 -1]];
        b = [b; -high];
        end

        % Set constraints
%         opt.input_A = A;
%         opt.input_b = b;
        opt.search_space_constrained.A_ineq= A;
        opt.search_space_constrained.b_ineq= b;

        opt.seed=1208403535;
    else
         error('Select only 1, 2 or 3');
    end
    disp(' ')
    disp(['Specification :' phi{form_id},])
    opt.optim_params.n_tests = 100; % number of tests
    [results,history] = staliro(model,initial_cond,input_range,cp_array,phi{form_id},preds,simTime,opt);

    results.run(results.optRobIndex).bestRob
    total_time=toc;
elseif table_id==2
    %% Falsification parameters options
    form_id = 1;
    load_specs_and_model;
    cp_array = [1;10] ; 
    opt = staliro_options();
    opt.black_box = 1;
    opt.falsification=1;
    opt.SampTime = 0.05;
    if opt_id==1
        opt.optimization_solver = 'UR_Taliro'; 
        load seeds_VAF_UR.mat;
    elseif opt_id==2
        opt.optimization_solver = 'SA_Taliro';
        opt.optim_params.dispStart= 5;
        opt.optim_params.dispAdap = 2;       
        load seeds_VAF_SA.mat;
    else
        error('Select only 1 or 2');
    end%SA_
    opt.spec_space = 'Y';
    opt.interpolationtype={'const','pconst'};
    opt.runs = 1;
    opt.n_workers = 1;
    opt.optim_params.n_tests = 100; % number of tests
    outer_runs = 50;
    n_tests = 100;
    runtimes=zeros(outer_runs,1);
    results_pref = [];
    results_suff = [];
    input_range2=[];
    n_fals = 0;
    model1=staliro_blackbox(model);
    for i = 1:outer_runs
        tic
        %% Run first for antecedent
        disp(' ')

        disp(['Specification : ',phi{form_id}])
    
        opt.seed=random(i);
        cp_array1=[1;5];
        [results1,history1] = staliro(model,initial_cond,input_range,cp_array1,phi{form_id},preds,simTime/2,opt);
    
        results_pref = [results_pref results1];

        if results1.run(results1.optRobIndex).falsified

            % Simulate to get prefix
            XPoint = initial_cond;
            UPoint = results1.run(results1.optRobIndex).bestSample';
            [hs,~,inpSig] = systemsimulator(model1, XPoint, UPoint, simTime/2, input_range, cp_array1, opt);

            [rob1,aux1] = dp_taliro(phi{form_id},preds,hs.YT,hs.T);
            rob2 = dp_t_taliro(phi{form_id},preds,hs.YT,hs.T);
            idx1 = find(inpSig(:,1)>=(hs.T(aux1.i)+rob2.pt),1)+1;

            opt1 = opt;
            % Reduce the total number of tests by the number already executed
            opt1.optim_params.n_tests = opt.optim_params.n_tests-results1.run(results1.optRobIndex).nTests-1;
            ifals=find(UPoint==inpSig(idx1,3))-cp_array(1);
            cp_array2(:,i)=cp_array; 
            cp_array2(2,i)=cp_array(2)-ifals;
        %cp_array2(2,i)=round(cp_array(2)*(sz-idx1)/sz);
             input_range2(:,:,i)=input_range;
             if (cp_array2(2,i)<1);
                 input_range2(2,:,i)=[UPoint(end) UPoint(end)];
                 cp_array2(2,i)=1;
                 i_cut=min(find(inpSig(:,3)==inpSig(idx1,3)));
                 pref_sig = [inpSig(1:i_cut,1) inpSig(1:i_cut,3)];
                opt1.interpolationtype = {'const',{pref_sig,  opt.interpolationtype{2}}};
             else
                 i_cut=max(find(inpSig(:,3)==inpSig(idx1,3)));
                 pref_sig = [inpSig(1:i_cut,1) inpSig(1:i_cut,3)];
                 opt1.interpolationtype = {'const',{pref_sig,  opt.interpolationtype{2}}};
             end       
        % Remark: empty initial conditions since we need to fix to the initial 
        % conditions from the previous search
            disp(' ')
            disp(' ')
            disp(['Specification : ',phi{form_id+1}])
            opt1.seed=random1(i);

            [results2, history2] = staliro(model,[],input_range2(:,:,i),cp_array2(:,i),phi{form_id+1},preds,simTime,opt1);
             
            results2.run.nTests = results2.run.nTests+results1.run(results1.optRobIndex).nTests+1;
            results_suff = [results_suff results2];
        
            if results2.run.falsified
                n_fals = n_fals+1;
            end
      
            disp(' ')
            disp('**************************************************************')
            disp([' Number of falsifications : ',num2str(n_fals),'/',num2str(i)])
            disp('**************************************************************')
            disp(' ')
        
        end
        runtimes(i)=toc;      
    end
else
        error('Select only 1 or 2');    
end
FinalResults;