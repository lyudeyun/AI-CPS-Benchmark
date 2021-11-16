% This is a demo for the benchmarks presented in the ARCH 2014 workshop
%
% We present the Automatic Transmission Benchmark and the Fault-Tolerant
% Fuel Control System benchmark with various specifications
%
% (C) Bardh Hoxha, 2014, Arizona State University

clear

%% Display and user feedback
disp(' ')
disp('This is the set of benchmark problems and specifications ')
disp('as presented in the ARCH 2014 benchmarks paper.')
disp(' ')
disp('Press any key to continue')


pause

phi_nat = 'There should be no transition from gear two to gear one and back to gear two in less than 2.5 sec.';
phi = '[]_[0,25]( (gear2 /\ X gear1) -> []_[0,5]( !p1 -> gear1 ) )';

opt = staliro_options();
opt.runs = 1;
opt.spec_space = 'Y';
opt.map2line = 0;
opt.n_workers = 1;
opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 2000;

init_cond = [];

input_range = [0 100;0 300];
model = @BlackBoxAutotrans05;

cp_array = [5,5];

opt.loc_traj = 'end';
opt.taliro_metric = 'hybrid_inf';
opt.varying_cp_times = 1;
opt.taliro = 'dp_taliro';
opt.black_box = 1;
opt.interpolationtype = {'pchip'};

%predicate definitions
ii = 1;
preds(ii).str = 'gear1';
preds(ii).A = [];
preds(ii).b = [];
preds(ii).loc = 1;

ii = ii+1;
preds(ii).str = 'gear2';
preds(ii).A = [];
preds(ii).b = [];
preds(ii).loc = 2;

ii = ii+1;
preds(ii).str = 'gear3';
preds(ii).A = [];
preds(ii).b = [];
preds(ii).loc = 3;

ii = ii+1;
preds(ii).str = 'gear4';
preds(ii).A = [];
preds(ii).b = [];
preds(ii).loc = 4;

ii = ii+1;
preds(ii).str='p1';
preds(ii).A = -1;
preds(ii).b = -5;
preds(ii).loc = [1:4]; %#ok<*NBRAK>

disp(' ')
disp('Total Simulation time:')
time = 30 %#ok<*NOPTS>

warning off %#ok<*WNOFF>
[results,history] = staliro(model,init_cond,input_range,cp_array,phi,preds,time,opt);

disp(' ')
disp(' See results for for the output of the robustness values of each run')
disp(' See history for the information on all the tests for each run')

[T1,XT1,YT1,IT1] = SimSimulinkMdl('autotrans_mod05',init_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample,time,opt);

figure

subplot(3,1,1)
plot(IT1(:,1),IT1(:,2))
axis([0 30 0 100])
title('Throttle')

subplot(3,1,2)
plot(IT1(:,1),IT1(:,3))
title('Break')
axis([0 30 0 500])

subplot(3,1,3)
plot(T1,YT1(:,2))
hold on
title('Gear')
axis([0 30 0 4])

warning on %#ok<*WNON>
