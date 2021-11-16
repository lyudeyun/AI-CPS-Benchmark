% This is a demo for using S-TALIRO for multiple parameter estimation.
%
% We compare the PE method developed in Hoxha et al. "Querying Parametric Temporal Logic
% Properties in Model Based Design" paper with the PS method developed in
% Jin, Xiaoqing, et al. "Mining requirements from closed-loop control 
% models." Proceedings of the 16th international conference on 
% Hybrid systems: computation and control. ACM, 2013.
%
% (C) Bardh Hoxha, 2014, Arizona State University.

clear

disp(' ')
disp('This is the set of benchmarks for multiple parameter estimation with')
disp('the Parameter Synthesis(PS) algorithm and Parameter Estimation(PE) algorithm')
disp(' ')
disp('Press any key to continue')

pause

model = 'sldemo_autotrans_mod01';
init_cond = [];
input_range = [0 100];
cp_array = 7;

disp(' ')
disp(' ')

nform = 0;
nform  = nform+1;
    phi{nform}= '[](speedp /\ rpm1p)' ;
    disp(['  1. ', phi{nform}]);
nform  = nform+1;
    phi{nform}='!(<>_[0,t1]speedG100 /\  []rpm2p)';
    disp(['  2. ',phi{nform}]);
    disp(' ');
form_id_1 = input('Please select the specification you would like to use:');

disp(' ')
disp('Total Simulation time:')
time = 30

switch form_id_1
    case 1 
        opt.polarity_weight = 'min'; %#ok<*STRNU>
        monotonicity = [1,1];
    case 2
        monotonicity = [-1,-1];
        opt.polarity_weight = 'max';
end

opt = staliro_options();
opt.interpolationtype={'pchip'};
opt.runs = 1;
opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 1000;
opt.taliro = 'dp_taliro';
opt.falsification = 0;
opt.parameterEstimation = 1;
opt.n_workers = 1;
opt.seed = 500;

ii = 1;
preds(ii).str = 'speedp';
preds(ii).par = 'speedPar';
preds(ii).A = [1 0];
preds(ii).b = 100;
preds(ii).value = 100;
preds(ii).range = [0 200];

ii=ii+1;
preds(ii).str='rpm1p';
preds(ii).par = 'rpm1Par';
preds(ii).A = [0 1];
preds(ii).b = 4500;
preds(ii).value = 4500;
preds(ii).range = [0 6000];

ii = ii+1;
preds(ii).par = 't1';
preds(ii).value = 30;
preds(ii).range = [0 60];

ii = ii+1;
preds(ii).par = 't2';
preds(ii).value = 30;
preds(ii).range = [0 60];

ii = ii+1;
preds(ii).par = 't3';
preds(ii).value = 30;
preds(ii).range = [0 60];

ii = ii+1;
preds(ii).par = 't4';
preds(ii).value = 30;
preds(ii).range = [0 60];

ii=ii+1;
preds(ii).str='speedG100';
preds(ii).A = [-1 0];
preds(ii).b = -100;

ii=ii+1;
preds(ii).str='rpmG3500';
preds(ii).A = [0 -1];
preds(ii).b = -3500;

ii=ii+1;
preds(ii).str='speedG120';
preds(ii).A = [-1 0];
preds(ii).b = -120;

ii=ii+1;
preds(ii).str='rpmG3750';
preds(ii).A = [0 -1];
preds(ii).b = -3750;

ii=ii+1;
preds(ii).str='rpm2p';
preds(ii).par = 'rpm2Par';
preds(ii).A = [0 1];
preds(ii).b = 4500;
preds(ii).value = 4500;
preds(ii).range = [0 6000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameter Synthesis Algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

opt.ps_used = 1;

global robCompCounter;
global sysSimCounter;
robCompCounter = 0;
sysSimCounter = 0;

disp('Press any key to run parameter synthesis algorithm')
pause 

tic
[results] = staliro_ps(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt,monotonicity);
ps_time = toc %#ok<*NOPTS>

disp(' ')
disp('Parameters synthesized with parameter synthesis algorithm:')
disp(num2str(results))
disp(' ')
disp('The number of system simulations:')
disp(num2str(sysSimCounter))
disp(' ')
disp('The number of robustness computations:')
disp(num2str(robCompCounter))

ps_robCompCounter = robCompCounter;
ps_sysSimCounter = sysSimCounter;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameter Estimation Algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Press any key to run the parameter estimation algorithm')
pause 

robCompCounter = 0;
sysSimCounter = 0;

pe_robCompCounter = robCompCounter;
pe_sysSimCounter = sysSimCounter;

opt.parameterEstimation = 1;
opt.falsification = 0;
opt.normalization = 1;
opt.seed = 500;
opt.ps_used = 0;

for kk = 1:size(preds,2)
    preds(kk).Normalized = 1;
    preds(kk).NormBounds = 1;
end

tic
[results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);
toc

pe_time = toc

disp(' ')
disp('Parameters estimated with parameter estimation algorithm:')
disp(num2str(results.run.paramVal))
disp(' ')
disp('The number of system simulations:')
disp(num2str(sysSimCounter))
disp(' ')
disp('The number of robustness computations:')
disp(num2str(robCompCounter))
