% This is the set of benchmark problems in the report:
% "Probabilistic Temporal Logic Falsification of Cyber-Physical Systems" 
% H. Abbas, G. Fainekos, S. Sankaranarayanan, F. Ivancic, and A. Gupta,

% (C) Georgios Fainekos 2011 - Arizona State Univeristy
% SOAR Modification: (C) Logan Mathesen 2019 - Arizona State University

clear

addpath('./models/');
rng('default')

%% Initialize Navigation benchmark model and Example specificiations
init.loc = 13;
init.cube = [0.2 0.8; 3.2 3.8; -0.4 0.4; -0.4 0.4];
A = [4 2 3 4; 3 6 5 6; 1 2 3 6; 2 2 1 1];

model = navbench_hautomaton(0,init,A);
input_range = [];
cp_array = [];

i=0;
i=i+1;
disp(' O(p11) = {4} x [3.2,3.8] x [0.2,0.8] x R^2')
Pred(i).str = 'p11';
Pred(i).A = [1 0 0 0; -1 0 0 0; 0 1 0 0; 0 -1 0 0];
Pred(i).b = [3.8; -3.2; 0.8; -0.2];
Pred(i).loc = 10;

i=i+1;
disp(' O(p12) = {8} x [3.2,3.8] x [1.2,1.8] x R^2')
Pred(i).str = 'p12';
Pred(i).A = [1 0 0 0; -1 0 0 0; 0 1 0 0; 0 -1 0 0];
Pred(i).b = [3.8; -3.2; 1.8; -1.2];
Pred(i).loc = 8;

i=i+1;
disp(' O(p21) = {10} x {x in R^4 | x_1>=1.1 }')
Pred(i).str = 'p21';
Pred(i).A = [-1 0 0 0];
Pred(i).b = [-1.1];
Pred(i).loc = 10;

i=i+1;
disp(' O(p22) = {5,6} x {x in R^4 | x_2<=1.05 }')
Pred(i).str = 'p22';
Pred(i).A = [0 1 0 0];
Pred(i).b = [1.05];
Pred(i).loc = [5 6];

i=i+1;
disp(' O(p23) = {9} x {x in R^4 | x_1<=0.9 }')
Pred(i).str = 'p23';
Pred(i).A = [1 0 0 0];
Pred(i).b = [0.9];
Pred(i).loc = 9;

i = i+1;
disp(' O(p31) = {10} x {x in R^4 | x_1>=1.05 /\ x_2>=2}')
Pred(i).str = 'p31';
Pred(i).A = [-1 0 0 0; 0 -1 0 0];
Pred(i).b = [-1.05; -2];
Pred(i).loc = 10;

i=i+1;
disp(' O(p32) = {5} x {x in R^4 | x_1<=1 /\ x_2<=1.95}')
Pred(i).str = 'p32';
Pred(i).A = [1 0 0 0; 0 1 0 0];
Pred(i).b = [1; 1.95];
Pred(i).loc = 5;

i = i+1;
disp(' O(p41) = {10} x {x in R^4 | x_1>=1.2 /\ x_2>=2}')
Pred(i).str = 'p41';
Pred(i).A = [-1 0 0 0; 0 -1 0 0];
Pred(i).b = [-1.2; -2];
Pred(i).loc = 10;

i=i+1;
disp(' O(p42) = {5} x {x in R^4 | x_1<=1 /\ x_2<=1.9}')
Pred(i).str = 'p42';
Pred(i).A = [1 0 0 0; 0 1 0 0];
Pred(i).b = [1; 1.9];
Pred(i).loc = 5;

i = i+1;
disp(' O(p31_1) = {10} x {x in R^4 | x_1>=1.05}')
Pred(i).str = 'p311';
Pred(i).A = [-1 0 0 0];
Pred(i).b = [-1.05];
Pred(i).loc = 10;

i = i+1;
disp(' O(p41_1) = {10} x {x in R^4 | x_1>=1.2}')
Pred(i).str = 'p411';
Pred(i).A = [-1 0 0 0];
Pred(i).b = [-1.2];
Pred(i).loc = 10;

i = i+1;
disp(' O(p31_2) = {10} x {x in R^4 | x_2>=2}')
Pred(i).str = 'p312';
Pred(i).A = [0 -1 0 0];
Pred(i).b = [-2];
Pred(i).loc = 10;

i=i+1;
disp(' O(p32_1) = {5} x {x in R^4 | x_1<=1}')
Pred(i).str = 'p321';
Pred(i).A = [1 0 0 0];
Pred(i).b = 1;
Pred(i).loc = 5;

i=i+1;
disp(' O(p32_2) = {5} x {x in R^4 | x_2<=1.95}')
Pred(i).str = 'p322';
Pred(i).A = [0 1 0 0];
Pred(i).b = 1.95;
Pred(i).loc = 5;

nform = 0;
nform  = nform+1;
phi{nform } = '(!p11) U p12';
nform  = nform+1;
phi{nform} = '[](!p21 \/ (p22 R (!p23)))';
nform  = nform+1;
phi{nform} = '[](p31 -> [](!p32))';
nform  = nform+1;
phi{nform} = '[](p41 -> [](!p32))';
nform  = nform+1;
phi{nform} = '[](p41 -> [](!p42))';
nform  = nform+1;
phi{nform} = '[]((p311/\p312) -> []!(p321/\p322))';
nform  = nform+1;
phi{nform} = '[]((p411/\p312) -> []!(p321/\p322))';
nform  = nform+1;
phi{nform } = '!((!p11) U_[0,25] p12)';

disp(' ')
for j = 1:nform
    disp(['   ',num2str(j),' . phi_',num2str(j),' = ',phi{j}])
end
disp('Note specification 3, 5, and 8 are those utilized in IEEE CASE 2019, Mathesen et. al., "Falisificaiton of CPS with SOAR"')
form_id = input('Choose a specification to falsify:');

disp(' ')
disp('Total Simulation time:')
if form_id==1
    time = 25
else
    time = 12
end

%% Set parameters for SOAR with Local Gaussian Processes
opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_LocalGPs';
opt.runs = 10;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = 100; % Set the length of each marco-replicaitons
opt.spec_space = 'X';
opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;
opt.seed = 12222;               % Set Optimization seed for repeatability
opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'

[results_SOAR_LocalGPs, history_SOAR_LocalGPs] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_Local_GP *****')
short_results_display(results_SOAR_LocalGPs,history_SOAR_LocalGPs);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')

%% Set parameters for SOAR with Finite Differencing Trust Region Local Search

opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_FiniteDiff';
opt.runs = 10;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = 100; % Set the length of each marco-replicaitons
opt.spec_space = 'X';
opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;
opt.seed = 12222;               % Set Optimization seed for repeatability
opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'

[results_SOAR_FiniteDiff, history_SOAR_FiniteDiff] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_FiniteDiff *****')
short_results_display(results_SOAR_FiniteDiff,history_SOAR_FiniteDiff);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')

%% Set parameters for SOAR with single estimation SPSA Trust Region Local Search

opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_SPSA';
opt.runs = 10;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = 100; % Set the length of each marco-replicaitons
opt.spec_space = 'X';
opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;
opt.seed = 12222;               % Set Optimization seed for repeatability
opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'

[results_SOAR_SPSA, history_SOAR_SPSA] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_SPSA *****')
short_results_display(results_SOAR_SPSA,history_SOAR_SPSA);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')


%% Set parameters for SOAR with hessian estimation Trust Region Local Search

opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_2SPSA';
opt.runs = 10;                  % Set algorithmic macro-replications
opt.optim_params.n_tests = 100; % Set the length of each marco-replicaitons
opt.spec_space = 'X';
opt.map2line = 1;               % CRITICAL - SOAR does not support hybrid output spaces currently
opt.rob_scale = 1;              % 'a=1' in map2line ==> robustness of 40+ = 1;
opt.seed = 12222;               % Set Optimization seed for repeatability
opt.taliro_metric = 'hybrid';   % Can alternative be set to 'hybrid_inf'

[results_SOAR_2SPSA, history_SOAR_2SPSA] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ') 
disp('**** Results for SOAR_2SPSA *****')
short_results_display(results_SOAR_2SPSA,history_SOAR_2SPSA);
disp('************************************')
disp(' ') 
input('Press any button to continue to next optimizer')


%% Function to quickly present results from the results/history structure returned by S-TaLiRo
function short_results_display(results,history)
    nT_SOAR=[]; %store number of simulations of falsifying runs
    nR_SOAR=[]; %store lowest robustness of non-falsifying runs
    numOfFals_SOAR=0; 
    Fals_Inps_SOAR=[]; %store the falsifying input of each falsifying run
    outer_runs_SOAR = length(history);

    runtimes_SOAR=[];
    for iii=1:outer_runs_SOAR
        if results.run(iii).falsified==1
            Fals_Inps_SOAR = [Fals_Inps_SOAR; results.run(iii).bestSample'];
            nT_SOAR=[nT_SOAR, results.run(iii).nTests];
            numOfFals_SOAR=numOfFals_SOAR+1;
        else
            nR_SOAR=[nR_SOAR; results.run(iii).bestRob];
        end
        runtimes_SOAR=[runtimes_SOAR;results.run(iii).time];
    end

    disp(['number of falsifications: ',num2str(numOfFals_SOAR),'/', num2str(outer_runs_SOAR)])
    disp(['Average number of runs to falsify: ', num2str(mean(nT_SOAR))])
    disp(['Median number of runs to falsify: ', num2str(median(nT_SOAR))])
end