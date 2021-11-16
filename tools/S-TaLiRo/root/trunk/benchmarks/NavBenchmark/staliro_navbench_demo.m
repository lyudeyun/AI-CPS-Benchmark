% This is a demo presents the application of S-TALIRO on the navigation
% benchmark HSCC 04 paper by Fehnker & Ivancic

% (C) Georgios Fainekos 2010 - Arizona State Univeristy

clear

init.loc = 13;
init.cube = [0.2 0.8; 3.2 3.8; -0.4 0.4; -0.4 0.4];
A = [4 2 3 4; 3 6 5 6; 1 2 3 6; 2 2 1 1];

model = navbench_hautomaton(1,init,A);

disp(' ')
disp('The initial conditions defined as a hypercube:')
model.init.loc
model.init.cube

disp(' ')
disp('No input signals')
input_range = []
cp_array=[]

disp(' ')
disp('Propositions:')

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

nform = 0;
nform  = nform+1;
phi{nform } = '(!p11) U p12';
nform  = nform+1;
phi{nform} = '[](!p21 \/ (p22 R (!p23)))';
nform  = nform+1;
phi{nform} = '[](p31 -> [](!p32))';
nform  = nform+1;
phi{nform} = '[](p41 -> [](!p32))';

disp(' ')
for j = 1:nform
    disp(['   ',num2str(j),' . phi_',num2str(j),' = ',phi{j}])
end
form_id = input('Choose a specification to falsify:');

disp(' ')
disp(' 1. Euclidean')
disp(' 2. Hybrid without distance to guards')
disp(' 3. Hybrid with distance to guards')
metric_id = input('Choose a metric to use:');

disp(' ')
disp(' 1. Simulated Annealing with Monte Carlo Sampling')
disp(' 2. Uniform Random Sampling')
disp(' 3. Cross Entropy ')
search_id = input('Choose a search algorithm:');

disp(' ')
disp('Total Simulation time:')
if form_id==1
    time = 25
else
    time = 12
end

disp(' ')
disp('Create an staliro_options object with the default options:')
opt = staliro_options()

disp(' ')
disp('Change options:')
opt.runs = 100;
n_tests = 1000;
opt.spec_space = 'X';
opt.map2line = 0;

if search_id==1
    opt.optimization_solver = 'SA_Taliro';
elseif search_id==2
    opt.optimization_solver = 'UR_Taliro';
elseif search_id == 3
  opt.optimization_solver = 'CE_Taliro';
else
    error('Search option not supported')
end
opt.optim_params.n_tests = n_tests;

if metric_id==1
    opt.taliro_metric = 'none';
elseif metric_id==2
    opt.taliro_metric = 'hybrid_inf';
elseif metric_id==3
    opt.taliro_metric = 'hybrid';
else
    error('Metric option not supported')
end

opt

disp(' ')
disp('Running S-TaLiRo ...')
tic
[results, history] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);
toc

hh = hasimulator(model,[model.init.loc 0 results.run(results.optRobIndex).bestSample(:,1)'],12,'ode45',[1 0 0 0]);
plot(hh(:,3),hh(:,4))

