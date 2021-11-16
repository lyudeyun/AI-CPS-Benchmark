% This is the set of benchmark problems in the report:
% "Falsification of Temporal Properties of Hybrid Systems Using the 
% Cross-Entropy Method." by Sriram Sankaranarayanan and Georgios Fainekos

% (C) Georgios Fainekos 2011 - Arizona State Univeristy

clear
rng(2)
model='modulator_3rd_order';
init_cond = [-.1 .1;-.1 .1;-.1 .1];
input_range{1} = [-.45 .45];
input_range{2} = [-.4 .4];
input_range{3} = [-.35 .35];
cp_array = [10];
phi = '[]a';

preds.str='a';
preds.A = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
preds.b = [1 1 1 1 1 1]';

time = 9;
opt = staliro_options();
opt.runs = 5;
n_tests=1000;
opt.spec_space='X';

disp(' ')
disp(' 1. Simulated Annealing with Monte Carlo Sampling')
disp(' 2. Uniform Random Sampling')
disp(' 3. Cross Entropy ')
disp(' ')
search_id = input('Choose a search algorithm:');

if search_id==1
    opt.optimization_solver='SA_Taliro';
elseif search_id==2
    opt.optimization_solver='UR_Taliro';
elseif search_id==3 
    opt.optimization_solver='CE_Taliro';
end
opt.optim_params.n_tests = n_tests;

disp(' ')
disp('   1 . [-.45 .45]')
disp('   2 . [-.4 .4]')
disp('   3 . [-.35 .35]')
disp(' ')
form_id = input('Choose the input range for the system:');
tic
[results, history] = staliro(model,init_cond,input_range{form_id},cp_array,phi,preds,time,opt);
toc
