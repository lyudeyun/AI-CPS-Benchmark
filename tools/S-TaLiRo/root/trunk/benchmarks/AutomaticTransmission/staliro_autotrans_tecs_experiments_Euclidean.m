% This is the Automatic Transmission (AT) benchmark. This is a modified 
% version of the Automatic Transmission demo that it is distributed with
% Simulink.
%
% This set of benchmarks tests the Euclidean distance metric and compares
% Uniform Random Sampling with Simulated Annealing.

% (C) Georgios Fainekos, 2011, Arizona State University 

clear

disp(' ')
disp('This is the set of Automatic Transmission benchmark problems using ')
disp('the Euclidean distance metric and comparing the Uniform Sampling method')
disp('with the Simulated Annealing optimization method.')
disp(' ')
disp('Press any key to continue')

pause

model='sldemo_autotrans_mod01';
init_cond = []
input_range = [0 100]
cp_array = 7

nform = 0;
nform  = nform+1;
phi{nform} = '!(<>r1 /\ <>r2)' 
nform  = nform+1;
phi{nform} = '!<>(r2 /\ <>_[0,10] r1)' 
nform  = nform+1;
phi{nform} = '!<>(r2 /\ <>_[0,10] r3)' 
nform  = nform+1;
phi{nform} = '!<>(r2 /\ <>_[0,7.5] r1)' 
nform  = nform+1;
phi{nform} = '!<>(r2 /\ <>_[0,5] r1)' 

ii = 1;
preds(ii).str='r1';
preds(ii).A = [-1 0];
preds(ii).b = -120;
preds(ii).loc = [1:7];

ii = ii+1;
preds(ii).str='r2';
preds(ii).A = [0 -1];
preds(ii).b = -4500;
preds(ii).loc = [1:7];

ii = ii+1;
preds(ii).str='r3';
preds(ii).A = [-1 0];
preds(ii).b = -125;
preds(ii).loc = [1:7];

disp(' ')
disp('Total Simulation time:')
time = 30

opt = staliro_options();
opt.runs = 2;
opt.spec_space = 'Y';
opt.map2line = 0;

opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 1000;

disp(' ')
disp('   0 . Run all formulas')
for j = 1:nform
    disp(['   ',num2str(j),' . phi_',num2str(j),' = ',phi{j}])
end
form_id_1 = input('Choose a specification to falsify:');

if form_id_1==0
	for form_id =  1:5
% 		opt.optimization_solver = 'UR_Taliro';
% 		[results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id},preds,time,opt);
		
	[results{form_id}, history{form_id}] = staliro(model,init_cond,input_range,cp_array,phi{form_id},preds,time,opt);
	end
else
%     opt.optimization_solver = 'UR_Taliro';
%     [results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);
    
    [results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);
end

disp(' ')
disp(' See results for for the output of the robustness values of each run')
disp(' See history for the information on all the tests for each run')
