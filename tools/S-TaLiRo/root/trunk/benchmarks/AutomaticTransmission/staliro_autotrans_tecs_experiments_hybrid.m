% This is the Automatic Transmission (AT) benchmark. This is a modified 
% version of the Automatic Transmission demo that it is distributed with
% Simulink.
%
% This set of benchmarks tests the hybrid distance metric and compares
% Uniform Random Sampling with Simulated Annealing.

% (C) Georgios Fainekos, 2011, Arizona State University 

clear

disp(' ')
disp('This is the set of Automatic Transmission benchmark problems using ')
disp('the hybrid distance metric and comparing the Uniform Sampling method')
disp('with the Simulated Annealing optimization method.')
disp(' ')
disp('Press any key to continue')

pause

model=@BlackBoxAutotrans;
init_cond = [];
input_range = [0 100];
cp_array = 7;

nform = 0;
nform  = nform+1;
phi{nform} = '!(<>r13 /\ <>r14 /\ <>r15)' ;
nform  = nform+1;
phi{nform} = '!(<>r10 /\ <>r11 /\ <>r12)' ;
nform  = nform+1;
phi{nform} = '!(<>r7 /\ <>r8 /\ <>r9)' ;

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

jj = 1;
for ii = 3:6
    preds(ii).str=['r',num2str(ii)];
    preds(ii).A = [];
    preds(ii).b = [];
    preds(ii).loc = [jj jj+4 jj+8];
    jj = jj+1;
end

jj = 1;
for ii = 7:9
    preds(ii).str=['r',num2str(ii)];
    preds(ii).A = [-1 0];
    preds(ii).b = -80;
    preds(ii).loc = jj:jj+3;
    jj = jj+4;
end

jj = 1;
for ii = 10:12
    preds(ii).str=['r',num2str(ii)];
    preds(ii).A = [-1 0];
    preds(ii).b = -79.5;
    preds(ii).loc = jj:jj+3;
    jj = jj+4;
end

jj = 1;
for ii = 13:15
    preds(ii).str=['r',num2str(ii)];
    preds(ii).A = [-1 0];
    preds(ii).b = -79;
    preds(ii).loc = jj:jj+3;
    jj = jj+4;
end

disp(' ')
disp('Total Simulation time:')
time = 30;

opt = staliro_options();
opt.runs = 2;
opt.spec_space = 'Y';
opt.taliro_metric='hybrid_inf';
opt.loc_traj='end';
opt.black_box = 1;
opt.map2line = 0;

opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 1000;

disp(' ')
disp('   0 . Run all formulas')
for j = 1:nform
    disp(['   ',num2str(j),' . phi_',num2str(j+5),' = ',phi{j}])
end
form_id_1 = input('Choose a specification to falsify:');

if form_id_1==0
	for form_id =  1:3
		[results{form_id}, history{form_id}] = staliro(model,init_cond,input_range,cp_array,phi{form_id},preds,time,opt);

% 		opt.optimization_solver = 'UR_Taliro';
% 		[results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id},preds,time,opt);		
	end
else
		[results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);

%     opt.optimization_solver = 'UR_Taliro';
%     [rob_ur,runtime_ur,cycles_ur,samples_ur] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);
end

disp(' ')
disp(' See results for for the output of the robustness values of each run')
disp(' See history for the information on all the tests for each run')
