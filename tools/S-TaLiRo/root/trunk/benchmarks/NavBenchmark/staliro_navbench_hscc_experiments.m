% This is the set of benchmark problems in the report:
% "Falsification of Temporal Properties of Hybrid Systems Using the 
% Cross-Entropy Method." by Sriram Sankaranarayanan and Georgios Fainekos

% (C) Georgios Fainekos 2011 - Arizona State Univeristy

clear

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
form_id = input('Choose a specification to falsify:');

disp(' ')
disp('Total Simulation time:')
if form_id==1
    time = 25
else
    time = 12
end

opt = staliro_options();
opt.runs = 100;
opt.spec_space = 'X';
opt.map2line = 1;

opt.optimization_solver = 'CE_Taliro';
opt.optim_params.n_tests = 1000;

opt.taliro_metric = 'hybrid_inf';
[results, history] = staliro(model,model.init.cube,input_range,cp_array,phi{form_id},Pred,time,opt);

disp(' ')
disp(' See results for for the output of the robustness values of each run')
disp(' See history for the information on all the tests for each run')




