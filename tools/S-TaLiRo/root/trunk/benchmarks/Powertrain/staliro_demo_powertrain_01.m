% Powertrain benchmark for S-Taliro
%
% The model is described in detail in the technical report:
% Alongkrit Chutinan and Kenneth R. Butts, Dynamic Analysis of Hybrid 
% System Models for Design Validation

% (C) Yashwanth Annapureddy - 2011 - Arizona State University 
% (C) Georgios Fainekos - 2011 - Arizona State University 
% Last update: 2011.09.20

clear

model = @BlackBoxPowertrain01;

disp(' ')
disp('Initial conditions')
init_cond = [0 100; 0 0.5];

disp(' ')
disp('No input signals:')
input_range = []
cp_array = [];

disp(' ')
disp('The specification:')

i = 1;
pred(i).str = 'gear1';
pred(i).A = [];
pred(i).b = [];
pred(i).loc = 1;

i = i+1;
pred(i).str = 'gear2';
pred(i).A = [];
pred(i).b = [];
pred(i).loc = 3;

i = i+1;
pred(i).str = 'gear12';
pred(i).A = [];
pred(i).b = [];
pred(i).loc = 2;

i = i+1;
pred(i).str = 'gear21';
pred(i).A = [];
pred(i).b = [];
pred(i).loc = 4;


disp(' ')
disp(' ')
disp('Create an staliro_options object with the default options:')
opt = staliro_options()

disp (' Select a solver to use ')
disp (' 1. Simulated Annealing ')
disp (' 2. Cross Entropy  ' )
disp (' 3. Uniform Random ' )
disp (' 4. Genetic Algorithm')
disp (' ')
form_id2 = input ('Select an option (1 -4): ')
if (form_id2 == 1)
    opt.optimization_solver = 'SA_Taliro';
else
    if (form_id2 == 2)
    opt.optimization_solver = 'CE_Taliro';
    else if (form_id2 == 3)
            opt.optimization_solver = 'UR_Taliro';
        else
            opt.optimization_solver='GA_Taliro';
        end
    end
end
opt.optim_params.n_tests = 1000;

disp(' ')
disp(' 1. phi_e1 = ! <>(gear2 /\ <>(gear1 /\ <>gear2))')
disp(' 2. phi_e2 = []((!gear1 /\ X gear1) -> []_[0,2.5]!gear2)')
form_id = input('Choose a specification to falsify:');

phi{1} = '! <>(gear2 /\ <>(gear1 /\ <>gear2))';
phi{2} = '[]((!gear1 /\ X gear1) -> []_[0,2.5]!gear2)';

disp(' ')
disp('Total Simulation time:')
time = 60

disp(' ')
disp('Change options:')
opt.runs = 25;
opt.black_box = 1;
opt.spec_space = 'X';
opt.taliro_metric = 'hybrid';
opt.map2line = 0;
opt.taliro = 'dp_taliro';

opt

disp(' ')
disp('Running S-TaLiRo ...')
tic
[results, history] = staliro(model,init_cond,input_range,cp_array,phi{form_id},pred,time,opt);
toc

results.run(results.optRobIndex).bestRob
results.run(results.optRobIndex).time
results.run(results.optRobIndex).nTests

[T XT YT LT] = model(results.run(results.optRobIndex).bestSample,results.run(results.optRobIndex).time);
plot_powertrain(T,[XT LT])




