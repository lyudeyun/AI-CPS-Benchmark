% Powertrain benchmark for S-Taliro
%
% The model is described in detail in the technical report:
% Alongkrit Chutinan and Kenneth R. Butts, Dynamic Analysis of Hybrid 
% System Models for Design Validation

% (C) Yashwanth Annapureddy - 2011 - Arizona State University 
% (C) Georgios Fainekos - 2011 - Arizona State University 
% Last update: 2011.09.20

clear

model = @BlackBoxPowertrain02;

disp(' ')
disp('Initial conditions')
init_cond = [0 100; 0 0.5]

disp(' ')
disp('No input signals:')
input_range = []
cp_array = [];

disp(' ')
disp('The specification:')

% CONSTANTS USED
Rsi = 0.2955;       
Rci = 0.6379;       
Rcr = 0.7045;       
Rd = 0.3521;        

R1 = Rci*Rsi/(1-Rci*Rcr); 
R2 = Rci;                 

AR2 = 4.125;
c2_mu1 = 0.1316;          
c2_mu2 = 0.0001748;

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
pred(i).str = 'b1';
pred(i).A = [0 0 0 0 0 0 1];
pred(i).b = 50;
pred(i).loc = 1:4;

i = i+1;
pred(i).str = 'b2';
pred(i).A = [0 0 0 0 0 0 1];
pred(i).b = 250;
pred(i).loc = 1:4;

i = i+1;
pred(i).str = 'b3';
pred(i).A = [0 0 0 0 0 0 1];
pred(i).b = 450;
pred(i).loc = 1:4;

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

% eq32
i = i+1;
C1 = [0 0 0 1 -1/R2 0 0 ];
d1 = 0.5;
pred(i).str = 'e32';
pred(i).A = C1;
pred(i).b = d1;
pred(i).loc = [2 3];

% e41a
i = i+1;
pred(i).str = 'e41a';
pred(i).A = [0 0 0 -(1 - R1/R2) 0 0 0; ...
    0 0 c2_mu1*AR2 0 0 c2_mu2*(1-R1/R2)*AR2 0];
pred(i).b = [0; ... 
    1];
pred(i).loc = [4 1];

% e41b
i = i+1;
pred(i).str = 'e41b';
pred(i).A = [0 0 0 -(1 - R1/R2) 0 0 0; ...
    0 0 0 (1 - R1/R2) 0 0 0; ...
    0 0 c2_mu1*AR2 0 0 -c2_mu2*(1-R1/R2)*AR2 0];
pred(i).b = [0.5;...
    0; ...
    1];
pred(i).loc = [4 1];

% e41c
i = i+1;
pred(i).str = 'e41c';
pred(i).A = [0 0 0 (1 - R1/R2) 0 0 0; ...
    0 0 -c2_mu1*AR2 0 0 c2_mu2*(1-R1/R2)*AR2 0];
pred(i).b = [-0.5; ...
    1];
pred(i).loc = [4 1];

phi = ' [] (gear21 -> b3)'

disp('Type "help taliro" to see the syntax of MTL formulas')

disp(' ')
disp('Total Simulation time:')
time = 60

disp(' ')
disp('Create an staliro_options object with the default options:')
opt = staliro_options()

disp(' ')
disp('Change options:')

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
            opt.optimization_solver = 'GA_Taliro';
        end
    end
end
opt.optim_params.n_tests = 1000;

opt.runs = 25;
opt.black_box = 1;
opt.spec_space = 'X';
opt.rob_scale = 500;
opt.taliro = 'dp_taliro';
opt.taliro_metric = 'hybrid';
opt.map2line = 0;

disp(' ')
disp('Running S-TaLiRo ...')
tic
[results, history] = staliro(model,init_cond,input_range,cp_array,phi,pred,time,opt);
toc

results.run(results.optRobIndex).bestRob
results.run(results.optRobIndex).time
results.run(results.optRobIndex).nTests

[T XT YT LT] = model(results.run(results.optRobIndex).bestSample,results.run(results.optRobIndex).time);

plot_powertrain(T,[XT LT])
figure
plot(XT(:,end))




