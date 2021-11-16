% This is a demo for using S-TALIRO with models defined as m-functions 

% (C) Sriram Sankaranarayanan (2011) University of Colorado, Boulder, CO

clear
model = @insulinGlucoseODE
disp(' ')
disp('The initial conditions defined as a hypercube:')
init_form_id = input('Enter which initial condition (1 - 3): ');
if (init_form_id == 1)
    init_cond = [0 10;0.07 0.1;-0.1 0.1]
elseif init_form_id == 2
    init_cond = [-1 10; 0.05 0.1; -.1 .1]
elseif init_form_id == 3
    init_cond = [-2 10; 0.05 0.1; -.1 .1]
else
    error('bad choice. ')
end

disp(' ')
disp('The constraints on the input signals defined as a hypercube:')
input_range = [0 3]
disp(' ')
disp('The number of control points for each input signal:')
cp_array=[4];

disp(' ')
disp('The specification:')
phi = '!([]_[0,200.0]a /\ []_[20,200.0] b)'
preds(1).str = 'a';
disp('Type "help monitor" to see the syntax of MTL formulas')
preds(1).A = [1 0 0; -1 0 0];
preds(1).b = [10; 2];
preds(2).str = 'b';
preds(2).A = [1 0 0; -1 0 0];
preds(2).b = [1; 1];

disp(' ')
disp('Total Simulation time:')
time = 200

disp(' ')
disp('Create an staliro_options object with the default options:')
opt = staliro_options()

disp(' 1. Run using Simulated Annealing. ');
disp(' 2. Run using Cross Entropy. ');
disp(' 3. Run with Uniform Random. ');
disp(' 4. Run with GA.');
disp(' ')

form_id=input('Enter options(1-4): ');

if (form_id == 1)
    opt.optimization_solver = 'SA_Taliro';
else
    if (form_id == 2)
    opt.optimization_solver = 'CE_Taliro';
    opt.optim_params.num_iteration=10;
    opt.optim_params.num_subdivs=25;
    else if (form_id == 3)
            opt.optimization_solver = 'UR_Taliro';
			opt.optim_params.n_tests=500;
        else
            opt.optimization_solver='GA_Taliro';
        end
    end
end

opt.runs =2;
opt.spec_space='X';
opt

disp(' ')
disp('Running S-TaLiRo')
tic
[results,history] = staliro(model,init_cond,input_range,cp_array,phi,preds,time,opt);
runtime=toc;

runtime

results.run(results.optRobIndex).bestRob

figure(1)
clf
[T1,XT1,YT1,IT1] = SimFunctionMdl(model,init_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample(:,1),time,opt);
subplot(2,1,1)
hold on
rectangle('Position',[0 -2 20 12],'LineWidth',2,'FaceColor',[.9 0.7 0.7 ])
hold on
rectangle('Position',[20 -1 180 2],'LineWidth',2,'FaceColor',[.7 0.9 0.7 ])
plot(T1,XT1(:,1))
title('State trajectory G')
subplot(2,1,2)
plot(IT1(:,1),IT1(:,2))
title('Input Signal u_1')
% 
% figure(2)
%% plot the samples from the first run


%% Plot the second run
% figure(3)
% clf
% [T2,XT2,YT2,IT2] = SimFunctionMdl(model,[size(init_cond,1) cp_array],samples(2,:),time,opt);
% subplot(2,1,1)
% hold on
% rectangle('Position',[0 -2 20 12],'LineWidth',2,'FaceColor',[.9 0.7 0.7 ])
% hold on
% rectangle('Position',[20 -1 180 2],'LineWidth',2,'FaceColor',[.7 0.9 0.7 ])
% plot(T2,XT2(:,1))
% title('State trajectory G')
% subplot(2,1,2)
% plot(IT2(:,1),IT2(:,2))
% title('Input Signal u_1')
