% This is a demo for using S-TALIRO with the Simuquest Enginuity Model
% Bardh Hoxha, Houssam Abbas, Georgios Fainekos. "Using S-TaLiRo on 
% Industrial Size Automotive Models" ARCH 2014

clear
Ts = 0.00025
model = @BlackBoxSimuquest02;

init_cond = []
% inputs throttle, break, road grade
input_range = [0 100; 0 100; 0 40];
cp_array = [7;7;7];

phi = '[]_[3,30]((gear2 /\ X gear1) -> []_[0.00025,2.5]( !p1 -> gear1 ) )'

i = 1;
preds(i).str = 'gear1';
preds(i).A = [];
preds(i).b = [];
preds(i).loc = 1;

i = i+1;
preds(i).str = 'gear2';
preds(i).A = [];
preds(i).b = [];
preds(i).loc = 2;

i = i+1;
preds(i).str = 'p1';
preds(i).A = [0 0 0 -1];
preds(i).b = [-2.5];
preds(i).loc = [1:4];

disp(' ')
disp(' ')
disp('Create an staliro_options object with the default options:')

opt = staliro_options()
opt.runs = 50;
opt.map2line = 0;
opt.taliro_metric='hybrid_inf';
opt.optimization_solver='SA_Taliro';
opt.varying_cp_times = 1;
opt.falsification = 1;
opt.optim_params.n_tests = 500;
opt.loc_traj='end';
opt.black_box = 1;
opt.n_workers = 1;

opt.interpolationtype = {'pchip'};
time = 30;

tic
[results,history] = staliro(model,init_cond,input_range,cp_array,phi,preds,time,opt);
toc

%[T1,XT1,YT1,IT1] = SimSimulinkMdl('Bh_i4_demo_harness_PFI',init_cond,input_range,cp_array,results.run(1).bestSample,time,opt);
%plot(T1, YT1(:,4))