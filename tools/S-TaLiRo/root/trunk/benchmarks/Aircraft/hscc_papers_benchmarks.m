% This is the set of benchmark problems for HSCC 2010 and 2012 papers

% (C) Georgios Fainekos 2012 - Arizona State Univeristy

clear

model = @aircraftODE;

disp(' ')
disp('The initial conditions defined as a hypercube:')
disp('  1. [200 260;-10 10;120 150] (HSCC 2012 benchmark problems)')
disp('  2. [207 260;-10 10;120 150] (HSCC 2010 benchmark problems)')
iinit = input('Select signal input interpolation method:');
if iinit==1
    init_cond = [200 260;-10 10;120 150];
elseif iinit==2
    init_cond = [207 260;-10 10;120 150];
else
    error('Option not supported')
end

ispec = 1;
phi{ispec} = '!([]_[.5,1.5]a /\ <>_[3,4] b)';
numsamp(ispec) = 500;
ispec = ispec+1;
phi{ispec} = '!([]_[0,4]a /\ <>_[3.5,4] d)';
numsamp(ispec) = 1000;
ispec = ispec+1;
phi{ispec} = '!(<>_[1,3] e)';
numsamp(ispec) = 2000;
ispec = ispec+1;
phi{ispec} = '!(<>_[.5,1]a /\ []_[3,4] g)';
numsamp(ispec) = 2500;
ispec = ispec+1;
phi{ispec} = '!([]_[0,.5]h)';
numsamp(ispec) = 2500;
ispec = ispec+1;
phi{ispec} = '!([]_[2,2.5]i1)';
numsamp(ispec) = 2500;
ispec = ispec+1;
phi{ispec} = '!([]_[2,2.5]i3)';
numsamp(ispec) = 2500;

disp(' ')
disp(' Benchmark problems')
for ii = 1:ispec
    disp([num2str(ii),'. ',phi{ii},'; Number of samples: ',num2str(numsamp(ii))]);
end
ibench = input('Choose a benchmark problem:');

ipred = 1;
preds(ipred).str = 'a';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [250; -240];
ipred = ipred+1;
preds(ipred).str = 'b';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [240; -230];
ipred = ipred+1;
preds(ipred).str = 'd';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [240.1; -240];
ipred = ipred+1;
preds(ipred).str = 'e';
preds(ipred).A = [-1 0 0];
preds(ipred).b = -260;
ipred = ipred+1;
preds(ipred).str = 'g';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [280; -270];
ipred = ipred+1;
preds(ipred).str = 'h';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [210; -190];
ipred = ipred+1;
preds(ipred).str = 'i1';
preds(ipred).A = [1 0 0; -1 0 0];
preds(ipred).b = [200; -190];
ipred = ipred+1;
preds(ipred).str = 'i3';
preds(ipred).A = [0 0 1; 0 0 -1];
preds(ipred).b = [200; -190];

disp(' ')
disp('Total Simulation time:')
time = 4

opt = staliro_options();
opt.runs = 100;
opt.spec_space = 'X';

disp(' ')
disp(' 1. Simulated Annealing with Monte Carlo Sampling')
disp(' 2. Uniform Random Sampling')
disp(' 3. Cross Entropy ')
disp(' 4. All above the methods')
search_id = input('Choose a search algorithm:');

input_range = [34386 53973;0 16];
cp_array=[10 10];

disp(' ')
disp('  1. pchip - Piecewise Cubic Hermite Interpolating Polynomial (HSCC 2012 benchmark problems)')
disp('  2. pconst - Piecewise Constant Input Signals (HSCC 2010 benchmark problems)')
iinter = input('Select signal input interpolation method:');

if iinter==1
    opt.interpolationtype={'pchip'};
elseif iinter==2 
    opt.interpolationtype={'pconst'};
else
    error('Interpolation option not supported')
end

file_name = ['air_bench_',num2str(ibench),'_inter_',num2str(iinter),'_init_',num2str(iinit),'_search'];

if search_id==1 || search_id==4
    opt.optimization_solver='SA_Taliro';
	opt.optim_params.n_tests = numsamp(ibench);
    disp(' ')
    disp('Running S-TaLiRo with Monte Carlo ...')
    tic
    [results, history] = staliro(model,init_cond,input_range,cp_array,phi{ibench},preds,time,opt);
    search_time_1 = toc
    save([file_name,'_1'], 'results', 'history');
end

if search_id==2 || search_id==4
    opt.optimization_solver='UR_Taliro';
	opt.optim_params.n_tests = numsamp(ibench);
    disp(' ')
    disp('Running S-TaLiRo with Uniform Random Sampling ...')
    tic
    [results, history] = staliro(model,init_cond,input_range,cp_array,phi{ibench},preds,time,opt);
    search_time_2 = toc
    save([file_name,'_1'], 'results', 'history');
end

if search_id==3 || search_id==4
    opt.optimization_solver='CE_Taliro';
	opt.optim_params.num_iteration = numsamp(ibench)/opt.ce_paramas.num_samples_per_iteration;
    disp(' ')
    disp('Running S-TaLiRo with Uniform Random Sampling ...')
    tic
    [results, history] = staliro(model,init_cond,input_range,cp_array,phi{ibench},preds,time,opt);
    search_time_3 = toc
    save([file_name,'_1'], 'results', 'history');
end
