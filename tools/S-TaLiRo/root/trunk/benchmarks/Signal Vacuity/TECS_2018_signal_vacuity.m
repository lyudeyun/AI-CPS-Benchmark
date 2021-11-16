% This is a demo for the benchmarks presented in the ACM/TECS 2018 Journal 
% paper: 
%
% Dokhanchi, et al. "Formal Requirement Debugging for Testing and  
% Verification of Cyber-Physical Systems", ACM/TECS 2018
%
% We present the Automatic Transmission Benchmark with various
% Request-Response specifications
%
% (C) Adel Dokhanchi, 2018, Arizona State University

clear

%% Display and user feedback
disp(' ')
disp('This is the set of benchmark problems and specifications as presented ')
disp('in the ACM/TECS 2018 Journal paper "Formal Requirement Debugging for ')
disp('Testing and Verification of Cyber-Physical Systems", Table 5, 6:')
disp(' ')

nform = 0;

nform  = nform+1;%\Phi_AT_1
phi_nat{nform} = 'There should be no transition from gear two to gear one and back to gear two in less than 2.5 sec.';
phi{nform} = '[]_[0,27.5]( (gear2 /\ <>_(0,0.04] gear1) \/ []_(0,2.5](!gear2))';

nform  = nform+1;%\Phi_AT_2
phi_nat{nform} = 'After shifting into gear one, there should be no shift from gear one to any other gear within 2.5sec.';
phi{nform} = '[]_[0,27.5]( (!gear1 /\ <>_(0,0.04] gear1) -> []_(0,2.5](gear1))';

nform  = nform+1;%\Phi_AT_3
phi_nat{nform} = 'If the engine speed is always less than 4500, then the vehicle speed can not exceed 85 in less than 10 sec.';
phi{nform} = '[]_[0,30](p1) -> []_[0,10](p2)';

nform  = nform+1;%\Phi_AT_4
phi_nat{nform} = 'Within 10 sec. the vehicle speed is grater than 80 and from that point on the engine speed is always less than 4500.';
phi{nform} = '<>_[0,10](p3 -> []_[0,30]p1)';




disp('The set of specifications for the Automatic Transmission model:')
for j = 1:nform
    disp(['   ',num2str(j),'. NAT: ',phi_nat{j}])
    disp(['      STL: ',phi{j}])
    disp(' ')
end

form_id_1 = input('Choose a specification to falsify:');
disp(' ')


%%
opt = staliro_options();
opt.runs = 1;
opt.spec_space = 'Y';
opt.map2line = 0;
opt.n_workers = 1;
opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 2000;
opt.taliro = 'dp_taliro';
opt.interpolationtype = {'pchip'};

init_cond = [];

    
    
input_range = [0 100;0 500];

    
cp_array = [7,3];
staliro_seeds=[779383877,4,0,23];
    
if form_id_1<3
    opt.taliro_metric = 'hybrid_inf';
    opt.loc_traj = 'end';
    model = @BlackBoxAutotrans;
    opt.black_box = 1;
    %predicate definitions
    ii = 1;
    preds(ii).str = 'gear1';
    preds(ii).A = [];
    preds(ii).b = [];
    preds(ii).loc = 1;
    
    ii = ii+1;
    preds(ii).str = 'gear2';
    preds(ii).A = [];
    preds(ii).b = [];
    preds(ii).loc = 2;
    
    ii = ii+1;
    preds(ii).str = 'gear3';
    preds(ii).A = [];
    preds(ii).b = [];
    preds(ii).loc = 3;
    
    ii = ii+1;
    preds(ii).str = 'gear4';
    preds(ii).A = [];
    preds(ii).b = [];
    preds(ii).loc = 4;
else
    model = 'autotrans_mod';
    opt.black_box = 0;
    ii = 1;
    preds(ii).str='p1';
    preds(ii).A = [0 1 0];
    preds(ii).b = 4500;
    preds(ii).loc = [1:4];
    
    ii = ii+1;
    preds(ii).str='p2';
    preds(ii).A = [1 0 0];
    preds(ii).b = 85;
    preds(ii).loc = [1:4];
    
    ii = ii+1;
    preds(ii).str='p3';
    preds(ii).A = [-1 0 0];
    preds(ii).b = -80;
    preds(ii).loc = [1:4];
end

    
    
    
disp(' ')
disp('Total Simulation time:')
time = 30
    

warning('off','all');
opt.seed=staliro_seeds(form_id_1);
[vacuity,results,history]=signal_vacuity(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);
Vacuous_Signals=length(vacuity.sample_index);
    
disp(' ')
disp('**************************************************************')
disp([' Number of Vacuous Signals/All Signals: ',num2str(Vacuous_Signals),'/',num2str(results.run.nTests)])
disp('**************************************************************')
warning('on','all');