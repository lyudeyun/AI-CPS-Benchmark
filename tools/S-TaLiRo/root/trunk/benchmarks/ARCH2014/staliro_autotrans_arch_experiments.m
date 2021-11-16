% This is a demo for the benchmarks presented in the ARCH 2014 workshop
%
% We present the Automatic Transmission Benchmark and the Fault-Tolerant
% Fuel Control System benchmark with various specifications
%
% (C) Bardh Hoxha, 2014, Arizona State University

clear

%% Display and user feedback
disp(' ')
disp('This is the set of benchmark problems and specifications ')
disp('as presented in the ARCH 2014 benchmarks paper.')
disp(' ')
disp('Press any key to continue')

pause

nform = 0;
nform  = nform+1;
phi_nat{nform} = 'The engine speed never reaches omega';
phi{nform} = '[]p1';

nform  = nform+1;
phi_nat{nform} = 'The engine and vehicle speed never reach omega and v, resp.';
phi{nform} = '[](p1 /\ p2)';

nform  = nform+1;
phi_nat{nform} = 'There should be no transition from gear two to gear one and back to gear two in less than 2.5 sec.';
phi{nform} = '[]_[0,27.5]( (!gear1 /\ X gear1) -> []_(0,2.5](gear1))';

nform  = nform+1;
phi_nat{nform} = 'There should be no transition from gear two to gear one to any other gear in less than 2.5 sec.';
phi{nform} = '[]_[0,27.5]( (!gear1 /\ X gear1) -> []_(0,2.5](!(gear2 \/ gear3 \/ gear4)))';

nform  = nform+1;
phi_nat{nform} = 'When shifting into any gear, there should be no shift from that gear to any other gear within 2.5sec.';
phi{nform} = '[]_[0,27.5]( (!gear1 /\ X gear1) -> []_(0,2.5](!(gear2 \/ gear3 \/ gear4))) /\ []_[0,27.5]( (!gear2 /\ X gear2) -> []_(0,2.5](!(gear1 \/ gear3 \/ gear4))) /\ []_[0,27.5]( (!gear3 /\ X gear3) -> []_(0,2.5](!(gear1 \/ gear2 \/ gear4))) /\ []_[0,27.5]( (!gear4 /\ X gear4) -> []_(0,2.5](!(gear1 \/ gear2 \/ gear3)))';

nform  = nform+1;
phi_nat{nform} = 'If the engine speed is always less than omega, then the vehicle speed can not exceed v in less than 10 sec.';
phi{nform} = '[](p1) -> []_[0,10](p2)';

nform  = nform+1;
phi_nat{nform} = 'Within 10 sec. the vehicle speed is less than v and from that point on the engine speed is always less than omega.';
phi{nform} = '<>_[0,10]p3 -> []p1';

nform  = nform+1;
phi_nat{nform} = 'A gear increase from first to fourth in under 10secs, ending in an RPM above omega within 2 seconds of that, should result in a vehicle speed above v.';
phi{nform} = '( (gear1 U gear2 U gear3 U gear4) /\ <>_[0,10](gear4 /\ <>_[0,2]p7) ) -> <>_[0,10]((gear4 U_[0,1]p3))';

nform  = nform+1;
phi_nat{nform} = 'The fuel fow rate should not be 0 for more than 1 sec within the next 100 sec. period';
phi{nform} = '!( <>_[0,99] ( []_[0,1] p4 ) )';

nform  = nform+1;
phi_nat{nform} = 'Always, if the air-to-fuel ratio output goes out of bounds, then within 1 sec it should settle inside the bounds and stay there for a sec.';
phi{nform} = '[]((p5 /\ p6) -> <>_[0,1] []_[0,1] !(p5 /\ p6))';

disp('The set of specifications for the Automatic Transmission model:')
for j = 1:nform
    if j == 9
        disp(' ')
        disp('The set of specifications for the Fault-Tolerant Fuel Control System model:')
    end
    disp(['   ',num2str(j),'. NAT: ',phi_nat{j}])
    disp(['      MTL: ',phi{j}])
    disp(' ')
end
form_id_1 = input('Choose a specification to falsify:');
disp(' ')

omega = [];
v = [];

switch form_id_1
    case 1
        omega = input('Provide a value for omega(default: 4500):');
        if isempty(omega)
            omega = 4500;
        end
        if isempty(v)
            v = 160;
        end
    case 2
        omega = input('Provide a value for omega(default: 4500):');
        v = input('Provide a value for v(default: 160):');
        if isempty(omega)
            omega = 4500;
        end
        if isempty(v)
            v = 160;
        end
    case 6
        omega = input('Provide a value for omega(default: 4500):');
        v = input('Provide a value for v(default: 85):');
        if isempty(omega)
            omega = 4500;
        end
        if isempty(v)
            v = 85;
        end
    case 7
        omega = input('Provide a value for omega(default: 4500):');
        v = input('Provide a value for v(default: 80):');
        if isempty(omega)
            omega = 4500;
        end
        if isempty(v)
            v = 80;
        end
    case 8
        omega = input('Provide a value for omega(default: 3800):');
        v = input('Provide a value for v(default: 120):');
        if isempty(omega)
            omega = 3800;
        end
        if isempty(v)
            v = 120;
        end
    otherwise
        if isempty(omega)
            omega = 4200;
        end
        if isempty(v)
            v = 160;
        end
end
%%
opt = staliro_options();
opt.runs = 1;
opt.spec_space = 'Y';
opt.map2line = 0;
opt.n_workers = 1;
opt.optimization_solver = 'SA_Taliro';
opt.optim_params.n_tests = 2000;

init_cond = [];

if form_id_1 == 3 || form_id_1 == 4 || form_id_1 == 5 || form_id_1 == 8
    %options settings and predicate definitions for spec. 3 for the
    %Automatic Transmission model that utilizes the hybrid distance metric
    
    disp(' ')
    disp('Remark: These benchmarks utilize the hybrid_inf metric. ')
    disp('        In general, it is recommended to use the hybrid distance metric in')
    disp('        cases where the guards for the transitions between locations is known.')
    disp(' ')
    disp('For a demo with the hybrid distance metric see \benchmarks\powertrain\staliro_demo_powertrain_01.m')
    disp(' ')
    disp('Press any key to continue')
    
    pause
    
    input_range = [0 100;0 500];
    model = @BlackBoxAutotrans;
    
    cp_array = [7,3];
    
    opt.loc_traj = 'end';
    opt.taliro_metric = 'hybrid_inf';
    %opt.taliro = 'dp_ht_taliro';
    opt.taliro = 'dp_taliro';
    opt.black_box = 1;
    opt.interpolationtype = {'pchip'};
    
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
    
    %predicate definitions
    ii = ii+1;
    preds(ii).str='p1';
    preds(ii).A = [0 1];
    preds(ii).b = omega;
    preds(ii).loc = [1:4]; %#ok<*NBRAK>
    
    
    ii = ii+1;
    preds(ii).str='p2';
    preds(ii).A = [1 0];
    preds(ii).b = v;
    preds(ii).loc = [1:4];
    
    ii = ii+1;
    preds(ii).str='p3';
    preds(ii).A = [-1 0];
    preds(ii).b = -1*v;
    preds(ii).loc = [1:4];
    
    ii = ii+1;
    preds(ii).str='p7';
    preds(ii).A = [0 -1];
    preds(ii).b = -1*omega;
    preds(ii).loc = [1:4];
    
    disp(' ')
    disp('Total Simulation time:')
    time = 30 %#ok<*NOPTS>
    
elseif form_id_1 == 9 || form_id_1 == 10
    %options settings and predicate definitions for spec. 7,8 for the
    %Fault-Tolerant Fuel Control System
    
    open('stoch_fault_tol_fuelsys')
    close_system('stoch_fault_tol_fuelsys')
    
    model='stoch_fault_tol_fuelsys';
    
    input_range = [5 25];
    cp_array = 1;
    
    opt.interpolationtype={'const'};
    opt.taliro = 'dp_t_taliro';
    
    %predicate definitions
    ii = 1;
    preds(ii).str='p4';
    preds(ii).A = [1 0];
    preds(ii).b = 0.1;
    
    ii = ii+1;
    preds(ii).str='p5';
    preds(ii).A = [0 1];
    preds(ii).b = 1.1;
    
    ii = ii+1;
    preds(ii).str='p6';
    preds(ii).A = [0 -1];
    preds(ii).b = -1.1;
    
    disp(' ')
    disp('Total Simulation time:')
    time = 100
    
    
else
    %options settings and predicate definitions for spec. 1,2,4,5,6 for the
    %Automatic Transmission model
    
    input_range = [0 100;0 500];
    model = 'autotrans_mod04';
    
    cp_array = [7,3];
    opt.interpolationtype = {'pchip'};
    opt.taliro = 'dp_taliro';
    
    %predicate definitions
    ii = 1;
    preds(ii).str='p1';
    preds(ii).A = [0 1 0];
    preds(ii).b = omega;
    preds(ii).loc = [1:4];
    
    
    ii = ii+1;
    preds(ii).str='p2';
    preds(ii).A = [1 0 0];
    preds(ii).b = v;
    preds(ii).loc = [1:4];
    
    ii = ii+1;
    preds(ii).str='p3';
    preds(ii).A = [-1 0 0];
    preds(ii).b = -1*v;
    preds(ii).loc = [1:4];
    
    disp(' ')
    disp('Total Simulation time:')
    time = 30
end

warning off %#ok<*WNOFF>
[results,history] = staliro(model,init_cond,input_range,cp_array,phi{form_id_1},preds,time,opt);

disp(' ')
disp(' See results for for the output of the robustness values of each run')
disp(' See history for the information on all the tests for each run')

%% plot the inputs/outputs for which the system does not satisfy the spec.
if form_id_1 == 9 || form_id_1 == 10
    
    disp(' ')
    disp(' NOTE: Since the Fault-Tolerant Fuel Control System model exhibits stochastic behaviour,')
    disp('       simulating the system with the same sample might not necessarily falsify the specification')
    
    [T1,XT1,YT1,IT1] = SimSimulinkMdl('stoch_fault_tol_fuelsys',init_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample,time,opt);
    
    figure
    
    subplot(3,1,1)
    plot(IT1(:,1),IT1(:,2))
    axis([0 100 0 25])
    title('Throttle')
    
    subplot(3,1,2)
    plot(T1(:,1),YT1(:,1))
    title('Fuel-Flow Rate')
    axis([0 100 0 2])
    
    subplot(3,1,3)
    plot(T1,YT1(:,2))
    hold on
    %plot([0 30],[omega omega],'r');
    title('Air-Fuel ratio')
    plot([0 100],[1.1 1.1],'r');
    plot([0 100],[0.9 0.9],'r');
    axis([0 100 0 2])
        
else
    
    [T1,XT1,YT1,IT1] = SimSimulinkMdl('autotrans_mod04',init_cond,input_range,cp_array,results.run(results.optRobIndex).bestSample,time,opt);
    
    figure
    
    subplot(5,1,1)
    plot(IT1(:,1),IT1(:,2))
    axis([0 30 0 100])
    title('Throttle')
    
    subplot(5,1,2)
    plot(IT1(:,1),IT1(:,3))
    title('Break')
    axis([0 30 0 500])
    
    subplot(5,1,3)
    plot(T1,YT1(:,2))
    hold on
    plot([0 30],[omega omega],'r');
    title('RPM')
    axis([0 30 0 6000])
    
    subplot(5,1,4)
    plot(T1,YT1(:,1))
    hold on
    plot([0 30],[v v],'r');
    
    title('Speed')
    axis([0 30 0 200])
    
    subplot(5,1,5)
    plot(T1,YT1(:,3))
    hold on
    title('Gear')
    axis([0 30 0 4])
    
end
warning on %#ok<*WNON>