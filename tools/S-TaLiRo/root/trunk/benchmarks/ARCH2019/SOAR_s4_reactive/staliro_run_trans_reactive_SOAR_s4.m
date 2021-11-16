clear

global SOAR_run_flag
opt = staliro_options();
trans_pred_phi

phi_ind = 4;

opt.optimization_solver = 'SOAR_Taliro_LocalGPs';
opt.runs = 1;
opt.spec_space = 'Y';
opt.map2line = 0;
opt.n_workers = 1;
cp_array = [7,3];

n_tests = 100;
opt.optim_params.n_tests = n_tests; 
init_cond = [];
input_range = [0 100;0 350];
model = @blackbox_autotrans;


outer_runs = 50;
simTime=30;

results_pref = [];
results_suff = [];
n_fals = 0;
opt.seed=1;

opt.loc_traj = 'end';
% opt.taliro_metric = 'hybrid_inf';
opt.black_box = 1;
phi_AF = phi_{phi_ind};
phi_M = phi{phi_ind};
if phi_ind> 2 && phi_ind <7
   opt.taliro_metric = 'hybrid_inf';
end

opt.interpolationtype = {'pconst','pconst'};


for i = 1:outer_runs
    opt.seed=i;
    SOAR_run_flag = i;
    %% Run first for antecedent
    disp(' ')
    disp(['Running S-TaLiRo on a small number of tests ',num2str(n_tests), ' ...'])

    disp(['Specification : ',phi_AF])
    tic
    
%     random(i)=randi([1 2147483647]);
%     opt.seed=random(i);   
    [results1, history1] = staliro(model,init_cond,input_range,cp_array,phi_AF,preds,simTime,opt);
    toc
    
    results_pref = [results_pref results1];
    model1= staliro_blackbox(model);

    if results1.run(results1.optRobIndex).falsified

        % Simulate to get prefix
        XPoint = [];
        UPoint = results1.run(results1.optRobIndex).bestSample';
        [hs,~,inpSig] = systemsimulator(model1, XPoint, UPoint, simTime, input_range, cp_array, opt);
        T1=hs.T;
        YT1=hs.YT;
        LT1=hs.LT;
        [rob1,aux1] = dp_taliro(phi_AF,preds,YT1,T1,LT1,hs.CLG);
        rob2 = dp_t_taliro(phi_AF,preds,YT1,T1,LT1,hs.CLG);
        idx1 = find(inpSig(:,1)>(hs.T(aux1.i)+rob2.pt),1);
        idx2 = find(hs.T>(hs.T(aux1.i)+rob2.pt),1);
        opt1 = opt;
        % Reduce the total number of tests by the number already executed
        opt1.optim_params.n_tests = opt.optim_params.n_tests-results1.run(results1.optRobIndex).nTests;

        if length(opt.interpolationtype)==1
            pref_sig = inpSig(1:idx1,:);
            opt1.interpolationtype = {{pref_sig, opt.interpolationtype{1}}};
        else
            % GF: Not unit tested 
            for ii = 1:length(opt.interpolationtype)
                pref_sig = [inpSig(1:idx1,1) inpSig(1:idx1,ii+1)];
                opt1.interpolationtype{ii} = {pref_sig,  opt.interpolationtype{ii}};
            end
        end

        % Remark: empty initial conditions since we need to fix to the initial 
        % conditions from the previous search
%         model.init.h0 = XPoint; % pass the previous initial conditions through the model
        disp(' ')
        disp(' ')
        disp(['Specification : ',phi_M])
%         random1(i)=randi([1 2147483647]);
%         opt1.seed=random1(i);   
        [results2, history2] = staliro(model,init_cond,input_range,cp_array,phi_M,preds,simTime,opt1);

        results2.run.nTests = results2.run.nTests+results1.run(results1.optRobIndex).nTests;
        results_suff = [results_suff results2];
        
        if results2.run.falsified
            n_fals = n_fals+1;
        end

        disp(' ')
        disp('**************************************************************')
        disp([' Number of falsifications : ',num2str(n_fals),'/',num2str(i)])
        disp('**************************************************************')
        disp(' ')
        
    end
        runtimes(i)=toc;      
    
end
save('SOAR_Trans_s4_Reactive_Arch19Bech','results_suff','results_pref','runtimes','n_fals')




