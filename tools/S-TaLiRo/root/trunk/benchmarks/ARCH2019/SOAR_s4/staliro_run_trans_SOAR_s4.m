clear
% speed->1 RPM->2 gear->3

phi_ind = 4;
trans_pred_phi;

opt = staliro_options();
opt.optimization_solver = 'SOAR_Taliro_LocalGPs';
opt.runs = 50;
opt.spec_space = 'Y';
opt.optim_params.n_tests = 100; %ntest will specifies the rest of tests
opt.seed= 131013014;%randi([1 2147483647]);   
cp_array = [7,3];
opt.loc_traj = 'end';
% opt.taliro_metric = 'hybrid_inf';
opt.black_box = 1;
phi_ = phi{phi_ind};
if phi_ind> 2 && phi_ind <7
   opt.taliro_metric = 'hybrid_inf';
end
init_cond = [];

input_range = [0 100;0 350];
model = @blackbox_autotrans;
tf = 30;
warning off %#ok<*WNOFF>

[results,history] = staliro(model,init_cond,input_range,cp_array,phi_,preds,tf,opt);
save('SOAR_Trans_s4_Arch19Bench','results','history')

