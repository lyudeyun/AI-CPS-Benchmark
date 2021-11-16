% open AbstractFuelControl_M1;
model = @BlackBoxAbstractFuelControl;
% total simulation time
simTime = 50 ; 
% time to start measurement, mainly used to ignore 
measureTime = 1;  
% number of control points, here we use onstant engine speed


fault_time = 100; 
% setting time
eta = 1;
% parameter h used for event definition
h = 0.05;
% parameter related to the period of the pulse signal
zeta_min = 5;
%
C = 0.05;
Cr = 0.1;
Cl = 0.1;
if form_id<=2
    Ut = 0.008;
    input_range = [900  1100; 0 61.1]; 
    low=8.8;
    high=40;
else
    error('Form ID must be less than three');
end
taus = 10 + eta;

% default settings
spec_num = 1; %specification measurement
fuel_inj_tol = 1.0; 
MAF_sensor_tol = 1.0;
AF_sensor_tol = 1.0;
initial_cond = [];%[0 61.1;10 30];
%---------------------------------------------------------------------    
i=0;

i = i+1;
preds(i).str = 'low'; % for the pedal input signal
preds(i).A =  [0 0 1] ;
preds(i).b =  low ;
i = i+1;

preds(i).str = 'high'; % for the pedal input signal
preds(i).A =  [0 0 -1] ;
preds(i).b =  -high ;
i = i+1;
% rise event is represented as low/\<>_(0,h)high
% fall event is represented as high/\<>_(0,h)low
preds(i).str = 'norm'; % mode < 0.5 (normal mode = 0)
preds(i).A =  [0 1 0] ;  
preds(i).b =  0.5 ;
i = i+1;
preds(i).str = 'pwr'; % mode >0.5 (power mode = 1)
preds(i).A =  [0 -1 0] ;
preds(i).b =  -0.5 ;
i = i+1;
preds(i).str = 'utr'; % u<=Ut
preds(i).A =  [1 0 0] ;
preds(i).b =  Ut ;
i = i+1;
preds(i).str = 'utl'; % u>=-Ut
preds(i).A =  [-1 0 0] ;
preds(i).b =  Ut ;
i = i+1;

nform = 0;
nform  = nform+1;
phi{nform} = ['[]_(' num2str(taus) ', inf)(!((low/\<>_(0,' num2str(h) ')high) \/ (high/\<>_(0,' num2str(h) ')low)))'];
nform  = nform+1;   % close-loop pulse response (formula 27)
phi{nform} =['[]_(' num2str(taus) ', inf)(((low/\<>_(0,' ...
            num2str(h) ')high) \/ (high/\<>_(0,' num2str(h) ')low))' ...
            '-> []_[' num2str(eta) ', ' num2str(zeta_min) '](utr /\ utl))'];