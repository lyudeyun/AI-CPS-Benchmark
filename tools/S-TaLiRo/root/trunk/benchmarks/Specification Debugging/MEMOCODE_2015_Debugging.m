% This is a demo for finding incorrect STL/MITL specifications represented 
% in the MEMOCODE 2015 conference paper, Table III:
%
%
% Dokhanchi, et al. "Metric Interval Temporal Logic Specification
% Elicitation and Debugging",  MEMOCODE 2015
%
%
% (C) Adel Dokhanchi, 2018, Arizona State University

%% Display and user feedback
disp(' ')
disp('This is the set of incorrect specifications that are detected using the ')
disp('STL/MITL debugging tool as presented in MEMOCODE 2015 paper, "Metric ')
disp('Interval Temporal Logic Specification Elicitation and Debugging", Table III.')
disp(' ')
disp('WARNING: Finding logical inconsistencies needs MITL/LTL satisfiability');
disp('         solvers. For more information about installing MITL/LTL ');
disp('         satisfiability solvers run: ');
disp('         >> help setup_vacuity');
disp(' ');
disp('Have you installed MITL/LTL satisfiability solvers on your system?');
answer = input('Choose yes/no answer -> 1) Yes 0) No   :');
if answer ~= 1
	return;
end
disp(' ');
iform = 1;
    phi_nat{iform} = 'At some point in time in the first 30 seconds, vehicle speed will go over 100 and stay above for 20 seconds.';
    phi{iform} = '<>_[0,30] p1 /\ <>_[0,20] p1 '; 

iform = 2;
    phi_nat{iform} = 'At some point in time in the first 30 seconds, vehicle speed will go over 100 and stay above for 20 seconds.';
    phi{iform} = '<>_[0,30] (p1 -> []_[0,20] p1 )';

iform = 3;
    phi_nat{iform} = ['If, at some point in time in the first 40 seconds, vehicle speed goes over 80, then from that point on, if',char(10),...
      '           within the next 20 seconds the engine speed goes over 4,000, then, for the next 30 seconds, the vehicle speed',char(10),...
      '           should be over 100.'];
    phi{iform} = ' <>_[0,40]((( p1 \/ p3 ) ->  <>_[0,40] p2 ) /\ []_[0,30] p1 ) ';
    

iform = 4;
    phi_nat{iform} = 'At every point in time in the first 40 seconds, vehicle speed will go over 100 in the next 10 seconds.';
    phi{iform} = ' []_[0,40] p1 /\ []_[0,40]<>_[0,10] p1 ';

iform = 5;
    phi_nat{iform} = ['If, at some point in time in the first 40 seconds, vehicle speed goes over 80, then from that point on, if',char(10),...
      '           within the next 20 seconds the engine speed goes over 4,000, then, for the next 30 seconds, the vehicle speed',char(10),...
      '           should be over 100.'];
    phi{iform} = ' <>_[0,40] ( p1 \/ p3 ) /\ <>_[0,40] p2 /\ <>_[0,40][]_[0,30] p1  ';

    
  %% Predicates  [speed rpm]
Pred(1).str = 'p1';%speed>=100 
Pred(1).A = [-1.0 0];
Pred(1).b = 100;

Pred(2).str = 'p2';% rpm>=4000
Pred(2).A = [0 -1.0];
Pred(2).b = 4000;

Pred(3).str = 'p3';%100>=speed>=80 
Pred(3).A = [1.0 0;
            -1.0 0];
Pred(3).b = [100; -80];

disp('The set of specifications for the STL/MITL debugging:')
for j = 1:iform
    disp(['   ',num2str(j),'. NAT: ',phi_nat{j}])
    disp(['      MITL: ',phi{j}])
    disp(' ')
end

form_id = input('Choose a specification for debugging:');
disp(' ')

opt = staliro_options();
opt.vacuity_param.use_LTL_satifiability=0;

disp('**************');
disp('Validity testing of');
disp(phi{form_id});
disp('**************');
    [result]=stl_debug(phi{form_id},Pred,opt,'validity');
    if isempty(find(result))==0
        return
    end
disp('**************');
disp('Redundancy testing of');
disp(phi{form_id});
disp('**************');
    [result]=stl_debug(phi{form_id},Pred,opt,'redundancy');
    if isempty(find(result))==0
        return
    end
disp('**************');
disp('Vacuity testing of');
disp(phi{form_id});
disp('**************');
    [result]=stl_debug(phi{form_id},Pred,opt,'vacuity');

