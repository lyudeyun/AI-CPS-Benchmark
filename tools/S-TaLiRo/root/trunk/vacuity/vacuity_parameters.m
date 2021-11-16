% Class definition for STL/MITL Debugging and Vacuity Aware Falsification
% For more information refer to 
% Dokhanchi, et al. "Vacuity Aware Falsification for MTL Request-Response  
%   Specifications", CASE 2017
% 
% vacuity_param = vacuity_parameters;
%
%
% obj.optimizer_Stage1_VAF;
%   This is the option for specifying the optimization algorithm for Stage 1
%   of the Vacuity Aware Falsification (VAF). In Stage 1 of VAF, s-taliro
%   focuses on falsifying the antecedent failure of the request-response
%   specification.
%
%   Default value: []
%       Use the optimizer specified in staliro_options 
%
%
% obj.number_of_tests;
%   This is the option for setting the total number of tests for stage 1 
%   and stage 2 of the Vacuity Aware Falsification (VAF). 
%
%   Default value: 300
%
%
% obj.use_LTL_satifiability;
%   This is the option for enabling LTL satisfiability if the modified 
%   formula contains Eventually or Always only temporal operation.  
%   If use_LTL_satifiability set to 1, then the LTL satisfiability checker 
%   runs before MITL SAT solver.
%
%   Default value: 0 (false)
%   
%   Note: This option needs NuSMV to be installed in the system. Type  
%   "help setup_vacuity" for more information.
%

% (C) 2018, Adel Dokhanchi, Arizona State University

classdef vacuity_parameters
    
    properties
        optimizer_Stage1_VAF = [];
        use_LTL_satifiability = 0;
        number_of_tests = 300;
        use_prefix_signal = true;
    end
    
    methods        
        function VAF_parameters(varargin)
        end
    end
end

        