% Class definition for uniform random algorithm parameters 
% 
% ur_params = UR_Taliro_parameters
%
% The above code sets the default parameters for uniform random search.
%
% Default values:
%
% n_tests = 1000;			  % Total number of tests
%                             
% To change the default values to user-specified values use the default 
% object already created to specify the properties.                             
%                              
% E.g., to change the number of tests:
%
% SOAR_Taliro_SPSA_parameters_params.n_tests = 10;
% 
% See also: staliro_options, UR_Taliro

% (C) 2013, Bardh Hoxha, Arizona State University
% (C) 2015, Georgios Fainekos, Arizona State University

classdef SOAR_Taliro_SPSA_parameters
    
    properties
		n_tests = 1000;
        crowding_threshold = 0.05; %the top percent of EI values to be inluded in the crowding distance phase
        finite_diff = 1e-12;
        TR_lowpass_thresh = 0.25;
        TR_highpass_thresh = 0.75;
        TR_delta = 0.5; %shrinking rate
        TR_gamma = 1.2; %growth rate
       
        crowded_EI_flag = 1;       
    end
    
    methods        
        function obj = SOAR_Taliro_parameters(varargin)
            if nargin>0
                error(' SOAR_Taliro_parameters : Please access directly the properties of the object.')
            end
        end
    end
end

        