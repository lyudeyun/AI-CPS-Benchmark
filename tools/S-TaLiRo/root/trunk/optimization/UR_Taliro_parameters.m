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
% ur_params.n_tests = 10;
% 
% See also: staliro_options, UR_Taliro

% (C) 2013, Bardh Hoxha, Arizona State University
% (C) 2015, Georgios Fainekos, Arizona State University

classdef UR_Taliro_parameters
    
    properties
		n_tests = 1000;
    end
    
    methods        
        function obj = UR_Taliro_parameters(varargin)
            if nargin>0
                error(' UR_Taliro_parameters : Please access directly the properties of the object.')
            end
        end
    end
end

        