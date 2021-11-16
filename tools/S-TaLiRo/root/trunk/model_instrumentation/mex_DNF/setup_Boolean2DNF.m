function setup_Boolean2DNF(varargin)

path_var=pwd;
addpath(path_var);

skip_mex = 0;
if nargin == 1
    skip_mex = varargin{1};
end

if ~skip_mex
    mex boolean2dnf.cpp;
end


disp('***************************************************************************')
disp('You are all set to use boolean2dnf!')
disp('The function translates propositional formulas into Disjunctive Normal Form.')
disp('***************************************************************************')

