function [value,error1,inform]=mexgenzmvnprb(correl,A,B,abseps,releps,maxpoints,method) 
%MEXGENZMVNPRB Computes multivariate normal probability by Genz' algorithm
%           using randomized Korobov rules or subregion adaptive
%           integration rules or crude Monte Carlo method
%
%  CALL [value,error,inform]=mexgenzmvnprb(correl,A,B,abseps,releps,maxpoints,method);
%
%     CORREL = vector of correlation coefficients; the correlation
%            coefficient in row I column J of the correlation matrix
%            should be stored in CORREL( J + ((I-2)*(I-1))/2 ), for J < I.
%            (Note: If S is a correlation matrix then CORREL = S(find(tril(S,-1)));)
%            The correlation matrix must be positive semidefinite.
%     A	     = vector of lower integration limits.
%     B	     = vector of upper integration limits.
%	       NOTE: any values greater the 37, are considered as infinite values.
%     ABSEPS = absolute error tolerance.
%     RELEPS = relative error tolerance.
%     MAXPTS = maximum number of function values allowed. This 
%            parameter can be used to limit the time. A sensible 
%            strategy is to start with MAXPTS = 1000*N, and then
%            increase MAXPTS if ERROR is too large.
%     ERROR  = estimated absolute error, with 99% confidence level.
%     VALUE  = estimated value for the integral
%     INFORM = termination status parameter:
%            if INFORM = 0, normal completion with ERROR < EPS;
%            if INFORM = 1, completion with ERROR > EPS and MAXPTS 
%                           function vaules used; increase MAXPTS to 
%                           decrease ERROR;
%            if INFORM = 2, N > NMAX or N < 1.
%     METHOD INTEGER, defining the integration method used
%            1 SADAPT Subregion Adaptive integration method , NMAX = 20
%            2 KROBOV Randomized KOROBOV rules                NMAX = 100 
%            3 RCRUDE Crude Monte-Carlo Algorithm with simple antithetic variates, NMAX = 100
%                     and weighted results on restart 
%            4 SPHMVN Monte-Carlo algorithm by Deak (1980), NMAX = 100
%
%

% The corresponding mex-file was successfully compiled for matlab 5.3
% using Compaq Visual Fortran 6.1, and Windows 2000 and XP.
% The example here uses Fortran77 source.
% First, you will need to modify your mexopts.bat file.
% To find it, issue the command prefdir(1) from the Matlab command line,
% the directory it answers with will contain your mexopts.bat file.
% Open it for editing. The first section will look like:
%
%rem ********************************************************************
%rem General parameters
%rem ********************************************************************
%set MATLAB=%MATLAB%
%set DF_ROOT=C:\Program Files\Microsoft Visual Studio
%set VCDir=%DF_ROOT%\VC98
%set MSDevDir=%DF_ROOT%\Common\msdev98
%set DFDir=%DF_ROOT%\DF98
%set PATH=%MSDevDir%\bin;%DFDir%\BIN;%VCDir%\BIN;%PATH%
%set INCLUDE=%DFDir%\INCLUDE;%DFDir%\IMSL\INCLUDE;%INCLUDE%
%set LIB=%DFDir%\LIB;%VCDir%\LIB
%
% then you are ready to compile this file at the matlab prompt using the following command:
%  mex -O mexGenzMvnPrb.f

value= [];
error1 = [];
inform = 1;
disp('mexGenzMvnPrb is not implemented as a m-function')
disp('                   compile the mexfile mexGenzMvnPrb.f before you try again.')
error('mexGenzMvnPrb error')
return