function [TS, XT, YT, LT, CLG, GRD]  = stoch_chain_simulator(x0,T,A1,G1,N)
% NAME
% 
%     stoch_chain_simulator - simulate the chain automaton
% 
% SYNOPSIS
% 
%     [TS, XT, YT, LT, CLG, GRD]  = stoch_chain_simulator(x0,T,A1,G1,N)
% 
% DESCRIPTION
% 
%     Simulate the chain linear stochastic hybrid automaton. It is a chain of 21 modes, with linear dynamics in each
%     with Brownian motion disturbance. System can only move between successive modes following a fixed-rate Poisson 
%     process. For system description, see 
%     @INCOLLECTION{Julius_StochasticBisimulation06,
%     author = {Julius, A.Agung},
%     title = {Approximate Abstraction of Stochastic Hybrid Automata},
%     booktitle = {Hybrid Systems: Computation and Control},
%     publisher = {Springer },
%     year = {2006},
%     volume = {3927},
%     series = {LNCS},
%     pages = {318-332},
%     }
%         
%   Inputs
% 
%     x0       
%         Initial condition
%   
%     T
%         simulation time
%     
%     A1
%        A matrix of the dynamics
% 
%     G1
%        A matrix of the dynamics
% 
%     N
%        nb of locations
%     
%   Outputs
%     
%     TS
%         time instants. Nb of instants if hard-coded, see variable NT.
%         
%     XT
%         2D continuous state, one row per time instant
%         
%     YT
%         1D continouos output = XT(2), one row per time instant
%         
%     LT
%         location (or 'mode') output
%    
%    CLG, GRD
%         both empty, here for interface compatibility
%     
% 
% EXAMPLES
% 
%         x0 = [1 1];
%         T = 100;
%         load('stochChain.mat'); % this will populate A1, G1 and N
%         [TS, XT, YT, LT, ~, ~]  = stoch_chain_simulator(x0,T,A1,G1,N)
%         subplot(2,1,1); plot(T,[XT,YT]); legend('X','Y')
%         subplot(2,1,1); plot(T,LT); title('locations');
%             
%        
% AUTHOR(S)
% 
%        Written by Agung Julius
%        (C) A. Agung Julius, 2006
% 
% See also - staliro_demo_stoch_cold_chain
            


NT=1000;
dt=T/NT;

% Initial conditions for simulation
L10=1;

XT=zeros(2,NT+1);
YT=zeros(1,NT+1);
LT=zeros(1,NT+1);

XT(:,1)=x0';
YT(1)=[0 1]*x0;
LT(1)=L10;

for i=2:NT+1
    bi1=rand;
    de1=rand;
    bi2=rand;
    de2=rand;
    birth1=0;
    death1=0;
    birth2=0;
    death2=0;
    
    if bi1>=exp(-100*dt) birth1=1;
    end;
    
    if de1>=exp(-100*dt) death1=1;
    end;
        
    LT(i)=min(max(LT(i-1)+birth1-death1,1),N);
    
    dwt=sqrt(dt)*randn;
    
    XT(:,i)=XT(:,i-1) + A1(2*LT(i)-1:2*LT(i),2*LT(i)-1:2*LT(i))*XT(:,i-1)*dt + G1*XT(:,i-1)*dwt;
    YT(i)=XT(2,i);
        
end;

XT = XT';
YT = YT';
LT = LT';
TS = dt*(0:1:NT)';
CLG = [];
GRD = [];
