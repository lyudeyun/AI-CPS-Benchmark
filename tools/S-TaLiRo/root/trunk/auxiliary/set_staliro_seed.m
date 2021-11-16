% set_staliro_seed : Set the seed for the random number generator.
% 
% Usage:
%     out = set_staliro_seed(inp_seed)
%
% Input:
%   inp_seed : 
%       * if it is a structure, then pass it to the random number generator. 
%         See rng.
%       * if it is -1, then return the state of the random number generator.
%       * if it is a positive number, then use it to initialize the random
%         number generator and return its state.
%
% Output:
%   out : the state of the random number generator to be used for
%         reproducibility
%       
% See also: staliro, rng

% (C) G. Fainekos - S-TaLiRo 

function out = set_staliro_seed(inp_seed)

if isstruct(inp_seed)
    
    rng(inp_seed);
    out = inp_seed;
    
elseif inp_seed ~= -1
    
    if verLessThan('matlab', '7.12')
        stream = RandStream('mt19937ar','Seed',inp_seed);
        RandStream.setDefaultStream(stream);
        out = inp_seed;
    else
        rng(inp_seed);
        out = rng;
    end
    
else
    
    if verLessThan('matlab', '7.12')
        out = inp_seed;
    else
        out = rng;
    end
        
end


end

