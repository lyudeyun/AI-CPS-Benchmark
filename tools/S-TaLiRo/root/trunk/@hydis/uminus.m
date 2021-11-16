% Unitary minus concatenation overloaded for hydis class

% (C) 2011 by Georgios Fainekos (fainekos@asu.edu)
% Last update: 2011.06.04

function out = uminus(inp)
out = inp;
out.ds = -out.ds;
out.dl = -out.dl;
