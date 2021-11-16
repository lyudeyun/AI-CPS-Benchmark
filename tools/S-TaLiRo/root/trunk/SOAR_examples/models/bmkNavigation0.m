function [HA h0] = bmkNavigation0(unsafe_model, invertibleAs)

% unsafe_model: circle or polyhedron
% invertibleAs: if 1, create an As matrix (in the paper's terminology) that
% is invertible. This allows explicit evaluation of the trajectory's state
% at a given time instant t as s(t;x)  = expm(t*A)*x0 + A\((expm(t*A)-I)*b)

if nargin < 1
    unsafe_model = 'circle';
end
if nargin < 2
    invertibleAs      = 0;
end
global plotit;
% Initial location
init.loc = 13;
% Initial (continuous) set = { x \in R^n | init(i,1) <= x(i) <= init(i,1)}
init.cube = [0.2 0.8
    3.2 3.8
    -0.4 0.4
    -0.4 0.4
    ];
n=4;
% System dynamics
% This is the C matrix from paper. See hsbenchamrk1 for more on it.
C = [4 2 3 4
    3 6 5 6
    1 2 3 6
    2 2 1 1];


if strcmp(unsafe_model, 'polyhedron')
    dj.set = polyhedron([eye(n); -eye(n)], [3.8; 0.8; 0.4; 0.4; -3.2; -0.2; 0.4; 0.4]);
    dj.loc = 4;
    unsafe = unsafeset([dj]);
elseif strcmp(unsafe_model, 'circle')
    xc = [3.5; 0.5; 0; 0];
    r = 0.3;
    us1.set = ball(xc,r); us1.loc = 4;
    unsafe = unsafeset([us1]);
else
    error(['Unrecognized unsafe set model ', unsafe_model]);
end

if invertibleAs
    % Create an invertible As matrix, by providing a 4x4 invertible Av matrix.
    % The choice below is just the matrix that hsbenchmark1 would have created,
    % but with its 0 eigenvalues replaced by -0.01. The eigenvalues must be
    % non-positive for stability.
    Av = [-0.01      0   1.008391608391609   0.000699300699301
        0  -0.01  -0.000699300699301   0.991608391608392
        0      0                -1.2   0.1
        0      0                 0.1  -1.2];
    HA = hsbenchmark1(plotit, init, C, unsafe, Av);
%     HA = navbench_hautomaton([],init,C,unsafe,Av)
    
else
    HA = hsbenchmark1(plotit, init, C, unsafe);
%     HA = navbench_hautomaton([],init,C,unsafe)
end
h0 = [init.loc 0  0.4 3.3 -0.1 -0.1 ];

