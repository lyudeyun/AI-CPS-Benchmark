function HA = hsbenchmark3(argv)

global ACTIVATE_ASSERT;

if nargin < 1
    error('You must supply hsbenchmark3 with appropriate arguments. See comments and bmkDemo.');
end

if isfield(argv,'name')
    HA.name = argv.name;
else
    HA.name = 'hsbenchmark3';
end

% list of locations : an array of integers
loclist = argv.loclist;

nLoc = length(loclist);

% initial set: init.loc is an integer indicating location, init.cube bounds each dimension of
% the state variable: cube(i,1) <= x(i) <= cube(i,2)
init= argv.init;
if ~isfield(init, 'loc') || ~isfield(init, 'cube')
    error('init must be a structure with fields loc and cube');
end

n = size(init.cube,1);
% Dynamics in each location
% loc is a structure array, where each element loc(i) is a structure with
% possible fields f (mandatory), A, b, ode_opts (optional).
% - f is a function handle that gives the RHS of the differential
%  equation defining the system in loc(i): dx/dt = f(x).
% - A, b: if given, f(x) = Ax+b
% - ode_opts: options created using odeset, and passed to ode solver when
% solving in location i.

for lix=loclist
    % dynamics(lix) is a function handle to f_i
    HA.loc(lix).f = argv.dynamics(lix).f;
    if isfield(argv.dynamics(lix),'A')
        HA.loc(lix).A = argv.dynamics(lix).A;
        if ACTIVATE_ASSERT
            assert(size(argv.dynamics(lix).A,1) == n, ['Matrix A of location ',num2str(lix), ' has wrong row dimension ',num2str(size(argv.dynamics(lix).A,1))]);
            assert(isempty(find(eig(argv.dynamics(lix).A)>0)), ['The A matrix of loc ', num2str(lix),' has positive eigenvalues, making it unstable.'])
        end
    end
    if isfield(argv.dynamics(lix),'b')
        HA.loc(lix).b = argv.dynamics(lix).b;
        assert(size(argv.dynamics(lix).b,1) == n, ['Vector b of location ',num2str(lix), ' has wrong size ',num2str(length(argv.dynamics(lix).b))]);
    end
    if isfield(argv, 'locations') && ~isempty(argv.locations(lix).ode_opts)
        HA.loc(lix).ode_opts = argv.locations(lix).ode_opts;
    else
        HA.loc(lix).ode_opts = struct();
    end
end

%sanity check
if length(HA.loc) ~= nLoc
    error('Must provide dynamics for each location');
end

% adjList{i} is list of locations reachable in one jump from location i
% e.g. adjList{1} = [3 6] means we can jump from loc 1 to location 3 via
% some guards(1,3) and to location 6 via guards(1,6)
adjList = argv.adjList;

% invariants is a list of pseudo-indicator function handles.
% fi = invariants{i} is a function handle s.t.
%   invariant set of loc i = {x|fi(x)<=0}
% E.g. invariants{i} = @(x) A*x+b
% loclist(i) = {x | invariants(loclist(i))(x) <= 0}
invariants = argv.invariants;
%sanity check
if ~isempty(invariants) && length(invariants) ~= nLoc
    error('Must provide invariants for each location');
end

% guards is a 2D array of structs.
% - guards(i,j).f is a function handle that gives the transition
% condition from loc i to loc j: for a jump to be possible, must have
% guards(i,j).f(x) <= 0 (or strict inequality - this is determined by
% guards(i,j).strict.
% - guards(i,j).strict: if 1, strict inequality is assumed for guard condition,
% otherwise non-strict inequality. Default = 0.
% - guards(i,j).A and guards(i,j).b are optional fields. If given, it means
% that guards(i,j).f = @(x) A*x+b. They are needed when computing Lyapunov
% functions for robustness ellipsoid.
% Note that guards(i,j) is non-empty iff resets(i,j) non-empty
guards = argv.guards;
for i=1:size(guards,1)
    for j=1:size(guards,2)
        if guard_is_nontrivial(guards(i,j))
            if ~isfield(guards(i,j), 'strict')
                guards(i,j).strict=0;
            end
            strg = ['guards(',num2str(i),',', num2str(j),')'];
            if ~isempty(guards(i,j).A) & ACTIVATE_ASSERT
                assert(~isempty(guards(i,j).b), ['Guard(',strg,'.b is empty even though A is not']);
                assert(size(guards(i,j).A,1)==size(guards(i,j).b,1), ['Incompatible dimensions for A and b of ',strg]);
                assert(n==size(guards(i,j).A,2),[strg, '.A has wrong nb of cols']);
            end
        end
    end
end

% resets is a 2D array of structs. Each entry
% resets(i,j) is a struct with field 'f', which gives the reset function
% for jumping from location i to location j.
% Note that guards(i,j) non-empty iff resets(i,j) non-empty
resets = argv.resets;
if ~isempty(argv.resets) && (size(resets,1) ~= size(guards,1) || size(resets,2) ~=size(guards,2))
    error('resets and guards must have same dimensions: to reset between location i and j, must have a guard to go through');
end

% unsafe is an object of class unsafeset
u = argv.unsafe;
if isa(u, 'unsafeset')
    unsafe = u;
elseif isfield(u,'predicates') % in s-taliro
    unsafe = u;
else
    error('hsbenchmark3: Invalid unsafe set specification');
end

% Any options and attributes specific to this benchmark (but these can't be
% used in hasimulator since they're not standard)
if isfield(argv, 'custom')
    HA.custom = argv.custom;
end

% % target is an object of class unsafeset
% if ~isfield(argv, 'target')
%     % default is a ball centered at xc or radius r
%     xct = [-1.5; 0.5; 0; 0];
%     rt = 0.2;
%     ust.set = ball(xct,rt); ust.loc = 2;
%     target = unsafeset([ust]);
% else
%     target = argv.target;
% end
%
% % Remove transitions to the target set
% for lix=loclist
%     jj = find(adjList{lix} == ust.loc);
%     adjList{lix}(jj) = [];
% end

HA.type = 'hautomaton';
HA.init = init;
HA.adjList = adjList;
HA.guards = guards;
HA.invariants = invariants;
HA.unsafe = unsafe;
HA.resets = resets;
% HA.target = target;
if isfield(argv,'simulator')
    HA.simulator = argv.simulator;
elseif  nLoc == 1 && isempty(guards) && isempty(resets)
    HA.simulator = @hasimulatorlinear;
else
    HA.simulator = @hasimulator3;
end

if isfield(argv, 'ode_solver')
    HA.ode_solver = argv.ode_solver;
else
    HA.ode_solver = 'ode45';
end

%===================================================================
    function yes=guard_is_nontrivial(g)
        yes = ~isempty(g.f);
    end

end
