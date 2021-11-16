function [hh, locHis, news] = hasimulator1(HA, h0, UPoint, tot_time, input_range, nbControlPnts, solv,opt)

% See hasimulator.m for help

Rcmap = rcmap.instance();

% Set options
if nargin < 8
    opt = zeros(1,5);
    opt(5) = 1;
end

if nargin < 7
    solv = HA.ode_solver;
end
% global CALL_HASIM; assert(CALL_HASIM == 1, 'hasimulator1 called directly - should only be called through hasimulator');

options = odeset;
if opt(2)
    options = odeset(options,'MaxStep',opt(2));
end
if opt(3)
    options = odeset(options,'RelTol',opt(3));
end
if opt(4)
    options = odeset(options,'AbsTol',opt(4));
end


% check for the unsafe set
news = Rcmap.int('RC_SUCCESS');

% number of locations
nloc = length(HA.loc); 

% First pass - simulate the hybrid automaton
% (we assume that h0 does not activate any guards)
cloc = h0(1); % current location 
locHis = cloc; % location history
hh = h0; % hybrid system trajectory
while hh(end,2)<tot_time
    % Create the events
    actGuards = HA.adjList{cloc};
    % Set options
    if opt(1)
        options = odeset(options,'Events',@eventsStr);
    else
        options = odeset(options,'Events',@events);
    end    
    %**
    [values, isterminal, crossing_direction] = events(0,hh(end,3:end)');
    if sum(values<0)>0  
%         display(['One of the guards of location ',num2str(cloc), ' is violated before the simulation starts']); 
        iv=find(values<0);
        gv=actGuards(iv);
        
%         display(['The following starting pnt is on the fence between ',num2str(cloc), ' and ', num2str(gv),':']);
%         display(num2str(hh(end,:)));
        
        news = Rcmap.int('RC_SIMULATION_ZENO');
        if ~opt(5)
            error('Aborting')
        end
        return;
    end 
    %**            
    % simulate system
    % dum1 - last time, dum2 - last state (for debugging)
    [tt,xx,dum1,dum2,guard] = feval(solv,@locDyn,[hh(end,2),tot_time],hh(end,3:end),options);
    loc = ones(length(tt),1)*cloc; 
    % If guard is empty it should happen in the last iteration - add check
    if ~isempty(guard)
        ii = find(guard==noag+1, 1);
        if ~isempty(ii)
            % This is the case when we have reached the unsafe set
            hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
            news = Rcmap.int('RC_SIMULATION_UNSAFE_SET_REACHED');
            return;
        end
        ii = find(guard==noag+2, 1);
        if ~isempty(ii)
            % This is the case when we have reached the target set
            hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
            news = Rcmap.int('RC_SIMULATION_TARGET_SET_REACHED');
            return;
        end
        if length(guard)>1  
            warning(['The hybrid automaton is nondeterministic! While in location ',num2str(cloc),' the guards for locations ',num2str(actGuards(guard)),' are active!']); %#ok<WNTAG>
            guard = guard(1); %Choose a guard - in the future make the choice random
        end
        cloc = actGuards(guard);
        locHis = [locHis cloc]; %#ok<AGROW>
        loc(end) = cloc;
    end
    % ignore first entry - same as last
    hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
end
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nested fuction definitions

% I Location Dynamics
function yy = locDyn(tt,xx)    
    yy = HA.loc(cloc).A*xx+HA.loc(cloc).b;
end

    function yes = x_in_unsafe(x)
        yes = 0;
        nod = length(HA.unsafe.descriptions);
        %         [noc,nos,nod] = size(HA.unsafe.A);
        for kk = 1:nod % for each disjunct
            if HA.unsafe.descriptions(kk).set.pseudo_indicator(x) <= 0
%             if sum(HA.unsafe.A(:,:,kk)*xx<=HA.unsafe.b(:,kk)) == noc
                yes = 1;
                break;
            end
        end
    end

% II Event function with inequalities
% OUTPUTS:
%   val -> the value of the event 
%            1 if the guard is not activated
%           -1 if the guard is activated
%   isterm -> does the computation stop?
%   dir -> what is the direction for crossing the zero value?
function [val,isterm,Dir] = events(tt,xx) %#ok<INUSL>
    noads = 2;
    % find active guards
    noag = length(actGuards);
    isterm = ones(noag+noads,1);
    Dir = zeros(noag+noads,1);
    val = zeros(noag+noads,1);
    for jj = 1:noag
        nloc = actGuards(jj);
        % # of conjunctions, # of states, # of disjunctions
        [noc,nos,nod] = size(HA.guards(cloc,nloc).A);
        % find triggered guards
        val(jj) = 1;
        % Check each disjunct
        for kk = 1:nod
            % potential bug: a matrix with zeros
            if sum(HA.guards(cloc,nloc).A(:,:,kk)*xx<=HA.guards(cloc,nloc).b(:,kk)) == noc
                val(jj) = -1;
                break;
            end
        end
    end
    % add an event for the unsafe set
    if isfield(HA,'unsafe')
        % find triggered guards
        val(noag+1) = 1;
        if x_in_unsafe(xx)
            val(noag+1) = -1;
        end       
    end
    % add an event for the target sets
    if isfield(HA,'target')
        [noc,nos,nod] = size(HA.target.A);
        % find triggered guards
        val(noag+2) = 1;
        % Check each disjunct
        for kk = 1:nod
            % potential bug: a matrix with zeros
            if sum(HA.target.A(:,:,kk)*xx<=HA.target.b(:,kk)) == noc
                val(noag+2) = -1;
                break;
            end
        end
    end
end

% III Event function with strict inequalities
% OUTPUTS:
%   val -> the value of the event 
%            1 if the guard is not activated
%           -1 if the guard is activated
%   isterm -> does the computation stop?
%   dir -> what is the direction for crossing the zero value?
function [val,isterm,Dir] = eventsStr(tt,xx) %#ok<INUSL>
    noads = 2;
    % find active guards
    noag = length(actGuards);
    isterm = ones(noag+noads,1);
    Dir = zeros(noag+noads,1);
    val = zeros(noag+noads,1);
    for jj = 1:noag
        nloc = actGuards(jj);
        % # of conjunctions, # of states, # of disjunctions
        [noc,nos,nod] = size(HA.guards(cloc,nloc).A);
        % find triggered guards
        val(jj) = 1;
        % Check each disjunct
        for kk = 1:nod
            % potential bug: a matrix with zeros
            if sum(HA.guards(cloc,nloc).A(:,:,kk)*xx<HA.guards(cloc,nloc).b(:,kk)) == noc
                val(jj) = -1;
                break;
            end
        end
    end
    % add an event for the unsafe set
    if isfield(HA,'unsafe')
        % find triggered guards
        val(noag+1) = 1;
        if x_in_unsafe(xx)
            val(noag+1) = -1;
        end  
    end
    % add an event for the target sets
    if isfield(HA,'target')
        [noc,nos,nod] = size(HA.target.A);
        % find triggered guards
        val(noag+2) = 1;
        % Check each disjunct
        for kk = 1:nod
            % potential bug: a matrix with zeros
            if sum(HA.target.A(:,:,kk)*xx<HA.target.b(:,kk)) == noc
                val(noag+2) = -1;
                break;
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
