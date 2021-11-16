function  [hh, locHis, news] = hasimulator4(HA, h0, UPoint, tot_time, staliro_InputBounds, nb_ControlPoints, solv, opt);

global RUNSTATS;
Rcmap = rcmap.instance();

% Set options
if nargin < 8
    opt = zeros(1,5);
    opt(5) = 1;
end

if nargin < 7
    solv = HA.ode_solver;
end

% Set options
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


news = Rcmap.int('RC_SUCCESS');
et =  1e-7; % amount of artificial shift in resest points' time to avoid interp1 errors

% number of locations
nloc = length(HA.loc);

% First pass - simulate the hybrid automaton
% (we assume that h0 does not activate any guards)
cloc = h0(1); % current location
locHis = cloc; % location history
hh = h0; % hybrid system trajectory
prev_jump_time = h0(2); 
prev_cloc = h0(1); 
prev_nloc = -1;
while hh(end,2)<tot_time
    % Create the events
    % disp(['   1. cloc = ', num2str(cloc), ', news = ', num2str(news)]);
    actGuards = HA.adjList{cloc};
    nbActGuards = length(actGuards);
    xr=[]; % reset point (if reset happens)
    % Set options
    options = odeset(options, 'Events', @eventsVar);
    
    %% Discrete dynamics for h0
    [values, isterminal, crossing_direction] = eventsVar(0,hh(end,3:end)');    
    if sum(values<0)>0      % some event happened at the starting point      
        if values(end-1) <0 
            disp('Starting point entered unsafe set')             
            news = Rcmap.int('RC_SIMULATION_UNSAFE_SET_REACHED');
            return;
        elseif values(end) < 0
            disp('Starting point entered target set')
            news = Rcmap.int('RC_SIMULATION_TARGET_SET_REACHED');
            return;
        end
        gv=actGuards(find(values<0));        
        %         display(['The following starting pnt is on the fence between ',num2str(cloc), ' and ', num2str(gv),':']);
        %         display(num2str(hh(end,:)));
        if detect_zeno(cloc,gv,hh(end,3:end)')
            news=Rcmap.int('RC_SIMULATION_ZENO');
            disp(['System is Zeno: [',num2str(hh(end,3:end)), '] ping-pongs out of loc ', num2str(cloc)]);
            if opt(5)==0 error('   => Aborting')
            else         disp('    => Return'); return
            end
        end
        if length(gv)>1
            gv = gv(1); % choose arbitrarily
            warning(['The automaton is non-deterministic because more than one transition is possible - selecting ',num2str(gv)]);
            news = Rcmap.int('RC_SIMULATION_NONDETERMINISTIC');
        end
        nloc = gv;
        xr = reset_this(hh(end,3:end)',cloc,nloc);
        disp(['Immediately crossing guard between ',num2str(cloc),' and ', num2str(nloc), ' (@ time = ',num2str(hh(end,2)), ')' ]);
        if (prev_jump_time == hh(end,2) && prev_cloc ==cloc && prev_nloc == nloc)
            disp('No time elapsed since last immediate jump - should be zeno?')
            1;          % place holder for breakpoint
        end
        %update for next iteration
        prev_jump_time = hh(end,2); prev_cloc=cloc; prev_nloc = nloc;
        cloc = nloc;
        locHis = [locHis cloc]; %#ok<AGROW>
        % In principle, the reset point should have same time as
        % pre-reset. But repeated values in the t vector causes an
        % error when interpolating (in get_hx_at_t), so shift it to the right by a
        % smidge
        hh = vertcat(hh,horzcat(cloc, hh(end,2) + et, xr'));
        continue;
    end
    %% Simulate system
    % Simulates until next guard crossing, or unsafe/target reaching
    loc_ode_opts = options;
    loc_ode_opts = odeset(loc_ode_opts, HA.loc(hh(end,1)).ode_opts);
%     if RUNSTATS.power
%         loc_ode_opts = odeset(loc_ode_opts, 'Stats', 'on'); 
%     end
    [tt,xx,TE,YE,guard] = feval(solv,@locDyn,[hh(end,2),tot_time],hh(end,3:end),loc_ode_opts);    
    guard = funky_weird_behavior(guard);
    % disp(['    2. cloc = ', num2str(cloc)])
    loc = ones(length(tt),1)*cloc;
    if ~isempty(guard)
        ii = find(guard==noag+1, 1);
        if ~isempty(ii)
            % This is the case when we have reached the unsafe set
            disp('Simulator reached unsafe set');
            hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
            news = Rcmap.int('RC_SIMULATION_UNSAFE_SET_REACHED');
            return;
        end
        ii = find(guard==noag+2, 1);
        if ~isempty(ii)
            % This is the case when we have reached the target set
            disp('Simulator reached target set')
            hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
            news = Rcmap.int('RC_SIMULATION_TARGET_SET_REACHED');
            return;
        end
        if length(guard)>1
            warning(['The automaton is non-deterministic! While in location ',num2str(cloc),' the guards for locations ',num2str(actGuards(guard)),' are active!']); %#ok<WNTAG>
            news = Rcmap.int('RC_SIMULATION_NONDETERMINISTIC');
            guard = guard(1); %Choose a guard - in the future make the choice random
        end
        nloc = actGuards(guard);
        xr = reset_this(xx(end,:)',cloc, nloc);
        %disp(['Crossing guard between ',num2str(cloc),' and ', num2str(nloc)])
%         plot(xr(1),xr(2),'ro'); hold on;
        % Update for next iteration
        cloc = nloc;
        locHis = [locHis cloc]; %#ok<AGROW>
        loc(end) = cloc;
    else
        % disp('Guard is empty.');
        if tt(end) < tot_time
            error('Simulation stopped before the end of time but we have not crossed any guard. How come?'),
        end
    end
    
    % Ignore first entry - same as last
    hh = vertcat(hh,horzcat(loc(2:end),tt(2:end),xx(2:end,:))); %#ok<AGROW>
    if ~isempty(xr)
        hh = vertcat(hh,horzcat(loc(end), tt(end)+et, xr'));
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nested fuction definitions

% If inputs pnt xx satisfies both guards cloc->nloc and nloc->cloc,
% then xx is Zeno
    function yes = detect_zeno(cloc,nextlocs, xx)
        yes = 0;
        for nloc=nextlocs
            gCN = HA.guards(cloc,nloc).f(xx);
            CN = find(gCN <= 0);
            if length(CN) == length(gCN)
                xreset  = reset_this(xx,cloc,nloc);
                gNC     = guard_this(xreset,nloc,cloc);
                NC      = find(gNC <= 0);
                if length(NC) == length(gNC)
                    yes = 1;
                    return;
                end
            end
        end
    end


%% Location Dynamics
    function yy = locDyn(tt,xx)
        yy = HA.loc(cloc).f(tt,xx);
    end
%% x_in_unsafe
    function yes = x_in_unsafe(x)
        yes = 0;        
        phonytime = 0;
        h = [cloc phonytime x'];
        yes = HA.unsafe.hybrid_inclusion(h);
    end
%% x_in_target
    function yes = x_in_target(x)
        yes = 0;
        if HA.target.strict
            target_cond = HA.target.pseudo_indicator(x) < 0;
        else
            target_cond = HA.target.pseudo_indicator(x) <= 0;
        end        
        if sum(target_cond) == length(target_cond)
            yes=1;
        end
        
    end

%% Event function 
% Events are crossing guards, entering unsafe set, or entering target set
% OUTPUTS:
%   val -> the value of the event
%            1 if the event is not activated
%           -1 if the event is activated
%   isterm -> does the computation stop?
%   dir -> what is the direction for crossing the zero value?
    function [val,isterm,Dir] = eventsVar(tt,xx) %#ok<INUSL>
        % find active guards
        noag = length(actGuards);
        % 2 other events possible (other than guards): reaching target or
        % unsafe set.
        noads = 2;        
        isterm = ones(noag+noads,1); % terminate integration at 0 of event
        % XXX two modification below should be reviewed with GF
        % Modification 1: A guard is active when its function evaluates to <[=]0 - therefore
        % the zero-crossing counts only if we're going to the negative side
        % (therefore decreasing), and not otherwise.        
%         Dir = zeros(noag+noads,1);
        Dir = [-ones(noag,1); zeros(noads,1)];
        % Modification 2: This should be ones and not zeros, because a 0 might be detected
        % by the 0-crossing algo. At any rate, ones can't be wrong.
%         val = zeros(noag+noads,1);
        val = ones(noag+noads,1);
        for jj = 1:noag
            nloc = actGuards(jj);
            % find triggered guards
            val(jj) = 1;
            if HA.guards(cloc,nloc).strict==1
                guard_cond = (HA.guards(cloc,nloc).f(xx) < 0) ;
            else
                guard_cond = (HA.guards(cloc,nloc).f(xx) <= 0) ;
            end
            if sum(guard_cond) == length(guard_cond)
                val(jj) = -1;
                break;
            end
        end
        % add an event for the unsafe set
        if isfield(HA,'unsafe')
            val(noag+1) = 1;
            if x_in_unsafe(xx)
                val(noag+1) = -1;
            end
        end
        % add an event for the target sets
        if isfield(HA,'target')
            val(noag+2) = 1;
            if x_in_target(xx)
                val(noag+2) = -1;
            end            
        end
    end



%% reset_this
    function xpost = reset_this(xpre,cloc,nloc)
        if ~isempty(HA.resets) && ~isempty(HA.resets(cloc,nloc))
            xpost = HA.resets(cloc, nloc).f(xpre);
        else
            xpost = xpre;
        end
    end % reset_this()
%% guard_this
    function g12 = guard_this(x, loc1, loc2)
        if ~isempty(HA.guards) && loc1 <= size(HA.guards,1) && loc2 <= size(HA.guards,2) && ~isempty(HA.guards(loc1,loc2)) && ~isempty(HA.guards(loc1,loc2).f)
            g12 = HA.guards(loc1,loc2).f(x);
        else
            g12 = [inf]; % guard condition definitely not satisfied, provoke mayhem
        end
    end % guard_this
%% funky_weird_behavior
    function g=funky_weird_behavior(guard)
        g=guard;
        if length(guard) > 1 && length(unique(guard)) == 1
            ag = actGuards(guard); 
            if size(ag,1)>1 ag = ag'; end            
            warning(['Overcoming weird behavior where 2 active guards ',num2str(ag), ' of loc ', num2str(cloc), ' are returned by feval, but they are both the same, at different times']);
            g=guard(1);
        end
    end % funky_weird_behavior
            
            
          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
