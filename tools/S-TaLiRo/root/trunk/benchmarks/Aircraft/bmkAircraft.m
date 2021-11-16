function [HA h0] =  bmkAircraft(nb_planes, nb_waypnts)
% Power train model from "Probabilistic testing for stochastic hybrid
% systems" by A. Agung Julius et al., 2008
% available at http://repository.upenn.edu/cgi/viewcontent.cgi?article=1035&context=grasp_papers 
%
% The overall system of planes can be thought of as the composition of
% nb_planes systems, each with its own dynamics, locations, guards, etc.

global plotit;
plane_colors =['b','m','r','g','k','c','y'];
plane_symbols=['o','x','d','s','v','*','+'];
% Each aircraft starts in location 1, which means "heading to waypoint 1".
% Location k of the aircraft means "heading to waypoint k". After last
% waypoint is reached (by traveling in location nb_waypnts), plane enters
% last location, from which there is no exit (since all waypnts have been
% visited). Ergo nbLocations  = nb_waypnts +1. And the system as a whole
% has (nb_waypnts+1)^nb_planes locations
nplaneLoc   = nb_waypnts+1;
nLoc        = nplaneLoc^nb_planes;
% State of a plane  = [x y vx vy]
n           = 4*nb_planes;

%% Describe the flight space
flight_box      = [0 10*nb_planes/2;0 10*nb_planes/2];
nb_horiz_boxes  = 10; 
nb_vert_boxes   = 10; 
assert(nb_horiz_boxes*nb_vert_boxes > nb_planes);
nb_boxes        = nb_horiz_boxes*nb_vert_boxes;
xincr = (flight_box(1,2)-flight_box(1,1)) / nb_horiz_boxes;
yincr = (flight_box(2,2)-flight_box(2,1)) / nb_vert_boxes;
% boxes(k,:) bounds the kth individual box (within flight_box): first 2
% entries bound the x dimension, second 2 entries bound the y dimension
boxes = zeros(nb_boxes, 4);
nvb = (1:nb_vert_boxes)';
box_bottom_limit    = flight_box(2,1)+(nvb -1)*yincr;
box_top_limit       = flight_box(2,1)+(nvb)*yincr;
k=1;
for i=1:nb_horiz_boxes
    box_left_limit      = repmat(flight_box(1,1)+(i-1)*xincr, nb_vert_boxes,1);
    box_right_limit     = box_left_limit+xincr;    
    boxes(k:k+nb_vert_boxes-1,:) = [box_left_limit box_right_limit box_bottom_limit box_top_limit];
    k=k+nb_vert_boxes;
end
assert(k-1 == nb_boxes);

%% Init
if plotit
    figure;
    plot(polytope(ProdTop2Polytope(flight_box)),'w');
    hold on;
end
% Assign boxes to planes for initial set. 2 planes might share same box.
assigned    = assign_boxes_to_planes(boxes, nb_planes, 'random');
init.loc    = 1;   % everyone going to their first waypoint
vmin        = 1;   assert(vmin > 0); % planes can't be flying at 0 speed
vmax        = 5;  assert(vmax >= vmin);
init.cube   = zeros(n,2);
for k=1:nb_planes
    init.cube(4*k-3:4*k,:) = [
        boxes(assigned(k),1:2);
        boxes(assigned(k),3:4);
        vmin vmax;
        vmin vmax];
    if plotit plot(polytope(ProdTop2Polytope(init.cube(4*k-3:4*k-2,:))),plane_colors(k)); end
end
assert(size(init.cube,1)==n);
if plotit    
    yyfixed=flight_box(2,1):yincr:flight_box(2,2);
    xxfixed=flight_box(1,1):xincr:flight_box(1,2);
    for k =1:nb_boxes        
        xx=boxes(k,1)*ones(1,length(yyfixed));
        plot(xx,yyfixed,'k--');        
        yy=boxes(k,3)*ones(1,length(xxfixed));
        plot(xxfixed,yy,'k--');
    end
end
    

%% Way points 
waypnts = zeros(2*nb_waypnts, nb_planes);
% The simplest way to produce the waypoints is to generate them
% randomly in the flight_box. But we don't want different instances
% (with same dimension) to have different waypoints - otherwise results
% are not comparable. So create a separate RandStream and use it to
% generate the waypoints. This stream will be reset with every instance of
% the benchmark.
wpStream = RandStream('mlfg6331_64');
ixeven=2:2:2*nb_waypnts; nbeven=length(ixeven);
ixodd =1:2:2*nb_waypnts; nbodd = length(ixodd);
waypnts_order = 'left to right';
for plane =1:nb_planes  
    pw=zeros(2*nb_waypnts,1);       
    pw(ixodd)     = flight_box(1,1)*ones(nbodd, 1)  + (flight_box(1,2)-flight_box(1,1))*rand(wpStream, nbodd, 1);
    pw(ixeven)    = flight_box(2,1)*ones(nbeven, 1) + (flight_box(2,2)-flight_box(2,1))*rand(wpStream, nbeven, 1);
    % Order the waypoints in a reasonable way so plane can fly in between
    % Possible order 1: first fly to the left-most point, then fly from 
    % left to right
    if strcmp(waypnts_order, 'left to right')
        sorted_pw = zeros(2*nb_waypnts,1);
        [s,ix]=sort(pw(ixodd));
        sorted_pw(ixodd)=s;
        pwY=pw(ixeven);
        sorted_pw(ixeven)=pwY(ix);
        waypnts(:,plane)=sorted_pw;
    elseif strcmp(waypnts_order, 'right to left')
        sorted_pw = zeros(2*nb_waypnts,1);
        [s,ix]=sort(pw(ixodd),1,'descend');
        sorted_pw(ixodd)=s;
        pwY=pw(ixeven);
        sorted_pw(ixeven)=pwY(ix);
        waypnts(:,plane)=sorted_pw;
    else
        error('Unknown waypnts_order');
    end
end

if plotit     
    for plane=1:nb_planes
        for w=1:nb_waypnts
            plot(waypnts(2*w-1,plane),waypnts(2*w,plane),[plane_colors(plane),plane_symbols(plane)]);
        end
    end    
end

adjList = cell(1,nLoc);
A = [0 0 1 0; 
    0 0 0 1; 
    -10 0 -10 0;
    0 -10 0 -10]; % note the second -10: this is a correction from a typo in the paper
assert(A(4,4)==-10,'A(4,4) must be -10: this is a correction from a typo in the paper');
% The (system)A matrix is independent of location, so build once
sysA = zeros(n);
sysb = zeros(n,1);
for plane=1:nb_planes
    sysA(4*plane-3:4*plane, 4*plane-3:4*plane) = A;
end
loc_ode_opts = struct('Jacobian', sysA, 'JConstant', 1);

%% Guard direction
% Depending on relative positions of waypnt and plane, actual guard
% is either  {x| a1*x + a2*y +b <= 0} or  {x| a1*x + a2*y +b >= 0}
% The goal is to make sure the initial pnt doesn't find itself on
% the active side of all guards, causing a complete discrete trajectory.
msign=ones(nb_planes, nb_waypnts);
for plane=1:nb_planes
    if strcmp(waypnts_order, 'left to right')
        msign(plane,2:end) = -ones(1,nb_waypnts-1);
        if init.cube(4*plane-3,2)<=waypnts(1,plane)
              msign(plane,1) = -1;
        else
            msign(plane,1) =1; % potentially, the first jump (into location 2) is immediate.
        end
    elseif strcmp(waypnts_order, 'right to left')
        msign(plane,2:end) = ones(1,nb_waypnts-1);
        if init.cube(4*plane-3,1)>=waypnts(end-1,plane)
              msign(plane,1) = 1;
        else
            msign(plane,1) = -1; % potentially, the first jump (into location 2) is immediate.
        end
    else
        warning('INPUT', ['The waypnts order you specified, ', waypnts_order,' is unknown to me - guards will all be left half-spaces (msign = 1)']); %#ok<CTPCT>
    end
       
end

%% Location-specific stuff
% loc is a vectorized location: it gives each plane's location, the
% global location index is computed from it.
loc = ones(1,nb_planes); % first location: all planes heading to their 1st waypoint
allcfgs = repmat([1, nplaneLoc],nb_planes,1);% while loop will iterate over this
invariants      = cell(1,nLoc);
[invariants{:}] = deal(@(x) 1);  % they're not used for anything, just use always true.
dynamics        = struct('A',cell(1,nLoc),'b',cell(1,nLoc),'f',cell(1,nLoc));
locations       = struct('ode_opts',cell(1,nLoc));
guards          = struct('A',cell(nLoc, nLoc), 'b',cell(nLoc, nLoc), 'f',cell(nLoc, nLoc), 'strict',0);
planeloc2sysloc = zeros(nLoc,nb_planes);
incr=1;
loop_alarm=0;
while(incr)  % for each system location..
    loop_alarm = loop_alarm+1;
    if loop_alarm > nLoc
        error('[bmkAircraft] It seems the while(incr) is looping too long - infinite?')
    end
    ix_loc  = shifted_basen2dec(loc,nplaneLoc,-1, 1);
    planeloc2sysloc(ix_loc,:) = loc;
        
    %% Dynamics
    for plane=1:nb_planes
        if loc(plane) < nplaneLoc   % haven't reached last location of this plane
            ix_wp = loc(plane);
            px = waypnts(2*ix_wp-1, plane); py = waypnts(2*ix_wp, plane);            
        end
        if loc(plane) > 1
            ix_lwp = loc(plane)-1;
            lpx = waypnts(2*ix_lwp-1, plane); lpy = waypnts(2*ix_lwp, plane);
            vx = px-lpx; vy = py-lpy;
            temp = norm([vx vy]);
            vx = vx/temp; vy = vy/temp;
        else
            vx = 0; vy = 0;
        end        
        % else, the dynamics don't change when going to the last location,
        % which indicates all this plane's waypnts have been met.
%         sysb(4*plane-3:4*plane) = [0;0;10*px;10*py];
        sysb(4*plane-3:4*plane) = [vx;vy;10*px;10*py];
    end
    dynamics(ix_loc).A = sysA;
    dynamics(ix_loc).b = sysb;
    dynamics(ix_loc).f = @(t,x) dynamics(ix_loc).A*x + dynamics(ix_loc).b;
    
    % Other opts
    locations(ix_loc).ode_opts = loc_ode_opts;
        
    %% Guards induced by various waypoints
    % Any other plane's waypoints induce a jump of the global system's
    % state, when that other plane goes through its waypoint. So starting 
    % at the current location, we create a jump per other plane.      
    for plane=1:nb_planes        
        if loc(plane)==nplaneLoc
            continue; % no more jumps for this plane
        end
        % Define guard that sends plane to next location = guard induced by waypoint we're heading towards
        ix_nwp = loc(plane);
        px = waypnts(2*ix_nwp-1, plane); py = waypnts(2*ix_nwp, plane);
        nloc = loc; nloc(plane)=loc(plane)+1;
        ix_nloc = shifted_basen2dec(nloc,nplaneLoc,-1, 1);
        % Preliminary guard = {x | a1*x + a2*y +b <= 0};
        slope = inf; % y = slope*x + b
        if ~isinf(slope)
            g.A = [slope -1 0 0]; % slope*x - y + b = 0
            g.b = py-slope*px;
        else % vertical line
            g.A = [1 0 0 0];      % x = px
            g.b = -px;
        end
        g.A =  msign(plane,ix_nwp)*g.A;
        g.b =  msign(plane,ix_nwp)*g.b;
        g.f = @(x) g.A*x + g.b;
        g.strict = 0;
        if plotit
            b = py-slope*px;            
            if ~isinf(slope) 
                xx=flight_box(1,1):flight_box(1,2); yy=slope*xx + b;
            else
                yy=flight_box(2,1):flight_box(2,2); xx = px*ones(size(yy));
            end
            p=find(yy>=flight_box(2,1) & yy<=flight_box(2,2));
            yy=yy(p); xx=xx(p);
            plot(xx,yy,plane_colors(plane));          
        end
        
        % Define global guard caused by this plane's transition
        a=4*plane-3;
        guards(ix_loc, ix_nloc).A       = [zeros(1,a-1) g.A zeros(1,n-(a-1+4))];
        guards(ix_loc, ix_nloc).b       = [g.b];
        guards(ix_loc, ix_nloc).f       = @(x) guards(ix_loc, ix_nloc).A*x+guards(ix_loc, ix_nloc).b;
        guards(ix_loc, ix_nloc).strict  = g.strict;
        
        % Adjacency list
        adjList{ix_loc} = [adjList{ix_loc} ix_nloc];        
    end
    
    [loc, incr] = get_next_point(loc,1,allcfgs);
end

for l=1:nLoc-1
    assert(~isempty(locations(l)) && isfield(locations(l),'ode_opts'));
    assert(~isempty(adjList{l}));
end
assert(~isempty(locations(nLoc)) && isfield(locations(nLoc),'ode_opts'));
assert(size(planeloc2sysloc,1)==nLoc);

%% unsafe
% Any 2 planes getting within dunsafe of each other
% Instead of L2 distance, we use L1 distance because it allows us to derive
% linear equations as follows (and is equivalent to L2-norm):
% |x1-x2|+|y1-y2| <= dunsafe <==>      x1-x2 + y1-y2 <= dunsafe
%                                 AND  x1-x2 + -y1+y2 <= dunsafe
%                                 AND  -x1+x2 + y1-y2 <= dunsafe
%                                 AND  -x1+x2 + -y1+y2 <= dunsafe
% So each plane gets 4 equations with every other plane. Let's call these 4 equations
% a bundle; so each plane has a bundle with every other plane, with a total
% of nb_planes(nb_planes - 1) bundles.
% For the point to be in unsafe, it must simultaneously satisfy all
% equations in at least one bundle. So each bundle can be put in a separate
% disjunct of unsafe.
% The locations don't matter: regardless of current dynamics, proximity of
% 2 planes is dangerous. Therefore each disjunct has all locations.
dunsafe = 0.01;
bK = dunsafe*ones(4,1);
assert(dunsafe <= 0.4*(flight_box(1,2)-flight_box(1,1)));
assert(dunsafe <= 0.4*(flight_box(2,2)-flight_box(2,1)));
disjuncts = struct('set', cell(1,nb_planes^2 - nb_planes), 'loc', cell(1,nb_planes^2 - nb_planes));
ixd=0;
for plane1=1:nb_planes
    for plane2=1:nb_planes
        if plane1==plane2
            continue
        end
        ixd=ixd+1;
        % Equation1
        z1=zeros(1,n);
        z1(1,4*plane1-3:4*plane1)=[1 1 0 0];
        z1(1,4*plane2-3:4*plane2)=[-1 -1 0 0];
        % Equation2
        z2=zeros(1,n);
        z2(1,4*plane1-3:4*plane1)=[1 -1 0 0];
        z2(1,4*plane2-3:4*plane2)=[-1 1 0 0];
        % Equation3
        z3=zeros(1,n);
        z3(1,4*plane1-3:4*plane1)=[-1 1 0 0];
        z3(1,4*plane2-3:4*plane2)=[1 -1 0 0];
        % Equation4
        z4=zeros(1,n);
        z4(1,4*plane1-3:4*plane1)=[-1 -1 0 0];
        z4(1,4*plane2-3:4*plane2)=[1 1 0 0];
        % Bundle (plane1,plane2)
        bH = [z1;z2;z3;z4];
        dj.set = polyho(bH, bK, struct('AND_or_OR','AND', 'strict',0));
        dj.loc = 1:nLoc;
        disjuncts(ixd) = dj;
    end
end
assert(length(disjuncts)==nb_planes*(nb_planes-1));
unsafe = unsafeset(disjuncts);

if plotit
    title(['bmkAircraft with ', num2str(nb_planes), ' planes, ',num2str(nb_waypnts), ' waypoints']);
end

resets = [];

custom = struct('nb_planes', nb_planes, 'waypnts', waypnts, 'planeloc2sysloc',planeloc2sysloc, 'flight_box', flight_box);

argv.init       = init;

argv.locations  = locations;
argv.invariants = invariants;
argv.guards     = guards;
argv.dynamics   = dynamics;
argv.adjList    = adjList;
argv.loclist    = 1:nLoc;
argv.resets     = resets;
argv.unsafe     = unsafe;
argv.simulator  = @hasimulator4;
argv.ode_solver = 'ode45';
argv.name       = 'bmkAircraft';
argv.custom     = custom;

HA = hsbenchmark3(argv);

h0=[init.loc 0 (init.cube(:,1)'+init.cube(:,2)')/2];

%
1;
    function d = basen2dec(v,base)
        % Convert base-b number to decimal
        V=length(v);
        p=V-1:-1:0;
        bp=base.^p;
        d=v*bp';
    end

    function d = shifted_basen2dec(v,b,shi, shr)
        % Take base-b input vector v, shift it by shi (shift input), convert to
        % decimal, and shift result by shr.
        d = basen2dec(v+shi*ones(size(v)), b) + shr;
    end

    function assigned = assign_boxes_to_planes(boxes, nb_planes, asmt)
        nb = size(boxes,1);
        assigned = zeros(1,nb_planes);
        if strcmp(asmt,'random')
            % Random assignement
            rasStream = RandStream('mlfg6331_64', 'Seed',1);
            assigned = randi(rasStream,nb_boxes,1,nb_planes);
        elseif strcmp(asmt,'every other')
            % Every other box. Assumes more boxes than planes
            assert(nb>=nb_planes, 'Every other method of assigment works only if nb_boxes >= nb_planes');
            p=1;
            for box=1:2:nb
                assigned(p)=box;
                p=p+1;
                if p > nb_planes
                    break
                end
            end
        end
        
    end


end
