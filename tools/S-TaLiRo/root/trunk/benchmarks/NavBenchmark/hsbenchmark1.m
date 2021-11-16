function HA = hsbenchmark1(id,init,A,unsafe,Av,reg)
% Function HA = hsbenchmark1(id,init,A,unsafe,Av,reg)
%
%   Navigation Benchmark from the HSCC 04 paper by Fehnker & Ivancic
% 
%   id - 0 do not plot (default)
%        1 plot environment (no text & no arrows)
%        2 plot environment (vector direction & arrows)
%        3 plot environment (HA location & arrows)
%        4 plot environment (vector direction + HA location & arrows)
%
%   init - initial continuous set. 
%       A structure with fileds: loc and cube
%           loc : the initial locations
%           cube : A hyper cube of the form 
%                   [x1_m x1_M; x2_m x2_M; x3_m x3_M; x4_m x4_M]
%       i.e., initial locations are
%                   loc x [x1_m x1_M; x2_m x2_M; x3_m x3_M; x4_m x4_M]
%
%   A - An array giving the direction of the constant vector in each
%       location. I.e.
%           System obeys x'(t) = As*x - Bs*u(i,j)
%           where u(i,j) = [sin(pi*A(i,j)); 
%                           cos(pi*A(i,j)]
%           and As, Bs depend on Av. See paper by Fehnker & Ivancic.
%       Special entries in A:
%           0 - unsafe cell
%           8 - reachable cell
%       Example: 
%           A = [0 2 4; 4 3 4; 2 2 8]
%
%   unsafe - An object of class unsafeset. See that class's help for
%   description.
%       Note: if unsafe is provided then the unsafe set defined in A is
%       ignored.
%
%   Av - An array used in the system dynamics. See A above.
%        If you want to use the default value enter [].
%
%   reg - other regions that should be ploted. This is a cell array of 2D
%         arrays. Eg.
%           reg{1} = [1 2; 1 2]
%
% OUTPUT:
%   HA : Hybrid Automaton structure
%       HA.loc(i).dyn : The dynamics in each location
%           0 - for linear with a constant drift
%               HA.loc(i).A : The matrix A_i of location i in eq. (2) 
%               HA.loc(i).b : The vector b_i of location i in eq. (2) 
%           1 - for any function providing the derivative of the system
%               HA.loc(i).f : The function pointer for eq. (1)
%                   Example: HA.loc(1).f = @(t,x) 2*x(1)^2+x(1)*x(2);
%       nonlinear:  dx/dt = f_i(x)                                     (1)
%       linear:     dx/dt = A_i*x+b_i                                  (2)
%
%       HA.adjList{i} : The adjacency list for each location i
%           Example: location 1 has transitions to locations 2 and 3
%                    then HA.adjList{1} = [2 3];
%       HA.guards(i,j).A; HA.guards(i,j).b;
%           The guard for the transition from location i to j.      
%           The guards are defined using a finite number of unions and 
%           intersections of halfspaces: a*x<=b. Example:
%               Gij = {x|(a_1*x<=b_1 /\ a_2*x<=b_2) \/ a_3*x<=b_3}
%           then
%                  First disjunct, on page 1
%               HA.guards(i,j).A(:,:,1) = [a_1; a_2];
%               HA.guards(i,j).b(:,1) = [b_1; b_2];
%                   2nd disjunct, on page 2
%               HA.guards(i,j).A(1,:,2) = a_3;
%               HA.guards(i,j).b(1,2) = b_3;
%           In detail, A is a matrix noc*nos*nod where noc is the number
%           of conjunctions, nos is the number of continuous states and
%           nod is the number of disjunctions. Note that the predicates
%           for the halfspaces must be in disjunctive normal form. Each
%           row describes a conjunct of the clause.
%       HA.invariants : list of invariant sets, each a struct w/ fields
%           A and b, s.t. Inv(l) = {x| Ax <= b}
%       HA.unsafe : same as input (or default, if input unsafe not given)
%       HA.target.A; HA.target.b; (optional)
%           The target set. The data structure is the same as for the 
%           guards above.
%

% Georgios Fainekos - ASU - Last update 2010.10.03

% History:
% 2010.10.03 - GF - Added support for the inital location (at last!)
% 2010.09.29 - GF - Added support for general unsafe sets



if nargin==0 || isempty(id), id=0; end
if nargin<=1 || isempty(init)
    HA.init.loc = 6; 
    HA.init.cube = [2, 3; 1, 2; -1, 1; -1, 1]; 
else
    HA.init = init; 
end
if nargin<=2 || isempty(A), A = [0 2 4; 4 3 4; 2 2 8]; end
if nargin<=3, unsafe = []; end
if nargin<=4 || isempty(Av), Av = [-1.2 0.1; 0.1 -1.2]; end
if nargin<=5, reg = []; end

if length(Av)==4
    As = Av;
    Bs = zeros(4,2);
    Bs(3:4,1:2) = [-1.2 0.1; 0.1 -1.2];
else
    As = zeros(4);
    As(1,3) = 1;
    As(2,4) = 1;
    As(3:4,3:4) = Av;
    Bs = zeros(4,2);
    Bs(3:4,1:2) = Av;
end

%Houssam
% [V,D]=eig(As); D(1,1)=0.01;D(2,2)=-0.01;As=V*D*inv(V);

unsafecolor =  'r';
[nx,ny] = size(A);
if id
    figure
    plot(polytope(ProdTop2Polytope(HA.init.cube(1:2,1:2))),'color', 'g');
    hold on 
    if ~isempty(unsafe)
        %         plot(polytope(unsafe.A(:,1:2),unsafe.b),'r');
        nod = length(unsafe.descriptions);
        for i = 1:nod
            iset = unsafe.descriptions(i).set;
            % p = unsafe.pseudo_indicator{1,i};
            % p2D = @(x,y) p([x y]);% ezplot doesn't work with for some
            % reason
            if isa(iset,'ball')
                p2D = @(x,y) norm([x; y] - iset.center(1:2))- iset.radius;
                handle_unsafe = ezplot(p2D);
                set(handle_unsafe,'LineColor',unsafecolor);                
            elseif isa(iset, 'polyhedron')
                plot(polytope(unsafe.descriptions(1).set.H(:,1:2),unsafe.descriptions(1).set.K),'color', unsafecolor);
            end
        end
    end
    if ~isempty(reg)
        for ii = 1:length(reg)
            plot(polytope(ProdTop2Polytope(reg{ii})),'color', 'c');
            hold on 
        end
    end
    for ii = 1:nx
        for jj = 1:ny
            xx = jj-0.5;
            yy = nx+0.5-ii;
            str = [];
            if A(ii,jj)==0 && isempty(unsafe)
                plot(polytope(ProdTop2Polytope([jj-1,jj;nx-ii,nx-ii+1])),'color', 'r');
                str = 'Unsafe';
                xx = jj-0.8;
            elseif A(ii,jj)==8
                plot(polytope(ProdTop2Polytope([jj-1,jj;nx-ii,nx-ii+1])),'color', 'y');
                str = 'Goal';
                xx = jj-0.8;
            else
                if id>1
                    str = num2str(A(ii,jj));
                end
                if id>1
                    arrow([xx-0.25 yy+0.25],[xx-0.25 yy+0.25]+vv(A(ii,jj))'*0.25,8);
                end
            end
            if id==3
                xx = jj-0.5;
                yy = nx+0.5-ii;
                str = num2str((nx-ii)*ny+jj);
            end
            if id==4
                str = ['(',str,',',num2str((nx-ii)*ny+jj),')']; %#ok<AGROW>
            end
            if ~isempty(str)
                hh = text(xx,yy,str);
                set(hh,'FontSize',12)
            end
        end
    end
    axis([0 nx 0 ny])
    axis square
    set(gca,'XTick',0:nx)
    set(gca,'YTick',0:ny)
    grid on
    ylabel('x_2')
    xlabel('x_1')
end

nLoc = nx*ny;

for ii = 1:nLoc
    HA.adjList{ii} = [];
end

ii = 1; % HA location
for j = 1:nx
    for k  = 1:ny
        % Location Dynamics
        HA.loc(ii).dyn = 0; 
        HA.loc(ii).A = As;
        HA.loc(ii).b = -Bs*vv(A(nx+1-j,k));
        HA.loc(ii).f = @(t,x) HA.loc(ii).A*x+HA.loc(ii).b;
        HA.loc(ii).ftest = @(t,x) -Av*vv(A(nx+1-j,k));
        if A(nx+1-j,k)==8
            tloc = ii;
        end
        % Guards and Transitions
        if (j < nx)
            HA.adjList{ii} = [HA.adjList{ii}, ii+nx];
            HA.guards(ii,ii+nx).A = [0 -1 0 0];
            HA.guards(ii,ii+nx).b = -j;
            HA.adjList{ii+nx} = [HA.adjList{ii+nx}, ii];
            HA.guards(ii+nx,ii).A = [0 1 0 0];
            HA.guards(ii+nx,ii).b = j;
        end
        if (k < ny)
            HA.adjList{ii} = [HA.adjList{ii}, ii+1];
            HA.guards(ii,ii+1).A = [-1 0 0 0];
            HA.guards(ii,ii+1).b = -k;
            HA.adjList{ii+1} = [HA.adjList{ii+1}, ii];
            HA.guards(ii+1,ii).A = [1 0 0 0];
            HA.guards(ii+1,ii).b = k;
        end
        % Invariant sets
        g = HA.guards(ii,:);
        invA=[]; invb=[];
        for nhbr=1:length(g)
            if ~isempty(g(nhbr).A)
                invA = [invA; -g(nhbr).A];
                invb = [invb; -g(nhbr).b];
            end
        end
        HA.invariants(ii).A = invA;
        HA.invariants(ii).b = invb;
        
        ii = ii+1;
    end
end

% Add diagonal Guards and Transitions
% ii = 1; % HA location
% for j = 1:nx
%     for k  = 1:ny
%         if (j < nx)&(k < ny)
%             idx = ii+nx+1;
%             HA.adjList{ii} = [HA.adjList{ii}, idx];
%             HA.guards(ii,idx).A = [-1 0 0 0; 0 -1 0 0];
%             HA.guards(ii,idx).b = [-k; -j];
%             HA.adjList{idx} = [HA.adjList{idx}, ii];
%             HA.guards(idx,ii).A = [1 0 0 0; 0 1 0 0];
%             HA.guards(idx,ii).b = [k; j];
%             if (j > 1)&(k > 1)
%                 idx = ii+nx-1;
%                 HA.adjList{ii} = [HA.adjList{ii}, idx];
%                 HA.guards(ii,idx).A = [1 0 0 0; 0 -1 0 0];
%                 HA.guards(ii,idx).b = [k; -j];
%                 HA.adjList{idx} = [HA.adjList{idx}, ii];
%                 HA.guards(idx,ii).A = [-1 0 0 0; 0 1 0 0];
%                 HA.guards(idx,ii).b = [-k; j];
%             end
%         end
%         ii = ii+1;
%     end
% end

% unsafe set
if isempty(unsafe)
    [ju,iu] = find(A==0);
    ju = ny+1-ju;
    nu = length(iu);
    if nu==1
        P = polytope([iu-1,ju-1;iu,ju-1;iu,ju;iu-1,ju]);
        [Ad,bd] = double(P);
        HA.unsafe.A = zeros(4);
        HA.unsafe.A(:,1:2) = Ad;
        HA.unsafe.b = bd;
        HA.unsafe.loc = 4;
    elseif nu>1
        error('Multiple unsafe sets are not supported yet');
    end
else
    HA.unsafe = unsafe;
end

HA.ode_solver = 'ode45';
HA.simulator = @hasimulator1;

HA.type = 'hautomaton';

% target set
[ju,iu] = find(A==8);
ju = ny+1-ju;
nu = length(iu);
if nu==1
    P = polytope([iu-1,ju-1;iu,ju-1;iu,ju;iu-1,ju]);
    [Ad,bd] = double(P);
    HA.target.A = zeros(4);
    HA.target.A(:,1:2) = Ad;
    HA.target.b = bd;
elseif nu>1
    error('Multiple sets to be reached are not supported yet');
end

% Remove transitions to the target set
if nu>0
    for ii = 1:nLoc
        jj = find(HA.adjList{ii}==tloc);
        HA.adjList{ii}(jj) = [];
    end
end

function val = vv(x)
val = [sin(x*pi/4); cos(x*pi/4)];
end

end

