function outargv = find_local_min_rob_ell(argv)
% Find a local minimum of obj fnt within a robustness ellipsoid.
%
% INPUTS
%         struct argv with the following fields. Some fields are used by
%         some algos and not others, and some fields are used by every
%         algo.
%         Used by every algo:
%         - HA           : hybrid automaton created with hsbenchamark1 or
%         hsbenchmark3
%         - h0           : inital point = [l0, t0, x0]
%         - tt           : total simulation time
%         - plotit       : 1 => plot trajectories
%         - max_nbse:  maximum nb of system evaluations (computed trajectories)
%         still allowed in this run of RED (= per SA/UR/NM sample)
%         - base_sample_rob: rob of sample at which local descent starts
%         - ell_params: cell of ellipsoid parameters
%               {1}: center, {2}: Pinv, {3}: the minimum dist computed in the
%               initial conditions
%         - red_descent_in_ellipsoid_algo : what algo to use to descend inside robustness ellipsoid
%         - red_min_ellipsoid_radius : if robustness ellipsoid radius is
%         lower than this, don't do search.
% 
%         Used by at least one algorithm: 
%         - one_ellipsoid_max_nbse : maximum nb of sys evals allower per local
%         minimization (e.g. per solution of Prob[W] for SQP). Optional. 
%         - complete_history : if 1, complete history to reach locunsafe.
%         - it      : If this is called in a loop, 'it' is the iteration nb.
%         Used to determine what color to plot in.
%         - z0      : Initial point for algo hfalsify_lin_hybrid_sys. Default : [x0 t0 0]
%         - constr_type: constraint type:  s_x(t) \in 'guards' or \in 'invariants' of locations
%         - testing_type :  Evaluate constraints, etc, on actual 'trajectory' (as generated by simulator)
%               or 'model' thereof
%         - formulation  : Constraints placed on 'max' over an interval, or at
%               every 'instant' in that interval
%         - use_slack_in_ge  : if 1, the constraint 'x \in E(x_0)' is
%             implemented as G_E(z) <= v. Else, it is implemented as G_E(z) <= 0.
%             (See 7b in paper).
%         - impose_loc_seq   : Optional. Sequence of locations we want the system to traverse.
%               This over-rides all other considerations: if this is provided, then
%               we forget about the unsafe set, descent set, the original trajectory,
%               all of it. Just follow this sequence of locations. The last entry
%               is the (fake) descent set.
%         - locHis: location history followed by trajectories starting in this ellipsoid
%         - couleur: plotting colour
%
% OUTPUTS
%     struct outargv with fields
%     - tt        Total simulation. Might be larger than input tt if this
%     supports ways of automatically increasing it.
%     - h_sol     [l0 t0 x_sol] where x_sol is from solution of Prob[W]
%     - z_sol     solution of Prob[W]
%     - nboptiter Nb of iterations done by optimizer to find local min
%     - fval      value of objective function at optimum: fval=obj(z_sol). 
%       Depending on the lcoal optimizer, this might or might not be the 
%       value of robustness corresponding to h_sol. E.g. for SQP, it is
%       not, since SQP objective fnt is the slack, not same as robustness. 
%       For UR, it is.
%     - ell_dist   min distance for robustness ellipsoid in set of initial conditions
%     - rc         return code.
%     - objective  'auxiliary' if the descent algo minimizes an auxiliary
%     objective function (that is not the robustness, e.g. F in the paper),
%     and 'robustness' if the descent algo minimizes the robustness value
%     directly.
%
%
%--------------------------------------------------------------------------


switch argv.red_descent_in_ellipsoid_algo
    case 'SQP'
        outargv = hfalsify_lin_hybrid_sys(argv);
        outargv.objective = 'auxiliary';
    case 'UR'
        outargv = UR_find_local_min_rob_ell(argv);
        outargv.objective = 'robustness';
    case 'NelderMead'
        outargv = NM_find_local_min_rob_ell(argv);
        outargv.objective = 'robustness';
    otherwise
        error(['Unrecognized type of local minimization ', argv.red_descent_in_ellipsoid_algo])
end


