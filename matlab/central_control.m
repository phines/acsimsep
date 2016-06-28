function [ps, MW_lost, imbalance] = central_control(ps,sub_grids,ramp_rate,opt,pf_model)

% Constants
C = psconstants;
EPS = 1e-4;
% Collect some data from the system
n = size(ps.bus,1);
m = size(ps.branch,1);
F = ps.bus_i(ps.branch(:,C.br.from));
T = ps.bus_i(ps.branch(:,C.br.to));
flow = ps.branch(:,C.br.Pf);
flow_max = ps.branch(:,opt.opf.contg_rate);
branch_st = ps.branch(:,C.br.status);
ge_status = ps.gen(:,C.ge.status);
Pg_max = ps.gen(:,C.ge.Pmax).*ge_status;
Pg_min = ps.gen(:,C.ge.Pmin).*ge_status;
G = ps.bus_i(ps.gen(:,1));
D = ps.bus_i(ps.shunt(:,1));
Pd0_sum = sum(ps.shunt(:,C.sh.P) .* ps.shunt(:,C.sh.factor));
Pg0_sum = sum(ps.gen(:,C.ge.P) .* ps.gen(:,C.ge.status));
comm_status = true(n,1);

% if there are overloads in the system, try to mitigate them
if any(abs(flow)>flow_max + EPS)
    no_change = false; % things will change in ps
    % Check the mismatch (in dc only)
    if strcmp(pf_model,'dc')
        mis_old = total_P_mismatch(ps);
        if abs(mis_old)>EPS
            error('System not balanced on entry to take_control_actions');
        end
    end
    % Figure out the ramp rate
    ramp_dt = opt.sim.fast_ramp_mins * 60; 
    ramp_limits = ramp_rate*ramp_dt;
    % Find the optimal load/gen shedding
    if opt.sim.use_mpc
        [delta_Pd, delta_Pg] = solve_mpc_emergency(ps,[],ramp_limits,opt);
    else
        [delta_Pd, delta_Pg] = emergency_control(ps,flow,branch_st,ramp_limits,comm_status,opt);
    end
    % If emergency control says that we should do something:
    if any(abs(delta_Pd)>EPS)
        if opt.verbose, print_load_shedding_results(ps,delta_Pg,delta_Pd); end
        % Compute the new amount of generation
        Pg = ps.gen(:,C.ge.P) + delta_Pg;
        % check if Pg is in its bounds if needed
        if opt.pf.check_Pg
            if any( Pg>Pg_max+EPS | Pg<Pg_min-EPS )
                error('Pg is out of bounds'); 
            end
        end
        % Compute the new load factor
        delta_lf = delta_Pd./ps.shunt(:,C.sh.P);
        delta_lf(isnan(delta_lf)) = 0;
        lf_new = ps.shunt(:,C.sh.factor) + delta_lf;
        if any(lf_new < -EPS | lf_new > 1)
            error('Load factors are out of bounds.')
        end
        % Implement the results
        ps.gen(:,C.ge.P) = Pg; % implement Pg
        ps.shunt(:,C.sh.factor) = lf_new; % implement shunt factor
        % record new Pg and Pd
        Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
        Pg_sum = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));
        % Get the imbalance caused by control 
        imbalance = abs( (Pg_sum-Pg0_sum) - (Pd_sum-Pd0_sum) );
        if imbalance < EPS
            imbalance = 0;
        else
            % the system needs a rebalance
            ps = rebalance(ps,sub_grids,ramp_limits,opt,[],pf_model);
            Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
        end
        MW_lost.control = Pd0_sum - Pd_sum;
        % run power flow
        if strcmp(pf_model,'dc')
            ps = dcpf(ps,sub_grids);
            MW_lost_total = MW_lost.control;
        elseif strcmp(pf_model,'ac')
            Pd_sum_before = Pd_sum;
            opt.pf.CascadeControl = true;
            ps = acpf(ps,[],opt);
            Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
            MW_lost.voltage_collapse = Pd_sum_before - Pd_sum;
            MW_lost_total = MW_lost.control + MW_lost.voltage_collapse;
        else
            error('Unknown power flow model.')
        end
        % check if Pg is in its bounds if needed
        if opt.pf.check_Pg
            Pg = ps.gen(:,C.ge.P);
            if any( Pg>Pg_max+EPS | Pg<Pg_min-EPS )
                error('Pg is out of bounds'); 
            end
        end
        % error check
        if abs(MW_lost_total - (Pd0_sum - Pd_sum)) > EPS
            error('Something is wrong!')
        end
    else
        no_change = true;
    end
else
    no_change = true;
end

if no_change
    MW_lost.control = 0;
    imbalance = 0;
    if strcmp(pf_model,'ac')
        MW_lost.voltage_collapse = 0;
    end
end




