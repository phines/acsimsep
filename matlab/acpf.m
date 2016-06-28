function [ps, converged] = acpf(ps,PartFact,opt)
% run AC power flow for ps. Based on options it will do load shedding to
% make an unsolvable system solvable.

if nargin < 2, PartFact = []; end
if nargin < 3, opt = psoptions; end
verbose = opt.verbose;
C = psconstants;
EPS = 1e-6;
% Extract stuff from ps
nbus = size(ps.bus,1);
nbr = size(ps.branch,1);
G = ps.bus_i(ps.gen(:,1));
D = ps.bus_i(ps.shunt(:,1));
% F = ps.bus_i(ps.branch(:,C.br.from));
% T = ps.bus_i(ps.branch(:,C.br.to));
Vmag = ps.bus(:,C.bu.Vmag);
% get Ybus and Sbus from ps
[~, Sd, Sg] = getSbus(ps);
[Ybus, Yf, Yt] = getYbus(ps);
% Convert loads to simple constant power
Sd = sum(Sd,2);
% Create bus versions of Sd,Sg
Sd_bus = full(sparse(D,1,Sd,nbus,1));
Sg_bus = full(sparse(G,1,Sg,nbus,1));
ge_status = (ps.gen(:,C.ge.status) == 1);
Pg_max = ps.gen(:,C.ge.Pmax).*ge_status;
Pgmax_bus = full(sparse(G,1,Pg_max,nbus,1));
% Find the bus types
pq  = ps.bus(:,C.bu.type)==C.PQ;
ref = ps.bus(:,C.bu.type)==C.REF;
pv  = ps.bus(:,C.bu.type)==C.PV;
if sum(ref) == 0 || sum(ref) > 1
    warning('Reference bus not specified correctly.')
end
if opt.pf.PartFactFlag && isempty(PartFact)
    % participation factors for generators are not provided. Default is
    % proportiante to Pg_max for all buses.
    PartFact = get_part_fact_sub(ps);
end
% find subgraphs 
nodes = ps.bus(:,C.bu.id);
br_st = (ps.branch(:,C.br.status) == 1);
links = ps.branch(br_st, [C.br.from, C.br.to]);
[graphNos,n_sub,linkNos_con] = find_subgraphs(nodes,links);
% set up linkNos such that it contains all branches (connected or tripped)
linkNos = zeros(nbr,1);
linkNos(br_st) = linkNos_con;
% zero flows and generations for tripped branches and generators
ps.branch(~br_st,[C.br.Imag_f,C.br.Imag_t,C.br.Pf,C.br.Qf,C.br.Pt,C.br.Qt]) = 0;
ps.gen(~ge_status,[C.ge.P,C.ge.Q]) = 0;
gen_buses = sparse(G,1,double(ge_status),nbus,1); % find buses with available generators
if opt.pf.CascadeControl
    % set the power plant ramp rates (for rebalance after load shedding)
    ramp_rate = ps.gen(:,C.ge.ramp_rate_up)/60; % ramp rate in MW/second
    if all(ramp_rate==0)
        ramp_rate_MW_per_min = max(1,Pg_max*.05); % assume that all plants can ramp at 5% per minute. 
                                                % for a 100 MW plant, this
                                                % would be 5 MW/min. Reasonable
        ramp_rate = ramp_rate_MW_per_min/60;
    end
    ramp_dt = opt.sim.fast_ramp_mins*60; % do a minute of ramping before gen tripping/load shedding
    ramp_limits = ramp_rate*ramp_dt;
end

for k = 1:n_sub
    % find buses in the island
    busi_sub = find(graphNos == k);
    busID_sub = ps.bus(busi_sub,1);
    % find branches in the island
    br_sub = find(linkNos == k);
    nbus_sub = length(busi_sub);
    theta_sub = zeros(nbus_sub,1);
    ref_sub = ref(busi_sub);
    pq_sub = pq(busi_sub);
    npq_sub = sum(pq_sub);
    pv_sub = pv(busi_sub);
    gen_buses_sub = gen_buses(busi_sub);
    this_Pgmax_bus = Pgmax_bus(busi_sub);
    while sum(ref_sub) == 0
        % assign the bus with the biggest generation capacity as the reference
        [this_Pgmax,idx] = max(this_Pgmax_bus);
        if this_Pgmax > 0 && gen_buses_sub(idx) > 0
            ref_sub(idx) = true;
        else
            this_Pgmax_bus(idx) = 0;
        end
        if all(this_Pgmax_bus == 0)
            break
        end
    end
    Vmag_sub = Vmag(busi_sub);
    Ybus_sub = Ybus(busi_sub,busi_sub);
    Sg_bus_sub = Sg_bus(busi_sub);
    Pg_bus_sub = real(Sg_bus_sub);
    Sd_bus_sub = Sd_bus(busi_sub);
    if sum(Pg_bus_sub) < EPS
        % no generator in this island
        P_out = real(sum(Sd_bus_sub))*ps.baseMVA;
        Q_out = imag(sum(Sd_bus_sub))*ps.baseMVA;
        if verbose
            fprintf('Island %d of %d is black (%d buses): no generation. Total load: %.2f MW and %.2f MVAr. \n', ...
                k,n_sub,nbus_sub,P_out,Q_out);
        end
        ps = save_results_black(ps,br_sub,busID_sub);
    elseif abs(sum(Sd_bus_sub))<EPS
        % no load in this island
        if verbose
            fprintf('Island %d of %d is black (%d buses): no load. \n',k,n_sub,nbus_sub);
        end
        ps = save_results_black(ps,br_sub,busID_sub);
    else
        % build the decision vector
        ix = struct;
        if opt.pf.PolarFlag
            ix.theta_sub = 1:(nbus_sub-1);
            ix.Vmag_sub  = (1:npq_sub) + (nbus_sub-1);
            if opt.pf.PartFactFlag
                ix.rho_sub = 1 + (npq_sub + nbus_sub - 1); % generator ramping variable
                nx = npq_sub + nbus_sub;
            else
                nx = npq_sub + nbus_sub - 1;
            end
            x0 = zeros(nx,1);
            x0(ix.theta_sub) = theta_sub(~ref_sub);
            x0(ix.Vmag_sub)  = 1;
            if opt.pf.PartFactFlag
                x0(ix.rho_sub)   = 0;
            end
            if any(x0(ix.Vmag_sub))==0
                x0(ix.Vmag_sub) = 1;
            end
            if opt.pf.PartFactFlag
                PartFact_sub = PartFact(busi_sub);
            else
                PartFact_sub = [];
            end
            % set up a virtual function to solve
            mismatch = @(newx)mismatch_polar(newx,Ybus_sub,Vmag_sub,Sg_bus_sub,Sd_bus_sub, ...
                pq_sub,pv_sub,ref_sub,opt.pf.PartFactFlag,PartFact_sub);
        else
            %{
            % rectangular coordinate power flow
            ix.e = 1:(nbus-1);
            ix.f  = (1:(nbus-1)) + max(ix.e);
            if PartFactFlag
                ix.rho = 1 + max(ix.f); % generator ramping variable
                nx = max(ix.rho);
            else
                nx = max(ix.f);
            end
            x = zeros(nx,1);

            x(ix.e) = ones(1,nbus-1);
            x(ix.f)  = zeros(1,nbus-1);
            if PartFactFlag
                x(ix.rho)   = 0;
            end
            % set up a virtual function to solve
            g = @(newx)mismatch_cartesian(newx,Ybus,Vmag,Sg_bus,Sd_bus,pq,pv,ref,PartFactFlag,PartFact);
            %}
        end

        %% Solve
        % try to solve the power flow problem
        if verbose
            fprintf('Solving power flow for island %d of %d (%d buses):\n',...
                k,n_sub,nbus_sub);
        end
        factor_i_old = ps.shunt(:,C.sh.factor);
        Pglim_flag = false;
        while ~Pglim_flag
            counter = 0; % counts the number of times power flow did not converge
            while true % exit from this loop if power flow converges
                counter = counter + 1;
                if counter >= 5 % if power flow is not converging for five times, the whole island goes black
                    is_black = true;
                    break
                end
                [x,converged] = nrsolve(mismatch,x0,opt);
                if converged
                    is_black = false;
                    break
                else
                    if verbose, fprintf('Power flow did not converge on island %d of %d.\n',k,n_sub); end
                    if ~opt.pf.CascadeControl
                        is_black = true;
                        break
                    else
                        % find sensitivity to laod shedding for this island
                        [beta, beta_S, xm] = find_sens_fact(mismatch,x0,Sd_bus_sub,pq_sub,ref_sub,opt.pf.PartFactFlag,verbose);
                        if beta < opt.pf.tolerance
                            if verbose
                                disp('Solution found to power flow while minimizing norm of mismatch.')
                            end
                            x = xm; 
                            converged = true;
                            is_black = false;
                            break
                        end
                        % shed load on a number of buses to set distance to zero
                        ps = shed_load(ps,busi_sub,beta,beta_S,verbose);
                        opt.pf.loss_factor = 0.1;
                        ps = rebalance(ps,graphNos,ramp_limits,opt,k,'ac');
                        ge_status = ps.gen(:,C.ge.status);
                        PartFact_sub = get_part_fact_sub(ps,busi_sub);
                        % get the new Sg and Sd to build Sg_bus_sub and Sd_bus_sub
                        [~, Sd, Sg] = getSbus(ps);
                        Sd = sum(Sd,2);
                        % Create bus versions of Sd,Sg
                        Sd_bus = full(sparse(D,1,Sd,nbus,1));
                        Sg_bus = full(sparse(G,1,Sg,nbus,1));
                        Sg_bus_sub = Sg_bus(busi_sub);
                        Sd_bus_sub = Sd_bus(busi_sub);
                        if sum(Sd_bus_sub) == 0 
                            if sum(real(Sg_bus_sub)) ~= 0
                                error('Something is wrong here!')
                            end
                            is_black = true;
                            break
                        end
                        mismatch = @(newx)mismatch_polar(newx,Ybus_sub,Vmag_sub,Sg_bus_sub,Sd_bus_sub, ...
                            pq_sub,pv_sub,ref_sub,opt.pf.PartFactFlag,PartFact_sub);
                    end
                end
            end
            if is_black
                ps = save_results_black(ps,br_sub,busID_sub);
                break
            end
            if opt.pf.check_Pg
                % check if Pg is inside bounds
                [Pglim_flag, PartFact_sub, need_load_shed] = check_Pg(ps,busi_sub,Ybus_sub,Sd_bus_sub,Vmag_sub,...
                    ge_status,x,ix,pq_sub,ref_sub,PartFact_sub);
                if ~Pglim_flag
                    if verbose
                        fprintf(' Pg is outside its limits. Solving again ...\n')
                    end
                    if need_load_shed
                        if verbose
                            fprintf(' Pmax is not enough in this island. shedding 5%% of the load.\n')
                        end
                        ish = ismember(ps.shunt(:,C.sh.bus),busID_sub);
                        ps.shunt(ish,C.sh.factor) = max(ps.shunt(ish,C.sh.factor)-0.05, 0);
                        [~, Sd, Sg] = getSbus(ps);
                        Sd = sum(Sd,2);
                        % Create bus versions of Sd,Sg
                        Sd_bus = full(sparse(D,1,Sd,nbus,1));
                        Sg_bus = full(sparse(G,1,Sg,nbus,1));
                        Sg_bus_sub = Sg_bus(busi_sub);
                        Sd_bus_sub = Sd_bus(busi_sub);
                        if sum(Sd_bus_sub) < EPS
                            is_black = true;
                            break
                        end
                    end
                    mismatch = @(newx)mismatch_polar(newx,Ybus_sub,Vmag_sub,Sg_bus_sub,Sd_bus_sub, ...
                             pq_sub,pv_sub,ref_sub,opt.pf.PartFactFlag,PartFact_sub);
                end
            else
                Pglim_flag = true;
            end
        end
        if verbose
            if ~is_black
                factor_i_new = ps.shunt(:,C.sh.factor);
                P_shed = sum(ps.shunt(:,C.sh.P).*(factor_i_old - factor_i_new));
                Q_shed = sum(ps.shunt(:,C.sh.Q).*(factor_i_old - factor_i_new));
                if P_shed > EPS || Q_shed > EPS
                    fprintf('Total load shed on island %d of %d: %.2f MW and %.2f MVAr.\n', ...
                        k,n_sub,P_shed,Q_shed);
                end
            else
                fprintf('Island %d of %d is black.\n',k,n_sub);
            end
        end
        if is_black
                ps = save_results_black(ps,br_sub,busID_sub);
            else
                ps = save_results(ps,Vmag_sub,Ybus_sub,Yf,Yt,...
                    br_sub,busID_sub,Sd_bus_sub,x,ix,pq_sub,ref_sub,ge_status,opt);
        end
    end
end
pg = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));
pd = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
Ploss = pg-pd;
if verbose, fprintf('Total losses: %.2g percent\n',Ploss/pd*100); end
end

function PartFact_sub = get_part_fact_sub(ps,busi_sub)
C = psconstants;
EPS = 1e-4;
nbus = size(ps.bus,1);
if nargin < 2
    busi_sub = 1:nbus;
end
G = ps.bus_i(ps.gen(:,1));
Pg_max = ps.gen(:,C.ge.Pmax).*ps.gen(:,C.ge.status);
Pgmax_bus = full(sparse(G,1,Pg_max,nbus,1));
PartFact_sub = Pgmax_bus(busi_sub);
if any(ps.gen(:,C.ge.P) < ps.gen(:,C.ge.Pmin)-EPS | ...
        ps.gen(:,C.ge.P) > ps.gen(:,C.ge.Pmax)+EPS)
%     error('Why is any generation outside their limtis already?')
end
end

function ps = save_results(ps,Vmag_sub,Ybus_sub,Yf,Yt,br_sub,...
    busID_sub,Sd_bus_sub,x,ix,pq_sub,ref_sub,ge_status,opt)
% save results from power flow in the ps structure 
C = psconstants;
busi_sub = ps.bus_i(busID_sub);
nbus = size(ps.bus,1);
F = ps.bus_i(ps.branch(:,C.br.from));
T = ps.bus_i(ps.branch(:,C.br.to));
theta_sub = zeros(length(busi_sub),1);
if opt.pf.PolarFlag
    Vmag_sub(pq_sub) = x(ix.Vmag_sub);
    theta_sub(~ref_sub) =  x(ix.theta_sub);
    theta_sub(ref_sub)  = 0;
end
% update results in ps only for this island
% update Vmag and Vang in ps.bus
ps.bus(busi_sub,C.bu.Vmag) = Vmag_sub;
ps.bus(busi_sub,C.bu.Vang) = theta_sub;
% update flows
V_sub = Vmag_sub.*exp(1i*theta_sub);
V = zeros(nbus,1);
V(busi_sub) = V_sub;

Yf_sub = Yf(br_sub,busi_sub);
Yt_sub = Yt(br_sub,busi_sub);
If_sub = Yf_sub*V_sub;
It_sub = Yt_sub*V_sub;
F_sub = F(br_sub);
Sf_sub = V(F_sub) .* conj(If_sub);
T_sub = T(br_sub);
St_sub = V(T_sub) .* conj(It_sub);
ps.branch(br_sub,C.br.Imag_f) = abs(If_sub);
ps.branch(br_sub,C.br.Imag_t) = abs(It_sub);
ps.branch(br_sub,C.br.Pf) = real(Sf_sub) * ps.baseMVA;
ps.branch(br_sub,C.br.Qf) = imag(Sf_sub) * ps.baseMVA;
ps.branch(br_sub,C.br.Pt) = real(St_sub) * ps.baseMVA;
ps.branch(br_sub,C.br.Qt) = imag(St_sub) * ps.baseMVA;
ps.branch(abs(ps.branch) < 1e-6) = 0;
% update ps.gen
Sg_bus_sub_final = V_sub.*conj(Ybus_sub*V_sub) + Sd_bus_sub;
Pg_bus_sub = real(Sg_bus_sub_final);
Qg_bus_sub = imag(Sg_bus_sub_final);
gen_busID = ps.gen(:,C.ge.bus);
gen_busi = ps.bus_i(gen_busID);
for j = 1:length(busi_sub)
    this_busi = busi_sub(j);
    gi = (gen_busi == this_busi & ge_status);
    if Pg_bus_sub(j)>1e-4 && sum(gi)==0
        error('There should be at least one generator on this bus.')
    end
    if V(this_busi) == 0.
        ps.gen(gi,C.ge.P) = 0;
        ps.gen(gi,C.ge.Q) = 0;
        ps.gen(gi,C.ge.status) = 0;
    else
        if sum(gi) == 1
            Pg_ratio = 1;
        else
            Pg_ratio = ps.gen(gi,C.ge.Pmax)/sum(ps.gen(gi,C.ge.Pmax));
        end
        ps.gen(gi,C.ge.P) = Pg_bus_sub(j) * Pg_ratio * ps.baseMVA;
        ps.gen(gi,C.ge.Q) = Qg_bus_sub(j) * Pg_ratio * ps.baseMVA;
    end
end
ps.gen(abs(ps.gen) < 1e-4) = 0;
end

function [Pglim_flag, PartFact_sub, need_load_shed] = check_Pg(ps,busi_sub,Ybus_sub,Sd_bus_sub,Vmag_sub,...
    ge_status,x,ix,pq_sub,ref_sub,PartFact_sub)
% find Pg to check if generations are in their limits
PartFact_sub_orig = PartFact_sub; % save participation factors
EPS = 1e-6;
C = psconstants;
nbus = size(ps.bus,1);
Vmag_sub(pq_sub) = x(ix.Vmag_sub);
theta_sub = zeros(length(busi_sub),1);
theta_sub(~ref_sub) =  x(ix.theta_sub);
theta_sub(ref_sub)  = 0;
V_sub = Vmag_sub.*exp(1i*theta_sub);
Sg_bus_sub = V_sub.*conj(Ybus_sub*V_sub) + Sd_bus_sub;
Pg_bus_sub = real(Sg_bus_sub);
% find Pgmax_bus and Pgmin_bus
G = ps.bus_i(ps.gen(:,1));
Pg_max = ps.gen(:,C.ge.Pmax).*ge_status/ps.baseMVA;
Pgmax_bus = full(sparse(G,1,Pg_max,nbus,1));
Pgmax_bus_sub = Pgmax_bus(busi_sub);
Pg_min = ps.gen(:,C.ge.Pmin).*ge_status/ps.baseMVA;
Pgmin_bus = full(sparse(G,1,Pg_min,nbus,1));
Pgmin_bus_sub = Pgmin_bus(busi_sub);
% set participation factor zero for violating buses 
need_load_shed = false;
if any(Pg_bus_sub < Pgmin_bus_sub-EPS | Pg_bus_sub > Pgmax_bus_sub+EPS)
    Pglim_flag = false;
    PartFact_sub(Pg_bus_sub < Pgmin_bus_sub) = 0;
    PartFact_sub(Pg_bus_sub > Pgmax_bus_sub) = 0;
    if all(PartFact_sub == 0)
        need_load_shed = true;
        PartFact_sub = PartFact_sub_orig;
    end
else
    Pglim_flag = true;
end 
end


function ps = save_results_black(ps,br_sub,busID_sub)
C = psconstants;
busi_sub = ps.bus_i(busID_sub);
% set flows to zero
ps.branch(br_sub,[C.br.Imag_f,C.br.Imag_t,C.br.Pf,C.br.Qf,C.br.Pt,C.br.Qt]) = 0;
% set voltages to zero
ps.bus(busi_sub,[C.bu.Vmag,C.bu.Vang]) = 0;
% set generations to zero
i_gen = ismember(ps.gen(:,C.ge.bus),busID_sub);
ps.gen(i_gen,[C.ge.P,C.ge.Q,C.ge.status]) = 0;
% set load factors to zero
i_sh = ismember(ps.shunt(:,C.sh.bus),busID_sub);
ps.shunt(i_sh,C.sh.factor) = 0;
end
