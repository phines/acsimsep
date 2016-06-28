function [ps_agents, ps, MW_lost, imbalance] = distributed_control(ps_agents,ps,sub_grids,ramp_rate,it_no,opt,pf_model)
% distributed emergency control
C = psconstants;
EPS = 1e-4;
nbus = size(ps.bus,1);
ramp_dt = opt.sim.fast_ramp_mins*60;
ramp_limits = ramp_rate * ramp_dt;
verbose = opt.verbose;
Pd0_sum = sum(ps.shunt(:,C.sh.P) .* ps.shunt(:,C.sh.factor));
Pg0_sum = sum(ps.gen(:,C.ge.P) .* ps.gen(:,C.ge.status));
G = ps.bus_i(ps.gen(:,C.ge.bus));
D = ps.bus_i(ps.shunt(:,C.sh.bus));

for i = 1:nbus
    % all agents collect data from the ps structure (measurements)
    collect_data(i,ps)
    % reset capacity and ramp rates for each agent
    reset_agent(i,ramp_limits);
end

% each agent checks for overloads on the lines connected to it on its from side
for bus_i_f = 1:nbus
    % find lines connected to the bus (on the from side)
    [br_ID_f, ~] = find(ps_agents(bus_i_f).branch(:,C.br.f) == ps_agents(bus_i_f).bus_id); % find branches that this bus is on the from side
    flow_max_agent = ps_agents(bus_i_f).branch(br_ID_f,opt.opf.contg_rate);
    idx = (abs(ps_agents(bus_i_f).branch(br_ID_f,C.br.Pf)) > flow_max_agent + EPS);
    if sum(idx) > 0
        br_over_id = br_ID_f(idx);
        bus_i_t = ps_agents(bus_i_f).bus_i(ps_agents(bus_i_f).branch(br_over_id,C.br.to));
        bus_i_t = full(bus_i_t);
        if verbose
            bus_id_f = ps_agents(bus_i_f).bus(bus_i_f,1);
            bus_id_t = ps_agents(bus_i_f).bus(bus_i_t,1);
            fprintf('Bus %d found overload on line(s): ', bus_id_f);
            fprintf('%d(%d-%d) ', ...
                [br_over_id';repmat(bus_id_f,1,length(br_over_id));bus_id_t']);
            fprintf('\n');
        end
        % pulling information from local neighborhood
        local_nei = unique([ps_agents([bus_i_f;bus_i_t]).loc_nei]);
        local_nei = local_nei(:)'; % make sure local_nei is a row vector
        for i = local_nei
            if i == bus_i_f
                continue
            end
            % prepare the local message
            msg_loc = prepare_msg_loc(ps_agents(i));
            % now send the message to the from bus (in charge of
            % solving)
            agent_send_msg(msg_loc,'local',bus_i_f)
            % add two to message numbers for both sender and receiver (once
            % requesting information and once receiving it)
            ps_agents(i).messages(it_no) = ps_agents(i).messages(it_no) + 2;
        end
        ps_agents(bus_i_f).messages(it_no) = ps_agents(bus_i_f).messages(it_no) + 2*(length(local_nei)-1); % -1 to exclude the agent itself from the local neighborhood
        OptVarBusID = ps_agents(bus_i_f).bus(local_nei,1); % bus IDs of the local neighborhood
        approved = false; % this will change to true if the optimization is solved and there is enough capacity for implementation
        continue_flag = false;
        while ~approved
            if opt.sim.use_mpc
                OptVarBus_I = ps_agents(bus_i_f).bus_i(OptVarBusID);
                [delta_Pd, delta_Pg] = solve_mpc_emergency(ps_agents(bus_i_f),OptVarBus_I,ramp_limits,opt);
            else
                [delta_Pd,delta_Pg] = emergency_control_dec(ps_agents(bus_i_f),OptVarBusID,opt);
            end
            % do some checks 
            if abs(sum(delta_Pd)-sum(delta_Pg)) > 1e-3
                warning('load shedding is not equal to generation reduction. Rebalance needed!')
            end
            if abs(sum(delta_Pd)) < EPS || abs(sum(delta_Pg)) < EPS
                continue_flag = true;
                if verbose
                    fprintf('   zero load shedding/generator reduction found.\n')
                end
                break
            end
            % make sure delta_Pd and delta_Pg are only non-zero on the
            % local neighborhood
            if ~isempty(setdiff(D(delta_Pd ~= 0), local_nei)) || ...
                    ~isempty(setdiff(G(delta_Pg ~= 0), local_nei))
                error('load/generation shedding outside local neighborhood.')
            end
            % check to see if the local agents have capacity
            approved = check_capacity(delta_Pd,delta_Pg,bus_i_f,local_nei,verbose);
            ps_agents(bus_i_f).messages(it_no) = ps_agents(bus_i_f).messages(it_no) + 2*(length(local_nei)-1);
            for k = local_nei
                if k == bus_i_f
                    continue
                end
                ps_agents(k).messages(it_no) = ps_agents(k).messages(it_no) + 2;
            end
        end
        if continue_flag 
            continue 
        end
        % now implement solution on ps and change capacity for
        % agents
        if verbose  
            fprintf('  sending solution from bus %d to the local neighborhood...\n', ...
                ps_agents(bus_i_f).bus_id); 
        end
        update_agent_data(delta_Pd,delta_Pg,bus_i_f,it_no);
    end
end
% implement all solutions from each agent on ps
ps = implement_solution(ps,ps_agents,verbose);
Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
Pg_sum = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));

% compute the imbalance in total delta_Pg and total delta_Pd
imbalance = abs( (Pg_sum-Pg0_sum) - (Pd_sum-Pd0_sum) ); % the imbalance that this control caused in the system
if imbalance < EPS
    imbalance = 0;
else
    % the system needs a rebalance
    ps = rebalance(ps,sub_grids,ramp_limits,opt,[],pf_model);
    Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
end
% compute the total load lost in control
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
    ge_status = ps.gen(:,C.ge.status);
    Pg_max = ps.gen(:,C.ge.Pmax).*ge_status;
    Pg_min = ps.gen(:,C.ge.Pmin).*ge_status;
    if any( Pg>Pg_max+EPS | Pg<Pg_min-EPS )
        error('Pg is out of bounds'); 
    end
end
% error check
if abs(MW_lost_total - (Pd0_sum - Pd_sum)) > EPS
    error('Something is wrong!')
end


    function agent_send_msg(msg,msg_type,nei)
        % ps_agents_sub is a subset of agents that receive the message. The 
        % extended neighborhood contains the local neighborhood in this
        % implementation, so the message for extended neighborhood goes to the
        % local nodes as well.
        for j = nei
            br_id = msg.branch.status(:,1);
            % update the status of the lines connected to the sending bus (this is
            % the same for both local and extended neighborhood)
            ps_agents(j).branch(br_id,C.br.status) = msg.branch.status(:,2);
            if strcmp(msg_type,'local')
                br_id_f = msg.branch.Pf(:,1);
                br_id_t = msg.branch.Pt(:,1);
                % update flows
                ps_agents(j).branch(br_id_f,C.br.Pf) = msg.branch.Pf(:,2);
                ps_agents(j).branch(br_id_t,C.br.Pt) = msg.branch.Pt(:,2);
                % update the other end flow (this is not part of message passing,
                % but because this is a dc model, giving info about Pf gives info
                % also about Pt and vice versa)
                ps_agents(j).branch(br_id_t,C.br.Pf) = -msg.branch.Pt(:,2);
                ps_agents(j).branch(br_id_f,C.br.Pt) = -msg.branch.Pf(:,2);

                % update loads
                i_sh = msg.shunt.factor(:,1);
                ps_agents(j).shunt(i_sh,C.sh.factor) = msg.shunt.factor(:,2);

                % update generations
                ig = msg.gen.Pg(:,1);
                ps_agents(j).gen(ig,C.ge.Pg) = msg.gen.Pg(:,2);
                ps_agents(j).gen(ig,C.ge.status) = msg.gen.status(:,2);
            elseif strcmp(msg_type,'extended')
                % the status of branches are already updated 
            else
                error('unknown message type.')
            end
        end
    end

    function collect_data(i,ps)
        % Agents get measurements from the power grid. They update their model
        % based on the actual values from the ps structure.
        % update generators on the bus
        ig = (ps_agents(i).gen(:,C.ge.bus) == ps_agents(i).bus_id);
        ps_agents(i).gen(ig,C.ge.Pg) = ps.gen(ig,C.ge.Pg);
        ps_agents(i).gen(ig,C.ge.status) = ps.gen(ig,C.ge.status);
        % update load factor
        i_sh = (ps_agents(i).shunt(:,C.sh.bus) == ps_agents(i).bus_id);
        ps_agents(i).shunt(i_sh,C.sh.factor) = ps.shunt(i_sh,C.sh.factor);
        % update measured flows on the branches connected to the bus
        [br_id_f, ~] = find(ps_agents(i).branch(:,C.br.f) == ps_agents(i).bus_id); % find branches that this bus is on the from side
        ps_agents(i).branch(br_id_f,C.br.Pf) = ps.branch(br_id_f,C.br.Pf);
        [br_id_t, ~] = find(ps_agents(i).branch(:,C.br.t) == ps_agents(i).bus_id); % find branches that this bus is on the to side
        ps_agents(i).branch(br_id_t,C.br.Pt) = ps.branch(br_id_t,C.br.Pt);
        br_id = [br_id_f;br_id_t];
        ps_agents(i).branch(br_id,C.br.status) = ps.branch(br_id,C.br.status);
    end

    function reset_agent(i,ramp_limits)
        % reset all capacities, delta_Pg0 and delta_Pd0 for all agents 
        % generation and ramping
        ig = (ps_agents(i).gen(:,C.ge.bus) == ps_agents(i).bus_id);
        ps_agents(i).capacity.Pg = ps_agents(i).gen(ig,C.ge.Pg) .* ps_agents(i).gen(ig,C.ge.status);
        ps_agents(i).capacity.this_ramp = ramp_limits(ig) .* ps_agents(i).gen(ig,C.ge.status);
        ps_agents(i).gen(:,C.ge.ramp_rate_down) = ramp_limits;
        % loads
        ish = (ps_agents(i).shunt(:,C.sh.bus) == ps_agents(i).bus_id);
        ps_agents(i).capacity.sf = ps_agents(i).shunt(ish,C.sh.factor);
        % delta_Pg0_all and delta_Pd0_all (a model for the whole ps)
        ng = size(ps_agents(i).gen,1);
        nd = size(ps_agents(i).shunt,1);
        ps_agents(i).delta_Pg0_all = zeros(ng,1);
        ps_agents(i).delta_Pd0_all = zeros(nd,1);
        % delta_Pg and delta_sf (only for the agent -> to be implemented)
        ps_agents(i).delta_Pg = zeros(sum(ig),1);
        ps_agents(i).delta_sf = zeros(sum(ish),1);
    end

    function update_agent_data(delta_Pd,delta_Pg,bus_i_f,it_no)
        % find the agents that we need to communicate with
        % first generation
        gen_agents = full(unique(G(delta_Pg < -EPS))');
        for j = gen_agents
            bus_id = ps_agents(j).bus_id;
            % find the generation indices and update gen capacity and delta gen 
            ig = (ps_agents(j).gen(:,C.ge.bus) == bus_id);
            ps_agents(j).capacity.Pg = ps_agents(j).capacity.Pg + delta_Pg(ig);
            ps_agents(j).delta_Pg = ps_agents(j).delta_Pg + delta_Pg(ig);
            % update ramping capacity
            ps_agents(j).capacity.this_ramp = ps_agents(j).capacity.this_ramp + delta_Pg(ig);
            % add 1 to the number of messages for both sending and
            % receiving agents
            ps_agents(bus_i_f).messages(it_no) = ps_agents(bus_i_f).messages(it_no) + 1;
            ps_agents(j).messages(it_no) = ps_agents(j).messages(it_no) + 1;
        end
        load_agents = full(unique(D(delta_Pd < -EPS))');
        for j = load_agents
            bus_id = ps_agents(j).bus_id;
            % find the load indices and update shunt capacity and delta load
            ish = (ps_agents(j).shunt(:,C.sh.bus) == bus_id);
            delta_sf = delta_Pd./ps_agents(j).shunt(:,C.sh.P);
            r_zero_loads = (ps_agents(j).shunt(:,C.sh.P) == 0); % correct delta_sf for loads with zero power
            delta_sf(r_zero_loads) = 0;
            ps_agents(j).capacity.sf = ps_agents(j).capacity.sf + delta_sf(ish);
            ps_agents(j).delta_sf = ps_agents(j).delta_sf + delta_sf(ish);
            % add 1 to the number of messages for both sending and
            % receiving agents
            ps_agents(bus_i_f).messages(it_no) = ps_agents(bus_i_f).messages(it_no) + 1;
            ps_agents(j).messages(it_no) = ps_agents(j).messages(it_no) + 1;
        end
    end
   
    function approved = check_capacity(delta_Pd,delta_Pg,bus_i_f,local_nei,verbose)
        % check if the agents have enough capacity in load/generation to implement
        % a solution. If so, reserve the capacity
        % initialize output
        all_gen_approved = true;
        all_sh_approved = true;
        for j = local_nei
            % get some information
            bus_id = ps_agents(j).bus_id;
            ig = (ps_agents(j).gen(:,C.ge.bus) == bus_id); % find the generation indices for the local agent
            ish = (ps_agents(j).shunt(:,C.sh.bus) == bus_id); % find the load indices for the local agent
            % now find available capacity
            Pg_cap = ps_agents(j).capacity.Pg;
            sf_cap = ps_agents(j).capacity.sf;
            Pd_cap = ps_agents(j).shunt(ish,C.sh.P).*sf_cap;
            % take care of negative loads
            if any(Pd_cap < -EPS) 
                if all(ps_agents(j).shunt(ish,C.sh.P) > EPS)
                    error('Something is wrong in setting the load capacity.')
                end
                Pd_cap = Pd_cap .* (Pd_cap>EPS);
            end        
            this_ramp_cap = ps_agents(j).capacity.this_ramp;
            % check if after generation will be positive
            Pg_after = Pg_cap + delta_Pg(ig);
            if all(Pg_after > -EPS)
                gen_approved = true;
            else
                gen_approved = false;
            end
            ps_agents(bus_i_f).gen(ig,C.ge.P) = Pg_cap; % update agent on the from side whether or not this is approved
            % check if ramping is allowed
            if -delta_Pg(ig) > this_ramp_cap + EPS
                gen_approved = false;
            end
            ps_agents(bus_i_f).gen(ig,C.ge.ramp_rate_down) = this_ramp_cap;
            % check if the solving agent knows what the local agent will do
            if max(abs(ps_agents(bus_i_f).delta_Pg0_all(ig) - ps_agents(j).delta_Pg)) > EPS
                gen_approved = false;
                ps_agents(bus_i_f).delta_Pg0_all(ig) = ps_agents(j).delta_Pg;
            end
            all_gen_approved = all_gen_approved && gen_approved;

            ish = (ps_agents(j).shunt(:,C.sh.bus) == bus_id);
            Pd_after = Pd_cap + delta_Pd(ish);
            if all(Pd_after > -EPS) % same check for loads
                sh_approved = true;
            else
                sh_approved = false;
            end
            ps_agents(bus_i_f).shunt(ish,C.sh.factor) = sf_cap;
            % check if the solving agent knows what the local agent will do
            if max(abs(ps_agents(bus_i_f).delta_Pd0_all(ish) - ps_agents(j).delta_sf .* ps_agents(j).shunt(ish,C.sh.P))) > EPS
                sh_approved = false;
                ps_agents(bus_i_f).delta_Pd0_all(ish) = ps_agents(j).delta_sf .* ps_agents(j).shunt(ish,C.sh.P);
            end
            all_sh_approved = all_sh_approved && sh_approved;
        end
        approved = all_gen_approved && all_sh_approved;

        % print something
        if verbose
            if all_gen_approved
                fprintf('   Generation shedding approved by all agents.\n');
            else
                fprintf(['   Generation shedding not approved by all agents. ', ...
                        'Updating agent on bus %d ...\n'], ps_agents(bus_i_f).bus_id);
            end
            if all_sh_approved
                fprintf('   Load shedding approved by all agents.\n');
            else
                fprintf(['   Load shedding not approved by all agents. ', ...
                        'Updating agent on bus %d ...\n'], ps_agents(bus_i_f).bus_id);
            end
        end
    end

end
