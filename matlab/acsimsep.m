function [is_blackout,br_outages_en,MW_lost,n_msg,record_data] = acsimsep(ps,br_outages_ex,bus_outages,opt)
% a cascading failure simulator with AC power flow
% n_msg is an nx1 vector of the maximum number of messages per iteration
% for each agent during the simulation (in case of distributed control)

if nargin < 4, opt = psoptions; end
% initialize stuff 
is_blackout = false;
br_outages_en = zeros(0,2);
MW_lost = struct('rebalance',0,'voltage_collapse',0,'control',0);
imbalance = 0;
ps.relay = relay_settings(ps,false,true);
% some constants
EPS = 1e-4;
% NO_SEP = 0;
BIG_SEP = 2;
% SMALL_SEP = 1;
verbose = opt.verbose;
stop_threshold = opt.sim.stop_threshold;
% record branch outages if needed
if opt.sim.debug
    file_name = 'br_outages_log.txt';
    if exist(file_name,'file')
        fid = fopen(file_name,'a');
    else
        fid = fopen(file_name,'w');
    end
    fprintf(fid,'%s: ',datestr(clock));
    fprintf(fid,'%d, ',br_outages_ex);
    fprintf(fid,'\n');
    fclose(fid);
end

% Grab some useful data
C = psconstants;
Pd0 = ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor);
Qd0 = ps.shunt(:,C.sh.Q).*ps.shunt(:,C.sh.factor);
Pd0_sum = sum(Pd0);
Qd0_sum = sum(Qd0);
nbus = size(ps.bus,1);
br_status = (ps.branch(:,C.br.status) == 1);
nbr0 = sum(br_status);
F = ps.bus_i(ps.branch(:,1));
T = ps.bus_i(ps.branch(:,2));
D = ps.bus_i(ps.shunt(:,1)); % load index
G = ps.bus_i(ps.gen(:,1));   % gen index
ge_status = (ps.gen(:,C.ge.status) == 1);
Pg_max = ps.gen(:,C.ge.Pmax).*ge_status;
Pg_min = ps.gen(:,C.ge.Pmin).*ge_status;
Pg   = ps.gen(:,C.ge.P);
if ~isfield(ps,'bus_i'), ps = updateps(ps); end
dt_max = opt.sim.dt; % maximum simulation time

% set up agents if this is distributed
if strcmp(opt.sim.control_method,'distributed_control')
    ps_agents = set_up_agents(ps,opt);
    if verbose, fprintf('setting up agents...\n'); end
end

% set the power plant ramp rates
ramp_rate = ps.gen(:,C.ge.ramp_rate_up)/60; % ramp rate in MW/second
if all(ramp_rate==0)
    ramp_rate_MW_per_min = max(1,Pg_max*.05); % assume that all plants can ramp at 5% per minute. 
                                            % for a 100 MW plant, this
                                            % would be 5 MW/min. Reasonable
    ramp_rate = ramp_rate_MW_per_min/60;
end
ramp_rate( ~ge_status ) = 0; % plants that are shut down cannot ramp
ramp_dt = opt.sim.fast_ramp_mins*60;
ramp_limits = ramp_rate * ramp_dt;
ps.gen(:,C.ge.ramp_rate_down) = ramp_limits;
% Error check
if any( Pg<Pg_min-EPS | Pg>Pg_max+EPS )
    error('Pg is out of bounds in the initial system.');
end
% Print the time
if verbose
    fprintf('---------- t = 0.00 ----------\n');
end

%% Step 1. Run power flow and make sure Pmin < Pg < Pmax
% Calculate the power flow with participation factors proportional to Pmax.
% The first power flow does not need a rebalance
[~,n_sub_old] = find_subgraphs(ps.bus(:,1),ps.branch(br_status,1:2));
if n_sub_old > 1
    error('The base case has more than one island.');
end
% set options for initial power flow
opt.pf.CascadeControl = false;
opt.pf.PartFactFlag = true;
% run power flow 
ps = acpf(ps,[],opt);
Pg0_sum = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));

if nargout > 4
    % record data
    record_data.time = 0;
    record_data.Imag_f = ps.branch(:,C.br.Imag_f);
    record_data.demand_lost = 0;
    record_data.gen_lost = 0;
    record_data.nbr_out = 0;
end
%% Step 2. Apply exogenous outages
t_sim = 1; % simulation time
if verbose
    fprintf('---------- t = %.3f ----------\n',t_sim);
    fprintf('Exogenous events:\n');
end
% Apply the branch outages
if ~isempty(br_outages_ex)
    ps.branch(br_outages_ex,C.br.status) = 0;
    if verbose
        fprintf(' Removed branch %d\n',br_outages_ex);
    end
end
    
% Apply the bus outages
if ~isempty(bus_outages)
    for i=1:length(bus_outages)
        bus_no = bus_outages(i);
        bus_ix = ps.bus_i(bus_no);
        if isempty(bus_ix) || bus_ix<=0 || bus_ix>nbus
            error('%d is not a valid bus number.\n',bus_no);
        end
        br_set = (F==bus_ix) | (T==bus_ix);
        ps.branch(br_set,C.br.status) = 0;
        % trip gens and shunts at this bus
        ps.gen  (G==bus_ix,C.ge.status) = 0;
        ps.shunt(D==bus_ix,C.sh.status) = 0;
        if verbose
            fprintf(' Removed bus %d.\n',bus_no);
        end
    end
end

%% Begin the main loop for ACSIMSEP
it_no = 1;
t_prev_control = -dt_max; % the time that a previous control action was done.
                          % Set negative so that the first control action will
                          % be done no matter its time. 
while true % loop will break when t_sim > t_max
    % Step 3. Find sub-grids in the network 
    [sep,sub_grids,n_sub] = check_separation(ps,stop_threshold,verbose);

    % Step 4. If there are new islands, rebalance the island
    if n_sub>n_sub_old
        ramp_dt = opt.sim.fast_ramp_mins*60; % do a minute of ramping before gen tripping/load shedding 
        ramp_limits = ramp_rate*ramp_dt;
        Pd_old = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
        ps = rebalance(ps,sub_grids,ramp_limits,opt,[],'ac');
        Pd_new = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
        MW_lost.rebalance = MW_lost.rebalance + Pd_old - Pd_new;
        ge_status = (ps.gen(:,C.ge.status) == 1);
        ramp_rate(~ge_status) = 0; % make sure that failed generators don't ramp
    end
    n_sub_old = n_sub;
    
    % Step 5. run power flow, and shed load to avoid voltage collapse
    % set up participation factors proportiante to max generation.
    % set cascade control true to shed load when power flow does not
    % converge
    opt.pf.CascadeControl = true;
    Pd_old = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
    ps = acpf(ps,[],opt);
    Pd_new = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
    Pg_new = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));
    MW_lost.voltage_collapse = MW_lost.voltage_collapse + Pd_old - Pd_new;
    if any(ps.gen(:,C.ge.P) < 0)
        error('Power flow resulted in some negative generations.')
    end
    % record some data if needed
    if nargout > 4
        % record data
        record_data.time = [record_data.time, t_sim];
        record_data.Imag_f = [record_data.Imag_f, ps.branch(:,C.br.Imag_f)];
        record_data.demand_lost = [record_data.demand_lost, Pd0_sum-Pd_new];
        record_data.gen_lost = [record_data.gen_lost, Pg0_sum-Pg_new];
        br_status_new = (ps.branch(:,C.br.status) == 1);
        nbr_new = sum(br_status_new);
        record_data.nbr_out = [record_data.nbr_out, nbr0-nbr_new];
    end
    
    % Step 5a. Take control actions if needed.
    % Make sure the previous control action was done at least dt_max seconds ago
    if ~strcmp(opt.sim.control_method, 'none') && ...
            t_sim - t_prev_control >= dt_max
        switch opt.sim.control_method
            % Compute and implement emergency control actions
            %  Note that this code also interfaces with a python comm. model,
            %  if requested in the options structure
            case 'emergency_control'
                [ps, this_MW_lost, mis] = central_control(ps,sub_grids,ramp_rate,opt,'ac');
            case 'distributed_control' % distributed emergency control 
                [ps_agents, ps, this_MW_lost, mis] = distributed_control(ps_agents,ps,sub_grids,ramp_rate,it_no,opt,'ac');
            otherwise
                error('Undefined control method.')
        end
        MW_lost.control = MW_lost.control + this_MW_lost.control;
        MW_lost.voltage_collapse = MW_lost.voltage_collapse + this_MW_lost.voltage_collapse;
        imbalance = imbalance + mis;
        % update previous control time
        t_prev_control = t_sim;
    end
    
    % If we want to stop when the network is divided into subnetworks, do
    % this:
    if opt.sim.stop_on_sep
        if sep==BIG_SEP
            is_blackout = true;
            if verbose
                fprintf('-------------- t = %.3f ----------------\n',t_sim);
                fprintf('----------- Major separation -----------\n');
            end
            break
        end
    else % If we wanted to stop after a certain amount of load shedding, do this:
        Pd_sum = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
        load_remaining_fraction = Pd_sum/Pd0_sum;
        if verbose
            %fprintf('------------- t = %.3f ---------------\n',t);
            fprintf('-------- %.1f%% of load remains --------\n',load_remaining_fraction*100);
        end
        if load_remaining_fraction < opt.sim.stop_threshold
            is_blackout = true;
            if verbose
                fprintf('----------- Blackout occurred ----------\n');
            end
            break
        end
    end
    
    % Step 6. update relays
    [ps.relay,br_out_new,dt,n_over] = update_relays(ps,verbose,dt_max);
    
    % Check for any remaining overload potential, decide if we should stop 
    % the simulation
    if n_over == 0 % previously: dt==Inf (changed because dt is now reduced
                   % to make sure control actions are implemented every minute)
        if verbose
            fprintf(' There are no overloads in the system. Quitting...\n');
        end
        break
    elseif n_over > 0 && verbose
        fprintf(' There are %d overloads in the system.\n',n_over);
    end

    % advance/print the time
    t_sim = t_sim + dt;
    if t_sim > opt.sim.t_max % stop if the next outage happens after t_max
        t_sim = opt.sim.t_max;
        break
    end
    if verbose
        fprintf('---------- t = %.3f ----------\n',t_sim);
    end
    
    % Step 7. Trip overloaded branches
    ps.branch(br_out_new,C.br.status) = 0;
    % record which branches were lost
    for i = 1:length(br_out_new)
        br = br_out_new(i);
        br_outages_en = cat(1,br_outages_en,[t_sim br]);
    end
    % print something
    if verbose && ~isempty(br_out_new)
        fprintf(' Branch %d triped on overcurrent.\n',br_out_new);
    end
    % Increment the counter and return to step 3.
    it_no = it_no + 1;
end

% Compute the amount of load lost
Pd = ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor);
Qd = ps.shunt(:,C.sh.Q).*ps.shunt(:,C.sh.factor);
MW_lost_total = Pd0_sum - sum(Pd);
if abs(MW_lost_total - MW_lost.rebalance - MW_lost.voltage_collapse ...
        - MW_lost.control) > EPS
    error('Something is wrong!')
end
MVAR_lost = Qd0_sum - sum(Qd);

if strcmp(opt.sim.control_method,'distributed_control')
    n_msg = zeros(nbus,1);
    % Compute the maximum number of messages per all iterations for each agent
    for i = 1:nbus
        n_msg(i) = max(ps_agents(i).messages);
    end
else
    n_msg = 0;
end

% record some data if needed
if nargout > 4
    % record data
    Pd_new = sum(ps.shunt(:,C.sh.P).*ps.shunt(:,C.sh.factor));
    Pg_new = sum(ps.gen(:,C.ge.P).*ps.gen(:,C.ge.status));    
    record_data.time = [record_data.time, t_sim];
    record_data.Imag_f = [record_data.Imag_f, ps.branch(:,C.br.Imag_f)];
    record_data.demand_lost = [record_data.demand_lost, Pd0_sum-Pd_new];
    record_data.gen_lost = [record_data.gen_lost, Pg0_sum-Pg_new];
    br_status_new = (ps.branch(:,C.br.status) == 1);
    nbr_new = sum(br_status_new);
    record_data.nbr_out = [record_data.nbr_out, nbr0-nbr_new];
end

% Print something
if verbose
    fprintf('----------------- t = %7.3f -----------------\n',t_sim);
    fprintf(' Simulation complete.\n');
    fprintf('  %d emergency (rateB) overloads remain.\n',n_over);
    fprintf('  %d endogenous relay outages\n',size(br_outages_en,1));
    fprintf('  %g MW load lost (%.1f%%)\n',MW_lost_total,MW_lost_total/Pd0_sum*100);
    fprintf('  %g MW (%.1f%%) in rebalance,  %g MW (%.1f%%) in control,\n', ...
        MW_lost.rebalance, MW_lost.rebalance/Pd0_sum*100, ...
        MW_lost.control, MW_lost.control/Pd0_sum*100);
    fprintf('  %g MW (%.1f%%) to avoid voltage collapse.\n',...
        MW_lost.voltage_collapse,MW_lost.voltage_collapse/Pd0_sum*100);     
    fprintf('  %g MVAr load lost (%.1f%%)\n',MVAR_lost,MVAR_lost/Qd0_sum*100); 
    if imbalance > 0
        fprintf('  %g MW (%.1f%%) imbalance occured due to control.',...
            imabalance, imbalance/Pd0_sum*100);
    end
    fprintf('--------------------------------------------\n');
end

end










