function run_power_flow

%% Load the data
C = psconstants;
verbose = false;
polar_flag = true;
addpath('./data');
load_factor = 5;
% ps = case2_ps;
ps = case6ww_ps;
% ps = case2383wp_ps;
ps.shunt(:,C.sh.P) = ps.shunt(:,C.sh.P) * load_factor;
ps.shunt(:,C.sh.Q) = ps.shunt(:,C.sh.Q) * load_factor;

ps = updateps(ps);
% Extract stuff from ps
n = size(ps.bus,1);
G = ps.bus_i(ps.gen(:,1));
D = ps.bus_i(ps.shunt(:,1));
Ybus = getYbus(ps);
Vmag = ps.bus(:,C.bu.Vmag);
theta = zeros(n,1);
[Sbus, Sd, Sg] = getSbus(ps);
% Convert loads to simple constant power
Sd = sum(Sd,2);
% Create bus versions of Sd,Sg
Sd_bus = full(sparse(D,1,Sd,n,1));
Sg_bus = full(sparse(G,1,Sg,n,1));
% Find the bus types
pq  = ps.bus(:,C.bu.type)==C.PQ;
ref = ps.bus(:,C.bu.type)==C.REF;
pv  = ps.bus(:,C.bu.type)==C.PV;
part_fact = (pv | ref);

%% build the decision vector
npq = sum(pq);
ix = struct;
if polar_flag
    ix.theta = 1:(n-1);
    ix.Vmag  = (1:npq) + max(ix.theta);
    ix.rho = 1 + max(ix.Vmag); % generator ramping variable
%     if isempty(ix.rho) % degenerate case
%         ix.rho = 1 + max(ix.theta);
%     end
    nx = max(ix.rho);
    x = zeros(nx,1);
    x(ix.theta) = theta(~ref);
    x(ix.Vmag)  = Vmag(pq);
    x(ix.rho)   = 0;
    if any(x(ix.Vmag))==0
        x(ix.Vmag) = 1;
    end
    % set up a virtual function to solve
    g = @(newx)mismatch(newx,Ybus,Vmag,Sg_bus,Sd_bus,pq,pv,ref,part_fact);
else
    % cartesian coordinate power flow
    ix.e = 1:(n-1);
    ix.f  = (1:(n-1)) + max(ix.e);
    ix.rho = 1 + max(ix.f); % generator ramping variable
%     if isempty(ix.rho) % degenerate case
%         ix.rho = 1 + max(ix.e);
%     end
    nx = max(ix.rho);
    x = zeros(nx,1);
    x(ix.e) = ones(1,n-1);
    x(ix.f)  = zeros(1,n-1);
    x(ix.rho)   = 0;
    % set up a virtual function to solve
    g = @(newx)mismatch_cartesian(newx,Ybus,Vmag,Sg_bus,Sd_bus,pq,pv,ref,part_fact);
end   
    
    
%% Solve
opts = numerics_options;
opts.nr.verbose = true;
% try to solve the power flow problem
[x,flag] = nrsolve(g,x,opts);

%% Check the results
success = (flag==1);
if polar_flag
    % save the results
    Vmag(pq) = x(ix.Vmag);
    theta(~ref) = x(ix.theta);
    theta(ref) = 0;
else
    e = Vmag; 
    f = zeros(n,1);
    e(~ref) = x(ix.e);
    f(~ref) = x(ix.f);
    Vmag = sqrt(e.^2 + f.^2);
    theta = atan(f./e);
end
V = Vmag.*exp(1i*theta);
if verbose
    fprintf('%.4f /_ %.2f\n',[Vmag';theta'*180/pi]);
end
