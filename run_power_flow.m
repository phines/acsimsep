%% Load the data
C = psconstants;
addpath('./data');
load_factor = 1;
ps = case2_ps;
% ps = case6ww_ps;
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
pv  = false(n,1);
part_fact = ps.bus(:,C.bu.type)==C.PV;

%% build the decision vector
npq = sum(pq);
ix = struct;
ix.theta = 1:(n-1);
ix.Vmag  = (1:npq) + max(ix.theta);
ix.rho = 1 + max(ix.Vmag); % generator ramping variable
if isempty(ix.rho) % degenerate case
    ix.rho = 1 + max(ix.theta);
end
nx = max(ix.rho);
x = zeros(nx,1);
x(ix.theta) = theta(~ref);
x(ix.Vmag)  = Vmag(pq);
x(ix.rho)   = 0;
if any(x(ix.Vmag))==0
    x(ix.Vmag) = 1;
end
%% Solve
opts = numerics_options;
opts.nr.verbose=true;
% set up a virtual function to solve
g = @(newx)mismatch(newx,Ybus,Vmag,Sg_bus,Sd_bus,pq,pv,ref,part_fact);
% try to solve the power flow problem
[x,flag] = nrsolve(g,x,opts);

%% Check the results
success = (flag==1);
% save the results
Vmag(pq) = x(ix.Vmag);
theta(~ref) = x(ix.theta);
theta(ref) = 0;

