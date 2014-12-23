function [Sbus, Sd_model, Sg] = getSbus(ps,only_PQ_loads)
% extract the Sbus injection from a power system structure
% usage: [Sbus, Sbus_load, Sbus_gen] = getSbus(ps,use_voltages)
% Sbus_load will be a matrix with the components from the ZIPE model, as defined in shunt

C = psconstants;
j = sqrt(-1);
n = size(ps.bus,1);

%% generators
status = ps.gen(:,C.ge.status);
Sg = status.*(ps.gen(:,C.ge.P) + j*ps.gen(:,C.ge.Q))/ps.baseMVA;
ge_bus_i = ps.bus_i(ps.gen(:,1));
Sg_bus = sparse(ge_bus_i,1,Sg,n,1);

%% loads
sh_bus_i = ps.bus_i(ps.shunt(:,1));
% figure out the ZIPE load types
f_Z = ps.shunt(:,C.sh.frac_Z);
f_P = ps.shunt(:,C.sh.frac_S);
f_E = ps.shunt(:,C.sh.frac_E);
f_I = 1 - f_Z - f_P - f_E;
% get the actual load and locations
Sd = ps.shunt(:,C.sh.factor).*(ps.shunt(:,C.sh.P) + j*ps.shunt(:,C.sh.Q))/ps.baseMVA;
% check the ZIPE load model
if any(f_I<0) || any(f_I>1)
    error('The load model in ps.shunt is not valid');
end
% get the exponent for the E portion
gamma = ps.shunt(:,C.sh.gamma);
% build the load matrix
Sd_model = [Sd.*f_Z Sd.*f_I Sd.*f_P Sd.*f_E gamma];

% build the final Sd_bus
if nargin>1 && only_PQ_loads
    Sd_bus = sparse(sh_bus_i,1,Sd.*f_P,n,1);
else
    Sd_bus = sparse(sh_bus_i,1,Sd,n,1);
end

%% combine to get Sbus
Sbus = Sg_bus - Sd_bus;

