function [g,dg_dx] = mismatch_polar(x,Ybus,Vmag,Sg_bus,Sd_bus,pq,pv,ref,PartFactFlag,PartFact,Si)

% constants
j = 1i;
nBus = size(Ybus,1);
if PartFactFlag && all(PartFact==0) 
    if abs(sum(Sd_bus)) < 1e-8
        PartFact(1) = 1;
    else
        error('No ramping generators provided');
    end
end
% convert pv/pq to logical arrays
if length(pv)<nBus
    pv_ = false(nBus,1);
    pv_(pv) = true;
    pv = pv_;
end
if length(pq)<nBus
    pq_ = false(nBus,1);
    pq_(pq) = true;
    pq = pq_;
end
if length(ref)<nBus
    ref_ = false(nBus,1);
    ref_(ref) = true;
    ref = ref_;
end
if (sum(ref)~=1), error('Must have only one ref bus'); end
% build an index
npq = sum(pq);
ix.theta = 1:(nBus-1);
ix.Vmag  = (1:npq) + (nBus-1);
if PartFactFlag
    % add in a variable for the generator ramping
    ix.rho = 1 + (npq + nBus-1);
    nx = nBus + npq;
else
    nx = nBus - 1 + npq;
end

% extract things from x
theta = zeros(nBus,1);
theta(~ref) = x(ix.theta);
Vmag(pq)    = x(ix.Vmag);
if PartFactFlag
    rho = x(ix.rho);
    ramp_gen = ref | pv;
    % make adjustments to Sg for gen ramping
    if isempty(PartFact)
        PartFact = zeros(nBus,1);
        PartFact(ramp_gen) = 1;
    end
end

% modify participating factors so that their mean is 1 (for scaling
% purposes to make sure jacobian is not ill-conditioned)
if PartFactFlag
    factor = nnz(PartFact)/sum(PartFact);
    PartFact = PartFact*factor;
end

% calculate the voltage
V = Vmag.*exp(j*theta);

% calculate the total load according to ZIPE matrix
zipe_cols       = size(Sd_bus,2);
if zipe_cols==1
    zipe_loads = false;
else
    zipe_loads = true;
end
if ~zipe_loads
    S_zipe = Sd_bus;
elseif zipe_cols == 5
    S_Z = Sd_bus(:,1) .* Vmag.^2;
    S_I = Sd_bus(:,2) .* Vmag;
    S_P = Sd_bus(:,3);
    S_E = Sd_bus(:,4) .* Vmag.^Sd_bus(:,5);
    S_zipe = S_Z + S_I + S_P + S_E;
else
    error('zipe load model matrix is not the right size');
end
if PartFactFlag
    Sg_bus = Sg_bus + PartFact*rho;
end

Sbus = Sg_bus - S_zipe;

% compute f(x) in f(x)-S=0
fx = (V .* conj(Ybus * V)); 
% compute S in f(x)-S=0
if nargin < 11
    % S is not provided. Build S:
    if PartFactFlag
        S = [real(Sbus); imag(Sbus(pq))];
    else
        S = [real(Sbus(~ref)); imag(Sbus(pq))];
    end
else
    S = Si;
    if PartFactFlag
        % add generation from participating generators
        S(1:nBus,1) = S(1:nBus,1) + PartFact*rho;
    end
end
% compute the mismatch g(x) = f(x)-S
if PartFactFlag
    g = [real(fx); imag(fx(pq))] - S;
else
    g = [real(fx(~ref)); imag(fx(pq))] - S;
end

% Jacobian
if nargout>1
    n = nBus;
    % do some matrix algebra (borrowed from MATPOWER)
    Ibus = Ybus * V;
    diagV     = spdiags(V, 0, n, n);
    diagIbus  = spdiags(Ibus, 0, n, n);
    diagVnorm = spdiags(exp(j*theta), 0, n, n);
    dSbus_dVm = diagV * conj(Ybus * diagVnorm) + conj(diagIbus) * diagVnorm;
    dSbus_dVa = j * diagV * conj(diagIbus - Ybus * diagV);
    if PartFactFlag
        % now put these into dg_dx
        dg_dx = sparse(nx,nx);
        dP_rows = (1:n);
        %dQ_rows = (1:npq) + n;
        % dP_dtheta
        [rows,cols,values] = find(real(dSbus_dVa(:,~ref)));
        dg_dx = dg_dx + sparse(rows,cols,values,nx,nx);
        % dQ_dtheta
        [rows,cols,values] = find(imag(dSbus_dVa(pq,~ref)));
        dg_dx = dg_dx + sparse(rows+nBus,cols,values,nx,nx);
        % dP_dVmag
        [rows,cols,values] = find(real(dSbus_dVm(:,pq)));
        dg_dx = dg_dx + sparse(rows,cols+(nBus-1),values,nx,nx);
        % dQ_dVmag
        [rows,cols,values] = find(imag(dSbus_dVm(pq,pq)));
        dg_dx = dg_dx + sparse(rows+nBus,cols+(nBus-1),values,nx,nx);
        % dP_drho
        dg_dx = dg_dx + sparse(dP_rows(ramp_gen),ix.rho,-PartFact(ramp_gen),nx,nx);
    else
        % no participation factor (standard power flow)
        dg_dx = sparse(nx,nx);
        % dP_dtheta
        [rows,cols,values] = find(real(dSbus_dVa(~ref,~ref)));
        dg_dx = dg_dx + sparse(rows,cols,values,nx,nx);
        % dQ_dtheta
        [rows,cols,values] = find(imag(dSbus_dVa(pq,~ref)));
        dg_dx = dg_dx + sparse(rows+(nBus-1),cols,values,nx,nx);
        % dP_dVmag
        [rows,cols,values] = find(real(dSbus_dVm(~ref,pq)));
        dg_dx = dg_dx + sparse(rows,cols+(nBus-1),values,nx,nx);
        % dQ_dVmag
        [rows,cols,values] = find(imag(dSbus_dVm(pq,pq)));
        dg_dx = dg_dx + sparse(rows+(nBus-1),cols+(nBus-1),values,nx,nx);
    end
    if zipe_loads
        % fix the derivatives with ZIP[E] contributions
        dP_E_dVmag = Sd_bus(:,5).*real(Sd_bus(:,4)).*Vmag.^(Sd_bus(:,5)-1);
        dQ_E_dVmag = Sd_bus(:,5).*imag(Sd_bus(:,4)).*Vmag.^(Sd_bus(:,5)-1);
        % fix the derivatives with [Z]IPE contributions
        dP_Z_dVmag = 2.*real(Sd_bus(:,1)).*Vmag;
        dQ_Z_dVmag = 2.*imag(Sd_bus(:,1)).*Vmag;
        % fix the derivatives with Z[I]PE contributions
        dP_I_dVmag = real(Sd_bus(:,2));
        dQ_I_dVmag = imag(Sd_bus(:,2));
        
        SS_zipe = S_zipe;
        SS_zipe(ref | pv)= 0;     % assume that exponential loads are not located in ref/pv buses
        
        rows = find(SS_zipe); cols = find(SS_zipe(pq));
        dg_dx = dg_dx + sparse(rows,cols+(nBus-1),dP_E_dVmag(rows),nx,nx);
        dg_dx = dg_dx + sparse(rows,cols+(nBus-1),dP_Z_dVmag(rows),nx,nx);
        dg_dx = dg_dx + sparse(rows,cols+(nBus-1),dP_I_dVmag(rows),nx,nx);
        
        rows = find(SS_zipe(pq)); cols = rows;
        dQ_E_dVmag = dQ_E_dVmag(pq);
        dQ_Z_dVmag = dQ_Z_dVmag(pq);
        dQ_I_dVmag = dQ_I_dVmag(pq);
        dg_dx = dg_dx + sparse(rows+nBus,cols+(nBus-1),dQ_E_dVmag(rows),nx,nx);
        dg_dx = dg_dx + sparse(rows+nBus,cols+(nBus-1),dQ_Z_dVmag(rows),nx,nx);
        dg_dx = dg_dx + sparse(rows+nBus,cols+(nBus-1),dQ_I_dVmag(rows),nx,nx);
    end
end

