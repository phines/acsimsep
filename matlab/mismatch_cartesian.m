function [g,dg_dx] = mismatch_cartesian(x,Ybus,Vmag,Sg,Sd,pq,pv,ref,PartFact)
% usage: [g,dg_dx] = mismatch_cartesian(x,Ybus,Vmag,Sg,Sd,pq,pv,ref,PartFact)
%
% a power flow mismatch function, with participation factors
% some of the buses are assumed to be generators that do load-following. 
% The bus at "ref" will be assiged a zero angle.
% If PartFact (participation factors) is defined it should be
% an n x 1 vector indicating the extent to which the generators particpate
% in load following

% constants
j = 1i;
 
nBus = size(Ybus,1);
if nargin<9
    PartFact = zeros(nBus,1);
    PartFact(ref) = 1;
end
if all(PartFact==0)
    error('No ramping generators provided');
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
npv = sum(pv);
ix.e = 1:(nBus-1);
ix.f  = (1:(nBus-1)) + max(ix.e); % we only need to keep the imaginary part of voltage for non-pv buses.
% add in a variable for the generator ramping
ix.rho = 1 + max(ix.f);
% degenerate case:
% if isempty(ix.rho) % degenerate case
%     ix.rho = 1 + max(ix.theta);
% end
nx = 2*nBus - 1;

% extract things from x
e = Vmag;
e(~ref) = x(ix.e);
f = zeros(nBus,1);
f(~ref) = x(ix.f);
rho   = x(ix.rho);

% calculate the voltage
V = e + j*f;
% make adjustments to Sg for gen ramping
ramp_gen        = pv | ref;
% calculate the total load according to ZIPE matrix
zipe_cols       = size(Sd,2);
if zipe_cols==1
    zipe_loads = false;
else
    zipe_loads = true;
end
if ~zipe_loads
    S_zipe = Sd;
elseif zipe_cols == 5
    S_Z = Sd(:,1) .* Vmag.^2;
    S_I = Sd(:,2) .* Vmag;
    S_P = Sd(:,3);
    S_E = Sd(:,4) .* Vmag.^Sd(:,5);
    S_zipe = S_Z + S_I + S_P + S_E;
else
    error('zipe load model matrix is not the right size');
end
Sbus            = Sg - S_zipe;
Sbus(ramp_gen)  = Sbus(ramp_gen) + PartFact(ramp_gen)*rho;
% compute the final mismatch
miscx = (V .* conj(Ybus * V)) - (Sbus);
misv = e(pv).^2 + f(pv).^2 - Vmag(pv).^2;
g = [real(miscx); imag(miscx(pq)); misv];

% Jacobian
if nargout>1
    G = real(Ybus); B = imag(Ybus);
    n = nBus;
    % J11
    temp = nan(n, n);
    for i = 1:n
        for j = 1:n
            temp(i,j) = Jac(i,j,G,-B,G,B,e,f);
        end
    end
    J11 = temp(:,~ref);
    % J12
    temp = nan(n, n);
    for i = 1:n
        for j = 1:n
            temp(i,j) = Jac(i,j,B,G,-B,G,e,f);
        end
    end
    J12 = temp(:,~ref);
    % J21
    temp = nan(n, n);
    for i = 1:n
        for j = 1:n
            temp(i,j) = Jac(i,j,-B,-G,-B,G,e,f);
        end
    end
    J21 = temp(pq,~ref);
    % J22
    temp = nan(n, n);
    for i = 1:n
        for j = 1:n
            temp(i,j) = Jac(i,j,G,-B,-G,-B,e,f);
        end
    end
    J22 = temp(pq,~ref);
    % J31
    all_bus = 1:n;
    bus_noref = all_bus(~ref);
    col_num = find(ismember(bus_noref,find(pv)));
    J31 = sparse(1:npv, col_num, 2*e(pv), npv, n-1);
    % J32
    J32 = sparse(1:npv, col_num, 2*f(pv), npv, n-1);
    
    dg_dx = [   J11 J12;
                J21 J22;
                J31 J32;     ];
    
    % dP_drho
    dg_dx = [dg_dx, sparse(2*n-1, 1)];
    dP_rows = (1:n);
    dg_dx = dg_dx + sparse(dP_rows(ramp_gen),ix.rho,-PartFact(ramp_gen),nx,nx);
%     det(dg_dx(2:end,1:end-1))
end

function Df = Jac(i,j,x,y,z,w,e,f)
if i == j
    Df = sum(x(i,:).*e' + y(i,:).*f') + z(i,i)*e(i) + w(i,i)*f(i);
else
    Df = z(i,j)*e(i) + w(i,j)*f(i);
end
return








