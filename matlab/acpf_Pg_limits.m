function ps_out = acpf_Pg_limits(ps,PartFact,opt,EPS)
% run acpf with participating factors only on buses that have enough
% capacity (such that Pg_min <= Pg <= Pg_max is not violated).
C = psconstants;
nbus = size(ps.bus,1);
G = ps.bus_i(ps.gen(:,1));   % gen index
ge_status = (ps.gen(:,C.ge.status) == 1);
pf_status = ge_status; % participation factor status (with generator indices)
Pg_max = ps.gen(:,C.ge.Pmax).*ge_status;
Pg_min = ps.gen(:,C.ge.Pmin).*ge_status;
while true
    ps_out = acpf(ps,PartFact,opt);
    
    %%%%%% this needs some work in the future, not working now! %%%%%%
    break
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Pg = ps_out.gen(:,C.ge.P);
    if all(Pg > Pg_min-EPS & Pg < Pg_max+EPS)
        % all Pg's are in their limits, so break
        break
    else
        if opt.verbose
            fprintf('Some generations are outside their limits. Fixing in acpf.\n');
        end
        % if total generation in ps_out is more than ps, part facts are 
        % positive, so zero part fact for the buses on Pmax. O.W. do the
        % same for Pmin. 
        if sum(ps_out.gen(:,C.ge.P).*ge_status) > sum(ps.gen(:,C.ge.P).*ge_status)
            i_max = Pg > Pg_max-EPS;
            ps_out.gen(i_max,C.ge.P) = Pg_max(i_max); 
            pf_status(i_max) = false;
        else
            i_min = Pg < Pg_min+EPS;
            ps_out.gen(i_min,C.ge.P) = Pg_min(i_min); 
            pf_status(i_min) = false;
        end
        gen_buses = sparse(G,1,double(pf_status),nbus,1); % find buses with available generators
        PartFact(~gen_buses) = 0; % they don't participate for losses either
    end
end