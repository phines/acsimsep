function ps = shed_load(ps,busi_sub,beta,beta_S,verbose)
    % shed load with constant power factor
    C = psconstants;
    dist = max(beta*1.5,0.05); % The distance that we want to move back to 
    % solvability: 1.5 times the original distance 
    beta_S_temp = beta_S; % make a copy to modify
    EPS = 1e-6;
    while dist > EPS
        % shed load to make dist zero
        [this_beta_S, idx] = max(real(beta_S_temp));
        if this_beta_S <= 0 
            % there is no more effective load shedding
            if verbose
                disp(' No more load shedding can help. Quitting...')
            end
            break
        end
        % find the load on that bus
        this_busi = busi_sub(idx);
        this_busID = ps.bus(this_busi,1);
        i_sh = find(ps.shunt(:,C.sh.bus) == this_busID); % rows in ps.shunt
        P_pu = sum(ps.shunt(i_sh,C.sh.P))/ps.baseMVA; 
        Q_pu = sum(ps.shunt(i_sh,C.sh.Q))/ps.baseMVA;
        S_pu = sqrt(P_pu^2+Q_pu^2);
        P_pu_now = sum(ps.shunt(i_sh,C.sh.P).*ps.shunt(i_sh,C.sh.factor))/ps.baseMVA; 
        Q_pu_now = sum(ps.shunt(i_sh,C.sh.Q).*ps.shunt(i_sh,C.sh.factor))/ps.baseMVA;
        S_pu_now = sqrt(P_pu_now^2+Q_pu_now^2);
        if S_pu_now < EPS % if there is no load remaining on the bus
            beta_S_temp(idx) = 0;
            continue
        end
        S_shed_pu = min(dist/this_beta_S,S_pu_now); % find the maximum load that can be shed
        if verbose
            S_MVA = S_pu_now*ps.baseMVA;
            S_shed_MVA = S_shed_pu*ps.baseMVA;
            fprintf('Distance to go in parameter space: %.2f pu. Shedding load on bus %d: %.2f MVA from %.2f MVA.\n', ...
                dist,this_busID,S_shed_MVA,S_MVA);
        end
        new_sh_factor = ps.shunt(i_sh,C.sh.factor) - S_shed_pu/S_pu;
        ps.shunt(i_sh,C.sh.factor) = max(new_sh_factor, 0);
        dist = dist - this_beta_S*S_shed_pu;
        beta_S_temp(idx) = 0;
    end
    if verbose
        fprintf('Remaining distance: %.2g pu.\n',dist);
    end
end
