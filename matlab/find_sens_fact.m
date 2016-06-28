function [beta, beta_S, xm] = find_sens_fact(mismatch,x0,Sd_bus_sub,pq_sub,ref_sub, ...
                                            PartFactFlag,verbose)
    % find sensitivity of distance to shedding P, Q or S (with constant 
    % power factor) at each bus
    use_eigv = false; % choose between using the eigen value or the vector of minimum distance directly
    % note: Overbye suggests to use the left eigen vector of Jacobian at 
    % minimum distance in his paper (1995). But the vector of residuals at
    % minimum distance (the minimum distance vector itself) can also be
    % used, and seems to work better as it does not deal with numerical
    % precisions when finding the left eigen vector for a close to zero
    % eigen value. 
    nbus_sub = size(Sd_bus_sub,1); % number of buses in the subset
    % initialize the vector of sensitivities to shedding at each bus
    beta_P = zeros(nbus_sub,1); beta_Q = zeros(nbus_sub,1); beta_S = zeros(nbus_sub,1); 
    
    % find the minimum distance to solvability :                            
    % solve the unconstrained optimization problem g(x)'*g(x), in which
    % g(x) is the mismatch in power injections
    if verbose
        verbose_opt = 'iter';
        disp('Finding the minimum distance to solvability...')
    else
        verbose_opt = 'none';
    end
    opt_lsq = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt', ...
        'Display',verbose_opt,'Jacobian','on','MaxIter',100);
    % [x,resnorm,residual,exitflag,output,lambda,jacobian] = ...
    [xm,resnorm,mismatch_xm,exitflag,~,~,Jac] = lsqnonlin(mismatch,x0,[],[],opt_lsq);
    beta = sqrt(resnorm); % distance to solvability
    if verbose
        fprintf('Distance from solvable region = %.3f pu.\n',beta)
    end
    if use_eigv
        % get the left eigen vector at the optimum point
        [wm, error_flag] = get_left_eigv(Jac, verbose);
    else
        wm = mismatch_xm/norm(mismatch_xm);
        error_flag = false;
    end
    % resolve the direction of the eigen vector to the outside of the
    % solvable region (only useful when eigen vector is used)
    if use_eigv && sum(mismatch_xm.*wm) < 0 
        wm = -wm;
    end
    if verbose && error_flag
        disp('Something might be wrong with the eigen vector.');
    end
    if PartFactFlag
        % beta_P: the sensitivity factors when reducing real power at buses
        beta_P = wm(1:nbus_sub);
        % beta_Q: the sensitivity factors when reducing reactive power at buses
        beta_Q(pq_sub) = wm(nbus_sub+1:end);
    else
        beta_P(~ref_sub) = wm(1:nbus_sub-1);
        beta_Q(pq_sub) = wm(nbus_sub:end);
    end
    % beta_S: the sensitivity factors when reducing complex power at buses while maintaining power factor
    % (only considers buses with positive load)
    Pd_bus_sub = real(Sd_bus_sub);
    Qd_bus_sub = imag(Sd_bus_sub);
    % finds buses with at least one non-zero P or Q
    load_busi_sub = (Pd_bus_sub > 0) | (Qd_bus_sub > 0);
    pf = zeros(nbus_sub,1); % power factor at each bus
    pf(load_busi_sub) = Pd_bus_sub(load_busi_sub)./ ...
                                sqrt(Pd_bus_sub(load_busi_sub).^2 + Qd_bus_sub(load_busi_sub).^2);
    beta_S(load_busi_sub) = beta_P(load_busi_sub).*pf(load_busi_sub) + ...
                            beta_Q(load_busi_sub).*sqrt(1-pf(load_busi_sub).^2);
end

function [eig_vec, error_flag] = get_left_eigv(Jac, verbose)
    EPS = 0.08; % limit for (almost) zero eigen values
    error_flag = false;
    [eig_vec,lambda] = eigs(Jac',1,'SM'); % left eigen vector so Jac'
    if (abs(imag(lambda)) > EPS || abs(real(lambda)) > EPS) && verbose
        disp('Check this case. The smallest eigen value has a large real/imaginary value.')
        error_flag = true;
    end
    if (max(abs(imag(eig_vec))) > EPS) && verbose
        disp('Check this case. Eigen vector has imaginary values!')
        error_flag = true;
    end
    if verbose
        fprintf('The (almost) zero eigen value found: %.4f.\n',lambda);
    end
end

    

