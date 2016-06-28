function [x,converged,k] = nrsolve(eval_g,x0,opts)
% use the Newton-Raphson method to solve a set of non-linear equations
% usage: [x,converged,k] = nrsolve(g,x0,opts)
%
% Inputs:
%  eval_g is the g function, with two outputs (g and Jac_g).
%  x0 is a starting point
%  opts comes form numerics_options
% Outputs:
%  x is the ouptut optizer
%  converged is one of:
%   1  - found a solution that solves g(x)=0
%   0  - failed to find any solution.
%  k is the number of isterations required to solve the problem.

% process inputs
if nargin<2, error('at least 2 inputs needed'); end
if nargin<3, opts = numerics_options; end;
if nargout(eval_g)==1
    error('The input function needs to output the jacobian');
end

max_iters = opts.pf.max_iters;
tolerance = opts.pf.tolerance;
verbose   = opts.verbose;
use_fsolve = opts.pf.use_fsolve;

x = x0;
converged = false;
tic;

% solve with fsolve
if use_fsolve
    fsolve_opts = optimset( 'Jacobian','on',...
        'Algorithm','trust-region-dogleg',...
        'Display','off');
    [x,~,flag] = fsolve(eval_g,x0,fsolve_opts);
    success = (flag==1);
    converged = success; % = (flag==1);
    
else
    % print something
    if verbose
        disp('Iter   Max(|g|)   |g|_2      |J.''*g|      alpha ');
    end
    norm_g_old = inf;
    for k = 1:max_iters
        % evaluate the function and the derivative
        [g,J] = feval(eval_g,x);
        % do some calculations to check for convergence:
        max_mismatch = max(abs(g)); % inf norm
        % check first order optimality condition
        Jg = J.'*g;
        mean_Jg = mean(Jg);
        norm_g = norm(g,2);
        if norm_g > norm_g_old
            x = x_old;
            break
        end
        % check for convergence
        if max_mismatch < tolerance %checks if max is is greater than tolerance
            converged = true;
            if verbose
                et = toc;
                fprintf('Solution found in %g seconds.\n',et);
            end
            break;
        end
        % choose the search direction
        p = -(J\g);
        % save x for the plot below with different alpha's
        x_old = x;
        % do some sort of line search to select the step size and the new x
        [x,alpha] = linesearch(p,x,eval_g,opts);
        % print something
        if verbose
            fprintf('%4d %10.7f %10.7f %10.7f %10.7f \n', ...
                k, full(max_mismatch), full(norm_g), full(mean_Jg), alpha);
        end
        norm_g_old = norm(g,2);
        % produce a plot to help think about alpha
        if 0
            alpha_range = 0.8:0.001:0.9;
            q_norm = zeros(size(alpha_range));
            for ai = 1:length(alpha_range)
                x_new = x_old + alpha_range(ai)*p;
                g_new = feval(eval_g,x_new);
                g_norm(ai) = mean(g_new.^2);
            end
            figure(1); clf
            plot(alpha_range,g_norm);
            xlabel('Alpha');
            ylabel('g-norm');
            pause;
        end
        if alpha < 1e-12
            break
        end
    end
end

if  verbose && converged~=1
    disp(' Did not find a solution to g(x)=0.');
end

end

function [x,alpha] = linesearch(p,x_prev,eval_g,opts)
% perform a line search to choose a step size

% choose the method
switch opts.pf.linesearch
    case 'none'
        alpha = 1;
        x = x_prev + alpha*p;
        g = feval(eval_g,x);
    case 'backtrack'
        % get the starting point
        f_prev = norm(feval(eval_g,x_prev));
        alpha = 1;
        alpha_min = opts.pf.alpha_min;
        mu        = opts.pf.mu;
        while alpha > alpha_min
            x = x_prev + alpha*p;
            g = feval(eval_g,x);
            f = norm(g);
            % test the sufficient decrease (Armijo) condition
            if f <= f_prev + mu * alpha * sum( p.*x );
                break
            end
            alpha = alpha / 2;
            if 0
                a = -1:.01:1;
                for i = 1:length(a)
                    fi(i) = norm(feval(eval_g,x_prev + a(i)*p));
                end
                figure(1);
                plot(a,fi);
                pause
            end
        end
    case 'exact'
        f_mis = @(a)norm(feval(eval_g,x_prev + a*p),2);
        [alpha,~,flag] = fminunc(f_mis,1,optimset('Display','off','LargeScale','off'));
%         [alpha,~,flag] = fminbnd(f_mis,0,2);
        if flag==0
            alpha = 0;
        end
        x = x_prev + alpha*p;
    case 'cubic_spline'
        alpha = 1;
        alphaEnd  = 1.5;
        alpha_min = opts.pf.alpha_min;
        mu        = opts.pf.mu;
        while alpha > alpha_min
            try
                [alpha] = minstep(eval_g, x_prev, 0, alphaEnd, p);
            catch
                alpha = alphaEnd;
            end
            %check if alpha is reasonable. if alpha > alphaEnd, set alpha
            %to alpha end.
            if alpha > alphaEnd
                alpha = alphaEnd;
            end
            
            x = x_prev + alpha*p;
            g = feval(eval_g,x);
            f = norm(g);
            %keyboard
            % test the sufficient decrease (Armijo) condition
            if f <= f_prev + mu * alpha * sum( p.*x );
                break
            end
            alphaEnd = alphaEnd/2;
        end
    otherwise
        error('unsupported line search method.');
end

end

