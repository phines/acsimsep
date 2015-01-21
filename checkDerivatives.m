function [n_errs] = checkDerivatives(get_g,dg_dx,x)
% This function checks the derivatives for function g with derivatives
% dg_dx
% usage: checkDerivatives(get_g,get_dg_dx,x)

eps1 = 1e-5;
eps2 = 1e-4;
n_errs = 0;
worst_err = 0;

if isempty(dg_dx)
    [g_x, dg_dx] = get_g(x);
else
    g_x = get_g(x);
end

[n_rows,n_cols] = size(dg_dx);
disp('-----------------------------------------------');
disp('  Row   Col  Analytical  Numerical  Difference');
for j = 1:n_cols
    x1 = x;
    x2 = x;
    x1(j) = x1(j) + eps1;
    x2(j) = x2(j) - eps1;
    dg_dx_j_num = (get_g(x1) - get_g(x2)) / (2*eps1);
    dg_dx_j_ana = dg_dx(:,j);
    err = abs(dg_dx_j_num - dg_dx_j_ana);
    nz = err>10^-20;
    err_mag = zeros(n_rows,1);
    err_mag(nz) = err(nz) ./ max(abs(dg_dx_j_num(nz)),abs(dg_dx_j_ana(nz)));
    problem_i = find( err_mag > eps2 );
    worst_err = max(worst_err,max(abs(err_mag)));
    for i=problem_i'
        fprintf('%5d %5d %10g %10g %10g\n',i,j,full(dg_dx_j_ana(i)),dg_dx_j_num(i),err(i));
        n_errs = n_errs + 1;
    end
end
disp('-----------------------------------------------');
fprintf('Found %d suspicious derivatives\n',n_errs);
fprintf('Worst error magnitude: %e\n',worst_err);
disp('-----------------------------------------------');

