
% load the data
ps = case3_ps;
ps = updateps(ps);

% run the power flow
ps = acpf(ps);
% print the data
printps(ps);

% increase the load
disp('Increasing the load');
ps.shunt(:,C.sh.P:C.sh.Q) = ps.shunt(:,C.sh.P:C.sh.Q)*2;
% run the power flow
ps = acpf(ps);
% print the data
printps(ps);
