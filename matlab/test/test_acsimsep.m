%% add the two acsimsep paths
addpath ../../data
addpath ../

%%
fprintf('\n\n');
clear variables;
opt = psoptions;
C = psconstants;
% load case
loadprc = 100;

% Polish
ps_filename = 'ps_polish_all.mat'; 
casename = sprintf('ps_polish_%d',loadprc);

% RTS-96
% ps_filename = 'ps_RTS_all_modified_gencost.mat';
% casename = sprintf('ps_%d',loadprc);

ps_struct = load(ps_filename,casename); 
ps = ps_struct.(casename);
ps = updateps(ps);

opt.verbose = true;
% randseed_Pooya;
% nbr = size(ps.branch,1);
% br_outages_ex = randi(nbr,1,3);

% choose from N-2's
outage_number = 1;
load ../../data/BOpairs2;
br_outages_ex = BOpairs(outage_number,:);

bus_outages = [];
tStart = tic;
% opt.sim.use_mpc = true;
% opt.sim.Np = 3;
% opt.sim.nHopLoc = 4;
opt.sim.control_method = 'none';
% opt.sim.control_method = 'emergency_control';
% opt.sim.control_method = 'distributed_control';
opt.pf.check_Pg = true; % make sure Pg is in its limits if true
opt.optimizer = 'cplex';
[is_blackout,br_outages_en,MW_lost,n_msg,record_data] = acsimsep(ps,br_outages_ex,bus_outages,opt);
toc(tStart);



