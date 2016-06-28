function opt = psoptions(opt)
% some options for the power system simulator files

if nargin==0
    opt = struct;
end

C = psconstants;

%% General options
opt.verbose = 1;
%opt.seecascade = 0;
opt.debug = true;
opt.optimizer = 'cplex'; % or cplex, mexosi

%% power flow options
opt.pf.tolerance = 1e-8; % convergence tolerance
opt.pf.max_iters = 20; % max power flow iterations
opt.pf.CalcIslands = 1; % iteratively calculates each island in runPowerFlow
opt.pf.flat_start = 0;
opt.pf.load_shed_rate = 0.25; % the rate at which under frequency load shedding is done in CascadingPowerFlow mode
opt.pf.linesearch = 'exact';
opt.pf.update = true;
opt.pf.PolarFlag = true; % choose between polar and rectangular formulation (for voltage)
opt.pf.CascadeControl = false;
opt.pf.PartFactFlag = true; % participation factors for generators
opt.pf.use_fsolve = false;
% opt.pf.factor = 0.1; % new limits for Pg after acpf: (Pmin - factor*Pgmax) <= Pg <= (Pmax + factor*Pgmax)
opt.pf.loss_factor = 0.05; % loss factor for AC power flow: Pg = (1+loss_factor)*Pd
opt.pf.check_Pg = true;

%% optimal power flow options
opt.opf.generator_commitment = 0; % switch generators on/off using MIP
opt.opf.branch_switching = 0;     % switch branches on/off using MIP
opt.opf.rate = C.br.rateA;
opt.opf.contg_rate = C.br.rateB;

%% time-domain simulation options
opt.sim.t_max = 15*60; % simulation time in seconds
opt.sim.dt = 60; % simulation time step
opt.sim.ramp_frac = 0.05; % fraction of generator allowed to ramp between generations
opt.sim.writelog  = false;
opt.sim.draw = true;
opt.sim.overload_time_limit = 10*60; % number of seconds that the branch can sit at its rateC level (PSS/E manual)
opt.sim.stop_on_sep = false; % When true, the simulator declares a blackout
                             % based on the Giant Component size. Otherwise
                             % this depends on the blackout size in MW
opt.sim.stop_threshold = 0; % the fraction of nodes in the giant component, or load still connected, at which to stop the simulation
opt.sim.fast_ramp_mins = 1; % the minimum minutes of ramping that generators are allowed to do without load shedding
% opt.sim.use_control = 0; -----> use opt.sim.control_method instead
opt.sim.control_method = 'none';
opt.sim.use_comm_model = false;
opt.sim.simple_rebalance = false; % Simple method used by Zussman's model
opt.sim.relay_trip_time = 15; % time to trip an overcurrent relay with a 50% overload (seconds)

% legacy
opt.simdc = opt.sim;
% emergency control
opt.sim.cost.load = 1;
opt.sim.cost.overload = 1000;
% distributed control options
opt.sim.nHopLoc = 2;
% opt.sim.nHopExt = 10;
% MPC options
opt.sim.use_mpc = false; % implement MPC or do one-step optimization
opt.sim.Np = 3;
opt.sim.debug = false;

%% drawing stuff
opt.draw.width = 0.1;
opt.draw.bus_nos = true;
opt.draw.simple = false;
opt.draw.fontsize = 14;


