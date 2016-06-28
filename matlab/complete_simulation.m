clear variables 

% plot inputs
n_cases = 6;
% folder_name = 'final1000_3';
folder_name = 'final1000_5';
simulator = 'acsimsep';

% initialize stuff
cnt = 0;
MW_lost = cell(n_cases,1);
if strcmp(simulator,'dcsimsep')
    new_data = zeros(1000,4);
elseif strcmp(simulator,'acsimsep')
    new_data = zeros(1000,5);
end
new_msg = zeros(1000,2383);

% find total load in the Polish system
load('ps_polish_all','ps_polish_100')
ps = ps_polish_100;
ps = updateps(ps);
C = psconstants;

% load the complete set of branch outages
load vacc_sep10/test_data
br_list = n2_list_overload(rand_id,:);
new_data(:,1:2) = br_list;

%% compute blackout sizes
for M = [1, 3, 2]
    switch M
        case 1
            Method = 'none';
            all_N = 0;
            all_MPC = 0;
            all_Np = 0;
        case 2
            Method = 'emergency_control';
            all_N = 0;
            all_MPC = 1;
            all_Np = 3;
        case 3
            Method = 'distributed_control';
            all_N = 1:4;
            all_MPC = 1;
            all_Np = 3;
    end
    for N = all_N
        for MPC = all_MPC
            for np = all_Np
                % blackout size
                filename = sprintf('output/%s/%s/MWlost_%s_nhop_%d_usempc_%d_np_%d.csv',...
                    simulator, folder_name, Method, N, MPC, np);
                data = csvread(filename,1);
                if strcmp(simulator,'dcsimsep') && size(data,2)==5
                    data(:,end) = []; % there was an extra comma at some point that created an extra column
                end
                n = size(data,1);
                fprintf('%s : %d\n',filename,n);
                % load messages
                filename_msg = sprintf('output/%s/%s/msg_%s_nhop_%d_usempc_%d_np_%d.csv',...
                    simulator, folder_name, Method, N, MPC, np);
                msg = csvread(filename_msg);
                msg(:,end) = [];
                for i = 1:1000
                    idx = find((data(:,1)==br_list(i,1) & data(:,2)==br_list(i,2)), 1);
                    if ~isempty(idx)
                        new_data(i,:) = data(idx,:);
                        new_msg(i,:) = msg(idx,:);
                    else
                        %% run simulation
                        % define simulation options
                        opt = psoptions;
                        opt.verbose = false;
                        % set simulation options
                        opt.sim.control_method = Method;
                        opt.sim.nHopLoc = N;
                        opt.sim.use_mpc = MPC;
                        opt.sim.Np = np;
                        br_outages = br_list(i,1:2);
                        bus_outages = [];
                        fprintf('  running case %d of 1000: outage of lines %d and %d\n', ...
                            i,br_outages);
                        if strcmp(simulator,'dcsimsep')
                            [~,~,MW_lost,~,~,~,n_msg] = dcsimsep(ps,br_outages,bus_outages,opt);
                        elseif strcmp(simulator,'acsimsep')
                            [~,~,MW_lost,n_msg] = acsimsep(ps,br_outages,bus_outages,opt);
                            new_data(i,5) = MW_lost.voltage_collapse;
                        end
                        new_data(i,3) = MW_lost.rebalance;
                        new_data(i,4) = MW_lost.control;
                        new_msg(i,:) = n_msg;
                    end
                end
                fprintf(' writing the new files...\n');
                new_folder_name = [folder_name,'_complete'];
                folder_name_complete = sprintf('output/%s/%s', simulator, new_folder_name);
                if ~exist(folder_name_complete,'dir')
                    mkdir(folder_name_complete);
                end
                % first blackout sizes
                new_filename = sprintf('output/%s/%s/MWlost_%s_nhop_%d_usempc_%d_np_%d.csv',...
                    simulator, new_folder_name, Method, N, MPC, np);
                csvwrite(new_filename, new_data);
                % now messages
                new_filename_msg = sprintf('output/%s/%s/msg_%s_nhop_%d_usempc_%d_np_%d.csv',...
                    simulator, new_folder_name, Method, N, MPC, np);
                csvwrite(new_filename_msg, new_msg);
            end
        end
    end
end