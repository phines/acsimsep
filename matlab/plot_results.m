
clear variables

% plot inputs
n_cases = 6;
% folder_name = 'final1000_3';
folder_name = 'final1000_5_complete';
simulator = 'acsimsep';

% initialize stuff
cnt = 0;
MW_lost = cell(n_cases,1);
Risk = cell(n_cases,1);
outage_prob = cell(n_cases,1);
MW_lost_mean = nan(n_cases,1);
MW_lost_all = zeros(1000,n_cases); % saves all MW_lost, not just unique ones

% find total load in the Polish system
load('ps_polish_all','ps_polish_100')
ps = ps_polish_100;
C = psconstants;
total_load = sum(ps.shunt(:,C.sh.P) .* ps.shunt(:,C.sh.factor));
Pr = ps.branch(:,C.br.fail_rate)/8760; % line outage probabilities

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
                try
                    data = csvread(filename);
                catch 
                    data = csvread(filename,1);
                end
                n = size(data,1);
                fprintf('%s : %d\n',filename,n);
                cnt = cnt+1;
                % find unique n-2's
                br_set = data(:,1:2);
                [br_set_unique, ia] = unique(br_set,'rows');
                data_MW = data(ia,3:end);
                new_col = sum(data_MW,2);
                MW_lost{cnt} = new_col;
                if size(data,1) == 1000
                    MW_lost_all(:,cnt) = sum(data(:,3:end),2);
                end
                outage_prob{cnt} = prod(Pr(br_set_unique),2);
                Risk{cnt} = MW_lost{cnt} .* outage_prob{cnt} * 1000; % risk in kW
                MW_lost_mean(cnt) = mean(new_col);
            end
        end
    end
end

%% compute bar results
range = [0,10,20,30,40,100];
% make y_bar
y_bar = nan(n_cases,length(range)-1);
for k = 1:n_cases
    this_MW_lost = MW_lost{k};
    this_risk = Risk{k};
    for i = 1:length(range)-1
        min_MW = range(i)*total_load/100;
        max_MW = range(i+1)*total_load/100;
        % compute risk for the before case
        subset = this_MW_lost>=min_MW & this_MW_lost<max_MW;
        y_bar(k,i) = sum(this_risk(subset));
    end
end
y_bar = y_bar*7.784; % to estimate total risk. These are the results for 1000 out of 7784 cases 
%% plot bars
figure(440); clf;
% colormap('summer')
cmap = [linspace(0,1,64)', zeros(64,1), linspace(1,0,64)'];
font_size = 16;
hbar = bar(y_bar,0.8,'stacked');
set(hbar,'EdgeColor','none');
xlim([0.5,6.5])
% ylim([0 40])
set(gca,'xticklabel',{'None','N_{hop}= 1','N_{hop}= 2','N_{hop}= 3','N_{hop}= 4','Centralized'},'FontSize',font_size)
ylabel('N-2 risk (expected blackout size, kW)','FontSize',font_size);
xlabel('Control type')
if strcmp(simulator,'acsimsep')
    text(2.5,290,'Blackout Sizes, S','FontSize',font_size)
else
    text(2.5,33,'Blackout Sizes, S','FontSize',font_size)
end
h1 = legend('S<10%','10%<S<20%','20%<S<30%','30%<S<40%','S>40%');
% h1 = legend('5%<S<10%','10%<S<20%','20%<S<30%','30%<S<40%','S>40%');
set(h1,'pos',get(h1,'pos')-[0,0.03,0,0])
legend boxoff
colormap('cool');
set(gca,'box','off')
set(gca,'FontSize',font_size);

%% plot number of messages
font_size = 16;
if strcmp(simulator,'dcsimsep')
    n_bins = 50*ones(1,4);
    min_per_bin = 50*ones(1,4);
else
    n_bins = 30*ones(1,4);
    min_per_bin = 80*ones(1,4); 
end
style = {'o-','s-','*-','x-'};
figure(550); clf;
for i = 1:4
    file_msg = sprintf('output/%s/%s/msg_distributed_control_nhop_%d_usempc_1_np_3.csv',...
        simulator, folder_name, i);
    data_msg = csvread(file_msg);
    [centers,density] = empirical_pdf(data_msg(:),n_bins(i),min_per_bin(i));
    figure(550);
    plot(centers,density,style{i},'markersize',10);
    hold on;
end
if strcmp(simulator,'dcsimsep')
    set(gca,'xlim',[0, 1200])
    set(gca,'xtick',0:200:1200)
else
    set(gca,'xlim',[0, 1600])
    set(gca,'xtick',0:200:1600)
end
h2 = legend('N_{hop}= 1','N_{hop}= 2','N_{hop}= 3','N_{hop}= 4','Location','Northeast');
set(h2,'pos',get(h1,'pos')-[0,0.01,0,0])
set(gca,'yscale','log')
set(gca,'FontSize',font_size);
legend boxoff
xlabel('Maximum number of messages per iteration for each agent','FontSize',font_size)
ylabel('Empirical probability')

%% record all MW lost for violin plot in R
MW_lost_percent = MW_lost_all/total_load*100;

% make the difference figure
figure(555); clf
for i = 1:5
    if i>=1 && i<=3
        h(i) = subplot(2,3,i);
        casename = sprintf('N_{hop}= %d',i);
    elseif i==4
        h(i) = subplot(2,2,3);
        casename = sprintf('N_{hop}= %d',i);
    elseif i==5
        h(i) = subplot(2,2,4);
        casename = 'Centralized';
    end
    this_diff = sort(MW_lost_percent(:,i+1)-MW_lost_percent(:,1));
    bar(this_diff)
    axis([-30, 1030, -100, 100])
    box off
    if i==1 || i==4
        ylabel('\DeltaS (% of total load)')
    end
    xlabel('case number')
    text(400,80,casename)
    set(gca,'xtick',[0:200:1000])
end
if strcmp(simulator,'dcsimsep')
    subplot(2,3,2);
    text(150,130,'DC simulator','FontWeight','bold','FontSize',18)
else
    subplot(2,3,2);
    text(170,130,'AC simulator','FontWeight','bold','FontSize',18)
end

return
%% save results in csv

if strcmp(simulator,'acsimsep')
    csvwrite('output/R/ac_final.csv',MW_lost_percent);
else
    csvwrite('output/R/dc_final.csv',MW_lost_percent);
end


