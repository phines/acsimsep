function ps2 = subgrid2ps(ps,i_sub)
% make a power system (ps) case from a subgrid in ps
C = psconstants;
nbr = size(ps.branch,1);
% find subgraphs 
nodes = ps.bus(:,C.bu.id);
br_st = (ps.branch(:,C.br.status) == 1);
links = ps.branch(br_st, [C.br.from, C.br.to]);
[graphNos,~,linkNos_con] = find_subgraphs(nodes,links);
% set up linkNos such that it contains all branches (connected or tripped)
linkNos = zeros(nbr,1);
linkNos(br_st) = linkNos_con;

% make ps2 from subgrid i_sub
ps2 = struct;
% make ps2.bus
busi_sub = find(graphNos == i_sub);
ps2.bus = ps.bus(busi_sub,:);
% make ps2.branch
bri_sub = (linkNos == i_sub);
ps2.branch = ps.branch(bri_sub,:);
% make ps2.gen
G = ps.bus_i(ps.gen(:,1));
i_gen = ismember(G,busi_sub);
ps2.gen = ps.gen(i_gen,:);
% make ps2.shunt
D = ps.bus_i(ps.shunt(:,1));
i_sh = ismember(D,busi_sub);
ps2.shunt = ps.shunt(i_sh,:);

ps2.baseMVA = ps.baseMVA;
ps2 = updateps(ps2);