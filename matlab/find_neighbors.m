clear variables
load('ps_polish_all','ps_polish_100')
ps = ps_polish_100;

all_neighbors = zeros(2383,5);

% do this for all number of hops in 1-4
for k = 1:5
    % do it the easy way as we have done this before
    opt.sim.nHopLoc = k;
    ps_agents = set_up_agents(ps,opt);
    neighbor_count = [];
    nbus = size(ps.bus,1);
    for i = 1:nbus
        neighbor_count = [neighbor_count, length(ps_agents(i).loc_nei)-1]; %#ok<AGROW> % -1 to exclude the node itself
    end
    fprintf('hop = %d, average number of neighbors: %.1f, maximum: %d.\n', ...
        opt.sim.nHopLoc, mean(neighbor_count), max(neighbor_count));
    all_neighbors(:,k) = neighbor_count;
end

csvwrite('all_neighbors.csv',all_neighbors)
    