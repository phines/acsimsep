function [centers,density,ranges] = empirical_pdf(X,n_bins,min_per_bin)
% usage: [centers,density,ranges] = empirical_pdf(X,n_bins,min_per_bin)
% Build an empirical probability density function for the data in X

n = length(X);
perturbance = 1 + eps;

% initial set of edges
x_min = min(X);
x_max = max(X);
dx = (x_max - x_min)/n_bins;

% log_min = log(min(X(X>0)));
% log_max = log(max_x);
% log_dx  = (log_max-log_min)/n_bins;

% produce initial set of edges and count
edges = x_min:dx:x_max;
counts = histcounts(X,edges);
% Find the point at which the bins are too small
too_small = counts<min_per_bin;
if sum(too_small)>0
    first_too_small = find(too_small,1,'first');
    % dump the right side of the distribution
    edges((first_too_small+1):end) = [];
    X = sort(X);
    while(edges(end)<x_max)
        %fprintf('There are %d bins that are too small\n',sum(too_small));
        % find the end points for this edge
        left = edges(end);
        % find the x values that are larger than this
        bigger_index = find(X>left);
        right_index = bigger_index(min_per_bin);
        mid_point = mean(X(right_index + (0:1)));
        % make sure that enough points remain
        if sum(X>mid_point)>min_per_bin
            right = mid_point * perturbance; % in case X(right_index) == X(right_index+1)
        else
            right = x_max * perturbance;
        end
        edges = [edges right]; %#ok<AGROW>
        count = sum(X>left & X<right);
        if count<min_per_bin
            keyboard
        end
    end
end

ne = length(edges);
ranges = [edges(1:(ne-1)); edges(2:ne)];
centers = mean(ranges,1);
bin_width = ranges(2,:) - ranges(1,:);
counts = histcounts(X,edges);
density = counts./bin_width/n;

if any(density==0)
    keyboard
end

