function h = plot_pdf_pl(X,n_bins,min_per_bin,style)
% Plot the empirical pdf of X

if nargin<4
    style = '.-';
end

% Compute the pdf
[centers,density,ranges] = empirical_pdf_pl(X,n_bins,min_per_bin);

% plot
if 0
    for i = 1:length(centers)
        plot(ranges(:,i)',[density(i) density(i)],'k-');
        hold on;
    end
end

h = plot(centers,density,style,'markersize',30);

set(gca,'xscale','log');
set(gca,'yscale','log');

