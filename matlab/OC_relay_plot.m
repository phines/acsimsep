t = 1.2:0.01:10;
y = 7.5./(t-1);

%% plot figure
figure(5); clf
plot(t,y,'b');
hold on;
axis([0,10,0,35])
r = rectangle('Position',[1,0,0.5,15]);
r.FaceColor = [0 0.5 0.5];
r.EdgeColor = r.FaceColor;
plot([1,1],[0,max(y)],'k--')
xlabel(['$\displaystyle\frac{|I|}{I_{rated}}$',' (p.u.)'],'interpreter','latex')
ylabel('Time to trip (sec)')



