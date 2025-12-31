theta_Elv = [ 18 36 54 72 90];
theta_HP = 1;
h2 = 10:10:15e3; 
d2 = zeros(numel(h2),numel(theta_Elv));
for m = 1:numel(theta_Elv)
    	d2(:,m) = (h2.*(cotd(theta_Elv(m)-(theta_HP/2)) ...
        -cotd(theta_Elv(m)+(theta_HP/2))))';
end


semilogy(h2,d2(:,1),h2,d2(:,2),h2,d2(:,3),h2,...
d2(:,4),h2,d2(:,5));

set(gca,'ColorOrderIndex',1); % reset color index for better comparison
legend('Elevation angle = 18°', ...
'Elevation angle = 36°','Elevation angle = 54°', ...
'Elevation angle = 72°', 'Elevation angle = 90°'); 
ylabel('Range of the receiveing anntena beam in m')

grid on;
xlabel('Vertical distance from the target to the reciever in m' );
title(['Comparing the range of the receiveing anntena beam for' ...
    	' different hights of a given target']);