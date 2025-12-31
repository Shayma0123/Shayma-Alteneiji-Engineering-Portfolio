theta_Elv = [ 18 36 54 72 90];
theta_HP = 1;
h = [ 500 5e3 15e3 ];

S_target = linspace((15e3/60),(45e3/60),1500);

T = zeros(numel(S_target), numel(theta_Elv));
for j = 1:numel(h)
for k = 1:numel(theta_Elv)
T(:,k) = (h(j)*(cotd(theta_Elv(k)-theta_HP/2)-cotd(theta_Elv(k)+theta_HP/2)))./(S_target)';
end
if (j == 1)
   
    subplot(3,1,j)
 
plot(S_target,T(:,1),S_target,T(:,2),S_target,T(:,3),S_target,...
T(:,4),S_target,T(:,5));
    ylim([0 1])
legend('Elevation angle = 18°', ...
'Elevation angle = 36°','Elevation angle = 54°', ...
'Elevation angle = 72°', 'Elevation angle = 90°'); 
title('Comparing the time taken for a target with a given speed to exit the transmitting anntena beam range, h = 500 m ')
end
if (j == 2)
     
      subplot(3,1,j)
 
plot(S_target,T(:,1),S_target,T(:,2),S_target,T(:,3),S_target,...
T(:,4),S_target,T(:,5));
    ylim([0 5])
xlabel('Speed of the target in (m/s)');
ylabel('Time for which the target is in sight in (s)' )
title('h = 5 km')
end
if (j ==3)
      subplot(3,1,j)

plot(S_target,T(:,1),S_target,T(:,2),S_target,T(:,3),S_target,...
T(:,4),S_target,T(:,5));
    ylim([0 15])
title('h = 15 km')
end
end