%% Only knee joint
subplot(2,1,1);
if B_index == 1;
    plot(t,q_d(2,:),'-.k');
    hold on;
    plot(time,angle(:,2),'m');
end
plot(t,q(2,:));
legend('Desired','Real','15 Nms/rad','30 Nms/rad','45 Nms/rad','60 Nms/rad','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
xlim([1.5 4])
%ylim([-1 0])
%

subplot(2,1,2);
if B_index == 1;
    plot(time,angularVelocity(:,2),'m');
    hold on
end
plot(t,dq(2,:));
%legend('Real','Simulation','Orientation','Horizontal','Location','Best')
ylabel('Knee joint (rad/s)')
grid on
axis tight
set(gca, 'FontName', 'CMU Serif')
hold on
set(gca,'YMinorGrid','on')
set(gca,'XMinorGrid','on')
set(gca,'MinorGridLineStyle',':')
set(gca,'GridLineStyle',':')
if length(t) > length(time)
    xlim([0 t(end)])
else
    xlim([0 time(end)])
end
xlim([1.5 4])
xlabel('Time (s)')
pause(0.005)
return
%%
fig_w = 8.5*2;
fig_h = 8.5;
paper_w = fig_w;
paper_h = fig_h;

fig = gcf;
fig.PaperUnits = 'centimeters';
set(gcf, 'PaperSize', [fig_w*2+1 fig_h])
fig.PaperPosition = [0 0 paper_w*2+1 paper_h];
print('DampingPlots.jpeg','-djpeg',['-r' num2str(600)])
%print('DampingPlots.pdf','-dpdf','-r0')
saveas(gcf,'DampingPlots.svg')

%% Resistive Medium Plot
