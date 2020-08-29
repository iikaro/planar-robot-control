O = [0 0];  %origin
L = l;
% Plot a circle at the origin
rc = 0.01;
th = 0:pi/50:2*pi;
xunit = rc * cos(th) + O(1,1);
yunit = rc * sin(th) + O(1,2);

h = figure;

A = L(1)*[cos(q(1,:)); sin(q(1,:))];
B = A + L(2)*[cos(q(2,:) + q(1,:)); sin(q(2,:) + q(1,:))];

D = A + L(2)*[cos(q_d(2,:) + q_d(1,:)); sin(q_d(2,:) + q_d(1,:))];

for i = 1 : length(t)
    if(ishandle(h))
        plot([O(1,1) A(1,i) B(1,i)], [O(1,2) A(2,i) B(2,i)], 'k', xunit, yunit, 'k', 'LineWidth', 3)
        axis([-1.5 1.5 -1.5 1.5])
        hold on
        plot([A(1,i) D(1,i)], [A(2,i) D(2,i)], ':b', 'LineWidth', 1)
        grid on
        hold off
        delete(findall(gcf,'type','annotation'));
        r = rectangle('Position',[x_w(1) -2 (x_w(1) + 1) 4]');
        r.FaceColor = [.01 .01 .01 0.05];
        r.EdgeColor = 'None';
        r.LineStyle = 'None';
        vline(x_w(1), 'k', 'Environment');
        dim = [0.3, 0.7, 0.1, 0.1];
        offset = q_v(2,i)*180/pi;
        penetration = (x_w(1) - x(1,i))*1000;
        if(penetration > 0)
            penetration = 0;
        end
        str=sprintf('error = %.2f deg \noffset = %.2f deg \npenetration = %.2f mm', (q_d(2,i) - q(2,i))*180/pi, offset, penetration); %if No floting varibale number, use %d
        annotation('textbox',dim,'String',str,'BackgroundColor','w','FitBoxToText','on','fontsize',10);
        drawnow;
        pause(5e-3)
    end
end