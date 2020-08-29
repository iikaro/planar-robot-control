O = [0 0];  %origin
L = l;
% Plot a circle at the origin
rc = 0.01;
th = 0:pi/50:2*pi;
xunit = rc * cos(th) + O(1,1);
yunit = rc * sin(th) + O(1,2);

% Figure handle
h = figure;

% "Real" robot
% First joint
A = L(1)*[cos(q(1,:)); sin(q(1,:))];
% Second joint
B = A + L(2)*[cos(q(2,:) + q(1,:)); sin(q(2,:) + q(1,:))];

% "Desired" robot
% First joint
Ad = L(1)*[cos(q_d(1,:)); sin(q_d(1,:))];
% Second joint
Bd = A + L(2)*[cos(q_d(2,:) + q_d(1,:)); sin(q_d(2,:) + q_d(1,:))];

% Animation
for i = 1 : length(t)
    if(ishandle(h))
        plot([O(1,1) A(1,i) B(1,i)], [O(1,2) A(2,i) B(2,i)], 'k', xunit, yunit, 'k', 'LineWidth', 3)    %plot the real robot
        axis([-1.5 1.5 -1.5 1.5])   %axes limits
        hold on
        plot([Ad(1,i) Bd(1,i)], [Ad(2,i) Bd(2,i)], ':b', 'LineWidth', 1)    %plot desired robot
        grid on
        hold off
        
        % Color environment
        r = rectangle('Position',[-2 y_w - 2 4 2]');
        r.FaceColor = [0 0.5 1 0.05];
        r.EdgeColor = 'None';
        r.LineStyle = 'None';
        % Draw interface line
        hline(y_w, 'k', 'Environment');
        
        % UI with basic information
        delete(findall(gcf,'type','annotation'));
        dim = [0.3, 0.7, 0.1, 0.1];
        offset = q_v(2,i)*180/pi;
        str=sprintf('error = %.2f deg \noffset = %.2f deg', (q_d(2,i) - q(2,i))*180/pi, offset); %if No floting varibale number, use %d
        annotation('textbox',dim,'String',str,'BackgroundColor','w','FitBoxToText','on','fontsize',10);
        drawnow;
        pause(5e-3)
    end
end