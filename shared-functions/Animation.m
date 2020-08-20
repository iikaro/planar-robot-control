function Animation(q, q_d, L, t, offset, limit, dof)
O = [0 0];  %origin

% Plot a circle at the origin
rc = 0.01;
th = 0:pi/50:2*pi;
xunit = rc * cos(th) + O(1,1);
yunit = rc * sin(th) + O(1,2);

h = figure;

if dof == 1
    EE = [L*cos(q + offset) L*sin(q + offset)];     %end-effector
    D = L*[cos(q_d + offset) sin(q_d + offset)];    %desired movement

    for i = 1 : length(t)
        if(ishandle(h))
            plot([O(1,1) EE(i,1)], [O(1,2) EE(i,2)], 'k', xunit, yunit, 'k', 'LineWidth', 3)
            axis([-0.5 0.5 -0.5 0.5])
            grid on
            hold on
            plot([O(1,1) D(i,1)], [O(1,2) D(i,2)], ':b', 'LineWidth', 1)
            hold off
            delete(findall(gcf,'type','annotation'));
            r = rectangle('Position',[-0.5 limit 1 0.5]');
            r.FaceColor = [.01 .01 .01 0.05];
            r.EdgeColor = 'None';
            r.LineStyle = 'None';
            hline(limit, 'k', 'Environment');
            %dim = [0.3, 0.7, 0.1, 0.1];
            %str=sprintf('offset = %.2f \npenetration = %.2f', q_r(2,i)*180/pi, (wall - x(i))); %if No floting varibale number, use %d
            %annotation('textbox',dim,'String',str);
            %drawnow;
            pause(5e-12)
        end
    end
end

if dof == 2
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
            r = rectangle('Position',[0.5 -2 1.5 4]');
            r.FaceColor = [.01 .01 .01 0.05];
            r.EdgeColor = 'None';
            r.LineStyle = 'None';
            vline(0.5, 'k', 'Environment');
            dim = [0.3, 0.7, 0.1, 0.1];
            %offset = q_v(2,i)*180/pi;
            offset = 0;
            %penetration = (limit - x(1,i))*1000;
            penetration = 0;
            if(penetration > 0)
                penetration = 0;
            end
            str=sprintf('error = %.2f deg \noffset = %.2f deg \npenetration = %.2f mm', (q_d(2,i) - q(2,i))*180/pi, offset, penetration); %if No floting varibale number, use %d
            annotation('textbox',dim,'String',str,'BackgroundColor','w','FitBoxToText','on','fontsize',10);
            drawnow;
            pause(5e-12)
        end
    end
end
end