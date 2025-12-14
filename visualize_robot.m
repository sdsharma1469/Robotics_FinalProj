function visualize_robot(theta, L1, L2, titleStr)
% visualize_robot  Draws 2-link RR arm in XY with wall x + y = 2.

th1 = theta(1);
th2 = theta(2);

x1 = L1*cos(th1);
y1 = L1*sin(th1);

x2 = x1 + L2*cos(th1 + th2);
y2 = y1 + L2*sin(th1 + th2);

plot([0 x1 x2], [0 y1 y2], 'b-o', 'LineWidth', 2, ...
     'MarkerFaceColor', 'b'); hold on;

plot(0,0,'ko','MarkerFaceColor','k');
plot(x1,y1,'ko','MarkerFaceColor','k');
plot(x2,y2,'ro','MarkerFaceColor','r');

% Wall: x + y = 2
fplot(@(x) 2 - x, [0 2.5], 'k--', 'LineWidth', 1.5);

axis equal; grid on;
xlim([-0.5, 2.5]);
ylim([-0.5, 2.5]);
xlabel('X'); ylabel('Y');
title(titleStr);

hold off;
end
