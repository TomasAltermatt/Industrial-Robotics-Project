function trajectory = Trajectory_2(a, b, n_points, line_start, elipse_start)
    % line start point
    x0 = line_start(3);
    y0 = line_start(2);
    z0 = line_start(1);
    phi_0 = line_start(4);
    
    % ellipse start point
    xe = elipse_start(3);
    ye = elipse_start(2);
    ze = elipse_start(1);
    phi_e = elipse_start(4);


    % Horizontal line
    x_linea = x0*ones(1, n_points);
    y_linea = linspace(y0, ye, n_points);
    z_linea = linspace(z0, ze, n_points);
%     line_angles = phi_0*ones(1, n_points);
    line_angles = linspace(phi_0, (phi_0 + phi_e)/2, n_points);

    
    line_seq = [x_linea; y_linea; z_linea; line_angles];

    % Kind of a quarter of the elipse
    t = linspace(0.96*pi, 9*pi/20, n_points);

    % Elipse equations, modified so it has some offset and not perfect rounded
    x_elipse = xe*ones(1, n_points);
    y_elipse = (ye+a) + a*cos(t);
    z_elipse = (ze - b*(sin(0.96*pi).^0.8)) + b*(sin(t).^0.8);
%     elipse_angles = phi_0 - 0.5*(pi - t);
    elipse_angles = linspace((phi_0 + phi_e)/2, phi_e, n_points);
    elipse_seq = [x_elipse; y_elipse; z_elipse; elipse_angles];

    % Put both trajectories together (this is the output result)
    trajectory = [line_seq, elipse_seq];
    

    % Soft the union of two curves to avoid hard edge and full stop in one axis
    trajectory(1, length(trajectory)/2) = x_linea(end) - (x_linea(end) - x_linea(end-1))/3;
    trajectory(2, length(trajectory)/2) = y_linea(end) - (y_linea(end) - y_linea(end-1))/3;
    trajectory(3, length(trajectory)/2) = z_linea(end) - (z_linea(end) - z_linea(end-1))/3;
    
    trajectory(1, 1+(length(trajectory)/2)) = x_elipse(1) + (x_elipse(2) - x_elipse(1))/3;
    trajectory(2, 1+(length(trajectory)/2)) = y_elipse(1) + (y_elipse(2) - y_elipse(1))/3;
    trajectory(3, 1+(length(trajectory)/2)) = z_elipse(1) + (z_elipse(2) - z_elipse(1))/3;
    
    trajectory_flip = trajectory;
    for i = 1:4
        trajectory_flip(i, :) = flip(trajectory(i, :));
    end
    
    trajectory =[trajectory, trajectory_flip(:,2:end)];
    x_seg = trajectory(1,:);
    z_seg = trajectory(3,:);
    
    trajectory(1,:) = z_seg;
    trajectory(3,:) = x_seg;
    

    % Plot
    % figure;
    % plot(trajectory(2,:), trajectory(3,:), 'rx-', 'LineWidth', 2);
    % grid on;
    % xlabel('x');
    % ylabel('y');
    % title('Trajectory');
    % axis equal;
end