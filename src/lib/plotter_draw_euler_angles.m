function plotter_draw_euler_angles(euler_table, HIDE_ROLL)
    % Plot the Euler angles (yaw, pitch, roll)
    % calculated in kga_quat.m (lg_angles)
    
    euler_matrix = table2array(euler_table);
    time = euler_matrix(:,4);
    figure('Name', 'Euler Angles (lg angles)');
    hold on;
    plot(time, euler_matrix(:,1), 'r'); % phi or yaw
    plot(time, euler_matrix(:,2), 'g'); % theta or pitch
    if not(HIDE_ROLL) % if true, don't graph roll
        plot(time, euler_matrix(:,3), 'b'); % psi or roll
    end
    title('Euler angles (lg angles)');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    if HIDE_ROLL % if true, don't graph roll
        legend('\phi', '\theta');
    else
        legend('\phi', '\theta', '\psi');
    end
    hold off;
end

