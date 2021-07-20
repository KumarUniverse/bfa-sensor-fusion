function plotter_draw_mag_sphere(x_data, y_data, z_data)
    % Graphs a 3D scatter plot of tri-axis magnetometer data.
    addpath('src/lib');
    
    x_data = table2array(x_data).';
    y_data = table2array(y_data).';
    z_data = table2array(z_data).';
    mag_data = [x_data; y_data; z_data]; % size: 3 x 4674
    center = [0,0,0];
    plotter_draw_sphere(1, center, mag_data);
end

