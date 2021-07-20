function plotter_draw_sphere(radius, center, mag_data)
    %{
    Graphs a wireframe sphere in a 3D plot.

    By default, this function draws a unit sphere
    (a sphere with a radius of 1 and centered at the origin).
    %}
    
    % Generate the x, y, and z data for the sphere
    if not(isnan(radius))
        r = radius * ones(30, 30); % radius of sphere
    else
        r = 1 * ones(30, 30); % radius of sphere is 1
    end
    [th, phi] = meshgrid(linspace(0, 2*pi, 30), linspace(-pi, pi, 30));
    [x,y,z] = sph2cart(th, phi, r);
    if not(isnan(center))
        x = x + center(1);  % center at center(1) in x-direction
        y = y + center(2);  % center at center(2) in y-direction
        z = z + center(3);  % center at center(3) in z-direction
    end
    
    % This is how we make the existing 3D plot
    %surf(peaks);
    % Now we use the surface command to add the sphere.
    % We just need to set the FaceColor as desired.
    figure('Name', 'Mag Sphere');
    hold on;
    title('Mag Sphere');
    surf(x,y,z,'FaceColor', 'none', 'EdgeColor', 'r') % draw the sphere mesh
    mag_x = mag_data(1,:);
    mag_y = mag_data(2,:);
    mag_z = mag_data(3,:);
    scatter3(mag_x, mag_y, mag_z) % plot the mag points
    xlabel('Mag X');
    ylabel('Mag Y');
    zlabel('Mag Z');
    view(-30, 20)
    saveas(gcf, 'out/kga_mag_cal.png');
    hold off;
end
