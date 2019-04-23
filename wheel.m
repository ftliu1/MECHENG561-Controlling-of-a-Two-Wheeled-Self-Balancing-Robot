function pos = wheel(x, y, r, phi)
% generate the wheel
% x: x-position
% y: y-position
% r: radium

    angle = 0:pi*0.02:2*pi;
    m = length(angle);
    pos(:,1) = x + r.* cos(angle);
    pos(:,2) = y + r.* sin(angle);
    
    n = 6;
    init_ang = 0:2*pi/n:2*pi;
    
    rot_mat = [cos(-phi), -sin(-phi); sin(-phi), cos(-phi)];
    
    for i = 1:length(init_ang)
        x_ini_b = r * cos(init_ang(i));
        y_ini_b = r * sin(init_ang(i));
        
        pos_b = rot_mat * [x_ini_b; y_ini_b];
        
        band(i,1) = pos_b(1) + x;       
        band(i,2) = pos_b(2) + y;
    end

    pos = [pos; band];
end