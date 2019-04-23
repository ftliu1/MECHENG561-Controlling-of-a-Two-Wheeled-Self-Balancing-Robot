function animation(t, theta, phi, r, makeVideo)
% generate animation
% t: time
% theta: body angle
% phi: wheel angle
% r :wheel radium

pauseLen = 0.05;

if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(50, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(30, 1/pauseLen));
        open(vo);
    end
end

    numSteps = length(t);
%     x_ini = phi(1) * r;
%     y_ini = r;
    
    for i = 1:numSteps
        figure(1); clf; hold on;
        x_pos = phi(i) * r;
        y_pos = r;
        
        wheel_pos = wheel(phi(i) * r, r, r, phi(i));

        
        dist_max = max(phi * r);
        dist_min = min(phi * r);
        
        margin = 0.1;
        
        cart_width = 0.03;
        cart_height = 0.15;
        bottom = 0.01;
        body_x = [x_pos + cart_width * cos(theta(i)) - bottom * sin(theta(i));
                  x_pos + cart_width * cos(theta(i)) + cart_height * sin(theta(i));
                  x_pos - cart_width * cos(theta(i)) + cart_height * sin(theta(i));
                  x_pos - cart_width * cos(theta(i)) + bottom * sin(theta(i))];
              
        body_y = [y_pos - cart_width * sin(theta(i)) - bottom * cos(theta(i));
                  y_pos - cart_width * sin(theta(i)) + cart_height * cos(theta(i));
                  y_pos + cart_width * sin(theta(i)) + cart_height * cos(theta(i));
                  y_pos + cart_width * sin(theta(i)) - bottom * cos(theta(i))];

        body = patch(body_x, body_y, 'r');
       
        
        plot([dist_min-margin, dist_max + margin],[0,0], 'k-.','LineWidth', 5);

                
        for numBand = 1:size(wheel_pos,1)-101
            plot([x_pos, wheel_pos(numBand+101,1)], [y_pos, wheel_pos(numBand+101,2)],'LineWidth',4,'Color',[0.5,0.5,0.5]);
        end
        plot(wheel_pos(1:101,1), wheel_pos(1:101,2), 'Color', [0.1 0.6 0.6], 'LineWidth',5);
        
        plot(x_pos, y_pos, 'ko', 'MarkerSize', 9, 'LineWidth', 2);
        
        p1 = plot([x_pos, x_pos], [y_pos, y_pos+0.2], 'k',  'LineWidth', 1.5);
        p2 = plot([x_pos, x_pos + 0.2 *sin(theta(i))], [y_pos, y_pos+0.2 *cos(theta(i))], 'b',  'LineWidth', 1.5);

        strt = {'Time: ', num2str(i/100)};
        text(dist_min - 0.7*margin,0.3,strt);
        
        str1 = {'Body angle: ', num2str(theta(i))};
        text(dist_min+0.07 - 0.7*margin,0.3,str1);
        
        str2 = {'Position: ', num2str(phi(i)*r)};
        text(dist_min+0.19 - 0.7*margin,0.3,str2);
        
        xlabel('Position/m');

        axis equal;
        axis([dist_min-margin, dist_max+1.5*margin,-0.03,0.33]);
        legend([p1,p2],{'Verticle Axis', 'Body Axis'});
        
        
        if makeVideo
            F = getframe(gcf);
            switch votype
              case 'avifile'
                vo = addframe(vo, F);
              case 'VideoWriter'
                writeVideo(vo, F);
              otherwise
                error('unrecognized votype');
            end
        end
    
    end

end