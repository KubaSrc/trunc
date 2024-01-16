function initialize_servos(port, channels, pause_length,pos,home_position)
    %
    fprintf('Initializing servos...\n');
    
    if ~exist('channels', 'var')
        channels = 0:11;
    end
    if ~exist('pause_length', 'var')
        pause_length = 6;
    end
    i=1;
    for channel = channels 
        set_servo_speed(port, channel, 1);
        pause(.1);        
        set_servo_acceleration(port, channel, 0);
        pause(.1);
        set_servo_position_auxarm(port, channel, home_position(channel+1),pos);
        pause(.1);
        pos(i)=home_position(channel+1);
        i= i+1;
    end
    fprintf("Hold...\n");
    pause(pause_length);
%     stop_servos(port, channels);
    
end