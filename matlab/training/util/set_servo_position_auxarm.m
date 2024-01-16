% set_servo_position(port, channel, deflection)
% 
% Inputs: 
%   * port: serial port communicating with Maestro
%   * channel: servo channel to control
%   * degree: amount to rotate servo by


% References:
%   * https://www.pololu.com/docs/0J40/5.e
function set_servo_position_auxarm(port, channel,deflection,pos)
    global position 
    position= pos;
    d = 37.7; % d is the diameter of the winch + cable attached to the motor.
    
    deg = (deflection*360)/(pi*d); % calculating the amount of rotation for the motor. 
    pwm_change = deg/1.57;  % calculating the pwm equivalent for the required rotation. 
    
    zero_offset = 1600;
    position(channel+1) = deflection;
    
    target = round((pwm_change + zero_offset)*4); % factor of 4 comes from Pololu documentation
    command = 0x84; % set target compact protocol
    bits = [binvec2dec(bitget(target, 1:7)), binvec2dec(bitget(target, 8:13))];
    position(channel+1);
%     fprintf("Set channel %d position to %.2f\n", channel, degree);
    send_servo_command(port, channel, command, bits);
%     pause(3);
    %return position;
end
