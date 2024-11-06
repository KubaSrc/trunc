function target = set_servo_position_target(port, channel, target)
    % Inputs: 
    %   * port: serial port communicating with Maestro
    %   * channel: servo channel to control
    %   * degree: amount to rotate servo by
    % References:
    %   * https://www.pololu.com/docs/0J40/5.e

    target = round(4*target); % factor of 4 comes from Pololu documentation
    command = 0x84; % set target compact protocol
    bits = [binvec2dec(bitget(target, 1:7)), binvec2dec(bitget(target, 8:13))];
    send_servo_command(port, channel, command, bits);
end
