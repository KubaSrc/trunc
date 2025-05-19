classdef armMotor
    properties
        s;           % Serial object
        motor_pin = 'D7'; % Pin where the motor relay is connected, if needed for reference
    end
    
    methods
        % Constructor method to create the arm_motor object
        function obj = armMotor()
            
            % Clean up ports
            objs = instrfind;
            if ~isempty(objs)
                fclose(objs);
                delete(objs);
            end

            % Define target device identifiers for the CH340 device.
            targetVID = 'VID_1A86';
            targetPID = 'PID_7523';
            
            % Query Win32_PnPEntity to include all COM ports.
            [status, cmdout] = system('wmic path Win32_PnPEntity where "Name like ''%(COM%''" get Name, DeviceID, PNPDeviceID');
            if status ~= 0
                error('Failed to retrieve device information.');
            end
            
            % Split the output into lines.
            lines = strsplit(cmdout, '\n');
            comPort = '';
            
            % Loop through each line to find the device matching your target VID and PID.
            for i = 1:length(lines)
                line = strtrim(lines{i});
                if contains(line, targetVID) && contains(line, targetPID)
                    % Extract the COM port using a regular expression (e.g., "COM6")
                    tokens = regexp(line, '(COM\d+)', 'match');
                    if ~isempty(tokens)
                        comPort = tokens{1};
                        break;
                    end
                end
            end
            
            if isempty(comPort)
                error('Target USB device with %s and %s not found.', targetVID, targetPID);
            end
            
            % Open the serial port using the discovered COM port.
            obj.s = serialport(comPort, 9600);

            flushinput(obj.s);
            flushoutput(obj.s);

            fopen(obj.s); % Open serial connection
            pause(2); % Allow some time for Arduino to reset and establish a serial connection
        end
        
        
        % Method to turn off the relay
        function toggle(obj)
            fprintf(obj.s, '%c', '-0'); % Send character '0' to Arduino for turning off the relay
        end
        
        % Method to pulse the motor for a fixed duration
        function pulse(obj, t)
            fprintf(obj.s, '%d', t); % Send character '0' to Arduino for turning off the relay
        end
        
        % Destructor method to close serial port when object is deleted
        function delete(obj)
            fclose(obj.s); % Close serial connection
            delete(obj.s); % Delete serial object
        end
    end
end
