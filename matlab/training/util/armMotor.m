classdef armMotor
    properties
        a;           % Arduino object
        motor_pin;   % Pin where the motor relay is connected
    end
    
    methods
        % Constructor method to create the arm_motor object
        function obj = armMotor()
            obj.a = arduino('COM6', 'Mega2560');
            obj.motor_pin = 'D7';
        end
        
        % Method to turn on the relay
        function turnOnRelay(obj)
            writeDigitalPin(obj.a, obj.motor_pin, 0); % Assume LOW signal turns on the relay
        end
        
        % Method to turn off the relay
        function turnOffRelay(obj)
            writeDigitalPin(obj.a, obj.motor_pin, 1); % Assume HIGH signal turns off the relay
        end
        
        % Method to pulse the motor for a fixed duration
        function pulse(obj, t)
            if t > 0
                obj.turnOnRelay();
                pause(t);
                obj.turnOffRelay();
            end
        end
    end
end
