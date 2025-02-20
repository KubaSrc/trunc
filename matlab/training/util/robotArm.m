classdef robotArm
    % Attributes
    properties
        port;
        channels = 0:8;
        comp = load("./state/comp.mat").comp;
        comp_max = load("./state/comp_max.mat").comp_max;
        home_pos = load("./state/home_measured.mat").pos;
        pause_length = 3;
        min_motor = -250;
        max_motor = 150;
        nnc;
        home_relative;
        arm_ID = 8
        stick_ID = 9;
        speed=2;
    end

    methods
        % Constructor
        function obj = robotArm(speed)
            addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
            addpath('./util/')

            % Set speed if provided by user
            if nargin > 0
                obj.speed = speed; % Override default value
            end
        if ~exist('obj.port', 'var')
            % Define target device identifiers.
            targetVID = 'VID_1FFB';
            targetPID = 'PID_008A';
            targetMI  = 'MI_00';  % We want the device with MI_00 (COM4)

            
            % Query Win32_PnPEntity to include all COM ports.
            [status, cmdout] = system('wmic path Win32_PnPEntity where "Name like ''%(COM%''" get Name, DeviceID, PNPDeviceID');
            if status ~= 0
                error('Failed to retrieve device information.');
            end
            
            % Split the output into lines.
            lines = strsplit(cmdout, '\n');
            comPort = '';
            
            % Loop through each line to find the device matching your VID and PID.
            for i = 1:length(lines)
                line = strtrim(lines{i});
                if contains(line, targetVID) && contains(line, targetPID) && contains(line,targetMI)
                    disp(line)
                    % Extract the COM port using a regex (e.g., "COM6")
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
            obj.port = serialport(comPort, 9600);
            disp(comPort)
            initialize_servos(obj.port, obj.channels, obj.pause_length, obj.comp, obj.speed);
            pause(5);
        end
            
            % Connect to motive
            if ~exist('nnc','var')
                obj.nnc = connect_to_natnet();
            end
            
        end

        % Get the pose of a specific body ID
        function pose = get_pose_id(obj,id)
            for i = 1:obj.nnc.getFrame.nRigidBodies
                body = obj.nnc.getFrame().RigidBodies(i);
                body_id = body.ID;
                if body_id == id
                    pose=body;
                    break;
                end
            end
        end

        
        % Read in position of arm from Mocap
        function tool = get_pose(obj)
            if obj.nnc.getFrame.nRigidBodies >= 2
                tool = obj.get_pose_id(obj.arm_ID);
            else
                tool = obj.nnc.getFrame().RigidBodies(1); %End Effector
            end
        end

        % Read in position of stick from Mocap
        function tool = get_pose_stick(obj)
            tool = obj.get_pose_id(obj.stick_ID);
        end

        % Read in position from Mocap
        function tool_pos = get_pose_absolute(obj)
            % Obtain rigid body state
            tool = obj.nnc.getFrame().RigidBodies(1); %End Effector

            xi = [1000.*tool.x, 1000.*tool.y, 1000.*tool.z, tool.qw, tool.qx, tool.qy, tool.qz];

            % Coordinate transform relative to home_pos
            xi(1:3) = (xi(1:3) - obj.home_relative(1:3)) + obj.home_pos(1:3);

            q1 = obj.home_pos(4:end);
            p1 = obj.home_pos(1:3);
            q2 = obj.home_relative(4:end);
            p2 = obj.home_pos(1:3);
            qi = xi(4:end);
            pi = xi(1:3);

            % Transform based on coordinate frames
            [qi_new,pi_new] = coordinate_transform(q1,p1,q2,p2,qi,pi);
            
            qi_swap = [qi_new(2:end),qi_new(1)];

            tool_pos = [pi_new.',qi_swap]; 
            
        end

        % Set relative home position
        function obj = set_relative_home(obj)
            pause(2);
            tool = obj.get_pose();
            obj.home_relative = [1000.*tool.x, 1000.*tool.y, 1000.*tool.z, tool.qw, tool.qx, tool.qy, tool.qz];
            disp(obj.home_relative)
        end
        
        % Method to set the position
        function set_pos(obj, motor_pos)
            
            if min(motor_pos) < obj.min_motor - eps
                error('Motor lower bound exceeded! Sent: %.2f Bound: %.3f', min(motor_pos), obj.min_motor);

            elseif max(motor_pos) > obj.max_motor + eps
                error('Motor upper bound exceeded! Sent: %.2f Bound: %.3f', max(motor_pos), obj.max_motor);

            else
                final_pos = zeros(1,8);
                current_pos = zeros(1,8);
                threshold = 1;
                reached = false;
                
                % Sends commands to servo motors
                for idx = 0:8
                    target = set_servo_position_auxarm(obj.port, idx, motor_pos(idx+1));
                    final_pos(idx+1) = target;
                end
                
                % Record initial time
                startTime = tic;
    
                % Waits until all servo motors are done moving
                while ~reached
                    % Read positions
                    current_pos = zeros(1, 9); % Preallocate array for current positions
                    for idx = 0:8
                        current = get_servo_position(obj.port, idx);
                        current_pos(idx+1) = current;
                    end
            
                    % Calculate error
                    e = sum((final_pos - current_pos).^2);
            
                    % Check threshold
                    if e <= threshold
                        reached = true;
                    end
            
                    % Check if timeout (15 seconds) is exceeded
                    if toc(startTime) > 20
                        fprintf('Timeout: Servos did not reach the desired position within 20 seconds.\n');
                        disp(e)
                        break;
                    end
                end
            end
        end


        % Setting position directly
        function set_pos_target(obj,target)
            % Sends commands to servo motors
            for idx = 0:8
                set_servo_position_target(obj.port, idx, target(idx+1));
            end
        end

            
        

        % Set pos based on l_delta
        function set_pos_delta(obj,l_delta)
            % Set arm to new pose
            arm_pos = double(obj.comp+l_delta);
            % Prevents the arm from letting too much slack out
            if max(arm_pos) > obj.max_motor
                motor_slice = arm_pos > obj.max_motor;
                disp("Clipping l_delta.")
                l_delta(motor_slice) = l_delta(motor_slice) - (arm_pos(motor_slice)-obj.max_motor);
                arm_pos = double(obj.comp+l_delta);
            end

            % Send command to servos
            obj.set_pos(arm_pos)
        end

        % Sends arm to compressed state and then cycles a known trajectory
        function reset_arm(obj)
            fprintf('Resetting arm.\n')
            obj.set_pos(obj.comp)
            for rep = 1:5
                pause(.5)
                obj.set_pos(obj.comp_max)
                pause(.5)
                obj.set_pos(obj.comp)
            end
        end

        % Method to turn off the motors
        function stop_motors(obj)
            stop_servos(obj.port, obj.channels);
        end
    end
end
