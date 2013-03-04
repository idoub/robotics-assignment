classdef NXTRobot < handle
	% uses the NXT toolbox to control the physical robot
	% methods are equivalent to those in the RobotModel class
	
	properties (SetAccess = private)
        % nxt connection handle
		handle;
        % current position and orientation of the robot
        x = 0;
        y = 0;
        theta = pi/2;
        % counter used to alternate between clockwise and counterclockwise
        count = 0;
		
		% motors
		mForward;
		mReverse;
		mLeft1;
		mLeft2;
		mRight1;
		mRight2;
		mSensor;
	
		% motor properties
		cmToTacho = 25.714;				% how many times does the motor have to turn to move 1cm forward?
		radToTacho = 411.383;				% how many times does the motor have to turn to rotate the robot by one radian?
		
		% sensor properties
		ultraErrorMean = 0;			% mean of ultrasonic sensor error distribution
		ultraErrorStdDev = 1.095;		% standard deviation of ultrasonic sensor error distribution
    end
	
	methods
		% CONSTRUCTOR      
		function NXT = NXTRobot(x,y,theta)
            NXT.x = x;
            NXT.y = y;
            NXT.theta = theta;
        end
		
        % SETUP
		% set up connection to nxt robot
		function initConnect(NXT)
			% clean up workspace
			COM_CloseNXT('all')
			close all
			clear all

			% set the nxt handle
			NXT.handle = COM_OpenNXT();			% open usb connection
			COM_SetDefaultNXT(NXT.handle);    	% set default handle so we don't have to call it every time
		end
		
		% close connection to nxt robot
		function closeConnect(NXT)
			CloseSensor(SENSOR_4);
			COM_CloseNXT(NXT.handle);
		end
		
		% set up motors
		function initMotors(NXT)
			% set each of the three motor ports
			wRight = MOTOR_A;
			wSensor = MOTOR_B;
			wLeft = MOTOR_C;
			wheels  = [wRight; wLeft];			% we want both wheels to move together
			% default power - use different values for turning and moving in a line
			drivePower = 60;
			turnPower = 40;
			
			% initialise motor objects
			% driving motors
			NXT.mForward = NXTMotor(wheels, 'Power', -drivePower);
			NXT.mReverse = NXTMotor(wheels, 'Power', drivePower);
			% turning motors
			% note that each turning movement consists of two halves, each done by one motor
			NXT.mLeft1 = NXTMotor(wLeft, 'Power', turnPower);
			NXT.mLeft2 = NXTMotor(wRight, 'Power', -turnPower);
			NXT.mRight1 = NXTMotor(wRight, 'Power', turnPower);
			NXT.mRight2 = NXTMotor(wLeft, 'Power', -turnPower);
			
			% initialise sensor motor
			NXT.mSensor = NXTMotor(wSensor, 'Power', 50);
			
			% experiment with control parameters
            NXT.mForward.SpeedRegulation = false;
            NXT.mReverse.SpeedRegulation = false;
			NXT.mLeft1.SpeedRegulation = false;
			NXT.mLeft2.SpeedRegulation = false;
			NXT.mRight1.SpeedRegulation = false;
			NXT.mRight2.SpeedRegulation = false;
            NXT.mSensor.SpeedRegulation = false;
			% .SmoothStart = true;
			% .SpeedRegulation = false;
		end
		
		% set up sensors
		function initSensors(NXT, mean, stddev)
			OpenUltrasonic(SENSOR_4);
            NXT.ultraErrorMean = mean;
            NXT.ultraErrorStdDev = stddev;
		end
		
		% general setup
		function initAll(NXT)
			initConnect(NXT);
			initMotors(NXT);
            % adjust mean and std dev of error distribution based on pitch
			initSensors(NXT, 0, 1.5);
        end
        
		% MOVEMENT
        % general movement function
        function goto(NXT,newX,newY)
            % distance to travel (stright line)
            dist = sqrt((newX-NXT.x)^2 + (newY-NXT.y)^2)
            % angle to turn
            newAngle = atan2((newY-NXT.y),(newX-NXT.x))
            rotate = newAngle - NXT.theta
            % Keep the values of rotate between pi and -pi
            if rotate < -pi
                rotate = (2*pi) + rotate
            end
            if rotate > pi
                rotate = rotate - (2*pi)
            end
            % turn to either the left or the right
            % using standard trigonometric orientation
            if rotate < 0
                % turn right
                NXT.turnRight(-rotate);
            elseif rotate > 0
                % turn left
                NXT.turnLeft(rotate);
            end
            % now go to destination
            NXT.forward(dist);
            
            % update position
            NXT.x = newX;
            NXT.y = newY;
            NXT.theta = newAngle;            
        end
        
		% move forward
		function forward(NXT, dist)
			% set tacho based on distance to move
			NXT.mForward.TachoLimit = round(dist * NXT.cmToTacho);
			% NXT.mForward.Power = 50;
			NXT.mForward.SendToNXT();
			NXT.mForward.WaitFor();
		end
		
		% move backward
		function reverse(NXT, dist)
			% set tacho based on distance to move
			NXT.mReverse.TachoLimit = round(dist * NXT.cmToTacho);
			% NXT.mForward.Power = -50;
			NXT.mReverse.SendToNXT();
			NXT.mReverse.WaitFor();
		end
		
		% turn
		function turnLeft(NXT, angle)
			% set tacho based on angle of rotation
			NXT.mLeft1.TachoLimit = round(angle * NXT.radToTacho * 0.5);
            NXT.mLeft2.TachoLimit = NXT.mLeft1.TachoLimit;
			% NXT.mLeft1.Power = -80;
			% NXT.mLeft2.Power = 80;
			NXT.mLeft1.SendToNXT();
			NXT.mLeft2.SendToNXT();
			NXT.mLeft2.WaitFor();
            NXT.mLeft1.WaitFor();
        end
        function turnLeftOnly(NXT, angle)
                NXT.mLeft1.TachoLimit = angle * NXT.radToTacho;
                NXT.mLeft1.SendToNXT();
                NXT.mLeft1.WaitFor();
        end
		function turnRight(NXT, angle)
			% set tacho based on angle of rotation
			NXT.mRight1.TachoLimit = round(angle * NXT.radToTacho * 0.5);
            NXT.mRight2.TachoLimit = NXT.mRight1.TachoLimit;
			% NXT.mRight1.Power = -50;
			% NXT.mRight2.Power = 50;
			NXT.mRight1.SendToNXT();
			NXT.mRight2.SendToNXT();
			NXT.mRight2.WaitFor();
            NXT.mRight1.WaitFor();
        end
        
        function brake(NXT)
            NXT.mForward.Stop('off');
            NXT.mReverse.Stop('off');
            NXT.mLeft1.Stop('off');
            NXT.mLeft2.Stop('off');
            NXT.mRight1.Stop('off');
            NXT.mRight2.Stop('off');
        end
		
		% DO WE ACTUALLY NEED FORMULAE IN LECTURE???
		% COMPENSATE FOR ACCUMULATING ERRORS BY MEASURING TACHOMETRY???
		
        % SENSING
		% rotate ultrasonic sensor by given angle
		function turnSensor(NXT, angle)
            NXT.mSensor.TachoLimit = angle;
			NXT.mSensor.SendToNXT();
			NXT.mSensor.WaitFor();
		end
		
		function [out] = sense(NXT,numreadings)            
            angleChange = 360/numreadings;                                   % Calculate the change in angle according to the number of readings requested
            out = zeros(1,numreadings);
            
             % to ensure that we don't tangle the cable,
             % we alternately rotate clockwise and counterclockwise
             if NXT.count==0
                NXT.mSensor.Power = 60;
             else
                NXT.mSensor.Power = -60;
             end
             for m=1:numreadings                                                % For each reading
                out(m) = NXT.senseSingle();                             		% Take a sensor measurement
                NXT.turnSensor(angleChange);
             end
             
             % we return the results in clockwise order (0,pi/2,pi,3*pi/2)
             % so if we've rotated ccw, then shift the results
             if NXT.count==1
                 out = [out(1) out(end:-1:2)];
             end
             
             NXT.count = mod(NXT.count+1,2);
        end
		
		function reading = senseSingle(NXT)
			% get reading and compensate for possible error
			% readings are integers
			reading = round(GetUltrasonic(SENSOR_4) - ( NXT.ultraErrorMean + NXT.ultraErrorStdDev.*randn(1,1)));
            % reading = GetUltrasonic(SENSOR_4);
		end
		
	end
end