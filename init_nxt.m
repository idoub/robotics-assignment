function init_nxt
    % Prepare MATLAB
    COM_CloseNXT all
    close all
    clear all

    % Connect to NXT, via USB or BT
    h = COM_OpenNXT();
    COM_SetDefaultNXT(h);
    
    % Set some parameters:
    rightWheel   = MOTOR_A;
    leftWheel  = MOTOR_C;
    bothWheels  = [rightWheel; leftWheel];
    drivingPower = 50;
    turningPower = 80;
    drivingDist  = 1000; % in degrees
    turningDist  = 1300;  % in degrees

    % now create the objects for straigt driving:
    mReverse = NXTMotor(bothWheels, 'Power', drivingPower, 'TachoLimit', drivingDist);
    mForward = mReverse; % clone object
    mForward.Power = -mReverse.Power; % just swap the power sign
    
    % for turning the bot, we have two objects each:
    mTurnLeft1 = NXTMotor(leftWheel, 'Power', -turningPower, 'TachoLimit', turningDist);
    mTurnLeft1.SpeedRegulation = false; % don't need this for turning

    % for the 2nd part of turning, use first part's settings and modify:
    mTurnLeft2 = mTurnLeft1;                 % copy object
    mTurnLeft2.Port     = rightWheel;        % but use other wheel
    mTurnLeft2.Power    = -mTurnLeft1.Power; % swap power again

    % the right-turn objects are the same, but mirrored:
    mTurnRight1 = mTurnLeft1;               % first copy...
    mTurnRight2 = mTurnLeft2;
    mTurnRight1.Power = -mTurnRight1.Power; % now mirror powers
    mTurnRight2.Power = -mTurnRight2.Power;
    % Instead of mirroring the powers, we could've also changed
    % the ports (swapped left and right wheels).
    
% let the bot drive forward
mForward.SendToNXT();
mForward.WaitFor();
end