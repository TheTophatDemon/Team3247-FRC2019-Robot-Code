package frc.robot;
// HELO

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import java.lang.Runnable;

//TODO: Test Sonars
//TODO: Determine sonar "collision" range
//TODO: Test AutoDrive
/** Used only for driving the robot */
class DriveSystem extends Subsystem
{
    static final String kYaw = "NavX/Yaw";
    static final String kDisplacement = "NavX/Displacement";
    static final String kAcceleration = "NavX/Acceleration";
    static final String kMoving = "NavX/Is Moving?";
    static final String kTurning = "NavX/Is Turning?";
    static final String kResetDisplacement = "NavX/Reset Displacement";
    static final String kResetGyro = "NavX/Reset Gyro";
    static final String kLeftSonar = "Drive System/Left Sonar Value";
    static final String kRightSonar = "Drive System/Right Sonar Value";
    static final String kThrottleSensitivity = "Drive System/Throttle Sensitivity";
    static final String kTurnSensitivity = "Drive System/Turn Sensitivity";
    static final String kLeftEncoder = "Drive System/Left Encoder Value";
    static final String kRightEncoder = "Drive System/Right Encoder Value";
    static final String kResetEncoders = "Drive System/Reset Encoders";
    static final String kClimbSpeed = "Drive System/Climb Motor Speed";
    static final String kForceClimbUnlock = "Drive System/Force Open Climb Lock";
    static final String kP = "Drive System/P Constant";
    static final String kI = "Drive System/I Constant";
    static final String kD = "Drive System/D Constant";
    static final String kF = "Drive System/F Constant";
    static final String kAutoDriveButton = "Drive System/Auto Drive";
    static final String kAutoDriveValue = "Drive System/Auto Drive Value";
    static final String kAutoTurnButton = "Drive System/Auto Turn";
    static final String kAutoTurnValue = "Drive System/Auto Turn Value";
    static final String kTrackTarget = "Drive System/Track Vision Target";
    static final String kDisablePID = "Drive System/Disable PID Driving";
    static final String kPIDRange = "Drive System/PID Range";

    Talon leftDriveMotor = new Talon(0);
    Talon rightDriveMotor = new Talon(1);
    Spark climbMotor = new Spark(4);
    DoubleSolenoid climbLock = new DoubleSolenoid(2, 3);
    DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
    Encoder leftDriveEncoder = new Encoder(0, 1);
    Encoder rightDriveEncoder = new Encoder(2, 3);
    AnalogInput leftSonar = new AnalogInput(0);
    AnalogInput rightSonar = new AnalogInput(1);

    double throttleSensitivity = 1.0;
    double turnSensitivity = 0.75;
    double climbSpeed = 1.0;

    /**
     * Describes the automated movement being performed by the Drive System
    */
    AutoPhase autoState = null;

    public DriveSystem(Robot robot)
    {
        super(robot);

        climbLock.set(DoubleSolenoid.Value.kForward);

        double circumfrence = 0.5 * Math.PI; //in feet
        double distancePerPulse = circumfrence / 2048.0; //divided by pulses per revolution
        leftDriveEncoder.reset();
        leftDriveEncoder.setReverseDirection(true);
        leftDriveEncoder.setDistancePerPulse(distancePerPulse);
        rightDriveEncoder.reset();
        rightDriveEncoder.setDistancePerPulse(distancePerPulse);
    }
    @Override
    public void Reset()
    {
        if (autoState != null)
        {
            autoState.Close();
            autoState = null;
        }
    }
    public void TeleopDrive(double throttle, double turn)
    {
        if (autoState == null) //Interference with auto movement not allowed
        {
            if (robot.InClimbMode())
            {
                climbMotor.set(throttle * climbSpeed);
            }
            drive.arcadeDrive(throttle * throttleSensitivity, 
                turn * turnSensitivity);
        }
    }

    @Override
    public void Update(double deltaTime)
    {
        //Only re-lock when mast is completely up
        if (!robot.InClimbMode() && robot.mast.GetHeight() >= 0.0)
        {
            climbLock.set(DoubleSolenoid.Value.kForward);
        }
        if (SmartDashboard.getBoolean(kForceClimbUnlock, false))
        {
            climbLock.set(DoubleSolenoid.Value.kReverse);
        }

        //Update automated movements
        if (autoState != null)
        {
            if (!autoState.Update(deltaTime))
            {
                autoState = null;
            }
        }

        //Coordinate w/ Dashboard
        climbSpeed = SmartDashboard.getNumber(kClimbSpeed, climbSpeed);
        throttleSensitivity = SmartDashboard.getNumber(kThrottleSensitivity, throttleSensitivity);
        turnSensitivity = SmartDashboard.getNumber(kTurnSensitivity, turnSensitivity);

        if (DashboardUtils.IsButtonPressed(kDisablePID) && autoState != null)
        {
            autoState.Close();
            autoState = null;
        }
        if (DashboardUtils.IsButtonPressed(kAutoDriveButton))
        {
            TravelDistance(SmartDashboard.getNumber(kAutoDriveValue, 0.0), null);
        }
        if (DashboardUtils.IsButtonPressed(kResetEncoders))
        {
            leftDriveEncoder.reset();
            rightDriveEncoder.reset();
        }
    }
    @Override
    public void OnClimbingModeToggled()
    {
        if (robot.InClimbMode())
        {
            climbLock.set(DoubleSolenoid.Value.kReverse);
        }
    }
    /**
     * Uses a PID controller to move the robot forward the specified distance.
     * @param distance In feet
     * @param callback (optional) A functional interface that's called when the driving is complete.
     */
    public void TravelDistance(double distance, Runnable callback)
    {
        if (autoState != null) autoState.Close();
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
        autoState = new AutoDrivePhase(
            SmartDashboard.getNumber(kP, 1.0),
            SmartDashboard.getNumber(kI, 0.001),
            SmartDashboard.getNumber(kD, 1.0),
            SmartDashboard.getNumber(kF, 0.0),
            distance,
            callback
        );
    }
    @Override
    public void InitDashboard()
    {
        SmartDashboard.putNumber(kYaw, 0.0);
        SmartDashboard.putString(kDisplacement, "");
        SmartDashboard.putString(kAcceleration, "");
        SmartDashboard.putBoolean(kMoving, false);
        SmartDashboard.putBoolean(kTurning, false);
        SmartDashboard.putBoolean(kResetDisplacement, false);
        SmartDashboard.putBoolean(kResetGyro, false);
        SmartDashboard.putNumber(kThrottleSensitivity, throttleSensitivity);
        SmartDashboard.putNumber(kTurnSensitivity, turnSensitivity);
        SmartDashboard.putNumber(kLeftEncoder, 0.0);
        SmartDashboard.putNumber(kRightEncoder, 0.0);
        SmartDashboard.putBoolean(kResetEncoders, false);
        SmartDashboard.putNumber(kClimbSpeed, climbSpeed);
        SmartDashboard.putBoolean(kForceClimbUnlock, false);
        SmartDashboard.putBoolean(kAutoDriveButton, false);
        SmartDashboard.putNumber(kAutoDriveValue, 0.0);
        SmartDashboard.putBoolean(kAutoTurnButton, false);
        SmartDashboard.putNumber(kAutoTurnValue, 0.0);
        SmartDashboard.putBoolean(kTrackTarget, false);
        SmartDashboard.putBoolean(kDisablePID, false);
        SmartDashboard.putNumber(kP, 1.0);
        SmartDashboard.putNumber(kI, 0.001);
        SmartDashboard.putNumber(kD, 1.0);
        SmartDashboard.putNumber(kF, 0.0);
        SmartDashboard.putNumber(kPIDRange, 1.0);
        SmartDashboard.putNumber(kLeftSonar, 0.0);
        SmartDashboard.putNumber(kRightSonar, 0.0);
    }
    @Override
    public void SendToDashboard()
    {
        SmartDashboard.putNumber(kLeftEncoder, leftDriveEncoder.getDistance());
        SmartDashboard.putNumber(kRightEncoder, rightDriveEncoder.getDistance());
        SmartDashboard.putNumber(kLeftSonar, leftSonar.getVoltage());
        SmartDashboard.putNumber(kRightSonar, rightSonar.getVoltage());
    }

    /** 
     * 
     * Sub-classes for automated driving operations
     * 
    */

    class AutoDrivePhase implements AutoPhase
    {
        double stopTimer = 0.0;
        PIDController pid;
        Runnable callback;
        AutoDrivePhase(double p, double i, double d, double f, double distance, Runnable callback)
        {
            pid = new PIDController(p, i, d, f, leftDriveEncoder, 
                (double output)->{ drive.arcadeDrive(output, 0.0); });
            pid.setAbsoluteTolerance(0.1);
            pid.setSetpoint(distance);
            pid.setEnabled(true);
            this.callback = callback;
        }
        @Override
        public boolean Update(double deltaTime)
        {
            if (pid.onTarget())
            {
                stopTimer += deltaTime;
                if (stopTimer > 0.5)
                {
                    Close();
                    return true;
                }
            }
            return false;
        }
        @Override
        public void Close()
        {
            pid.close();
            if (callback != null)
            {
                callback.run();
            }
        }
    }
}