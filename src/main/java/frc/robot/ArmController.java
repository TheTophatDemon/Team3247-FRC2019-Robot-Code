package frc.robot;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//716
//713
//749

//TODO: Test Support Solenoids
//TODO: Test encoder bias
//TODO: Calculate ticks per inch
class ArmController extends Subsystem
{
    static final String kEncoder = "Arm/Encoder Value";
    static final String kEncoderBias = "Arm/Encoder Bias";
    static final String kEncoderReset = "Arm/Encoder Reset";
    static final String kMoveSpeed = "Arm/Move Speed";
    static final String kHoldSpeed = "Arm/Hold Speed";
    static final String kTargetHeight = "Arm/Target Height";
    static final String kMoveToTarget = "Arm/Move To Target";

    public static final double ENCODER_RANGE = 6000.0;

    VictorSP turnMotor = new VictorSP(6);
    Encoder encoder = new Encoder(6, 7);
    ///Operates both of the solenoids that push on the arm as it moves up
    DoubleSolenoid supportSoln = new DoubleSolenoid(0, 1);
    DoubleSolenoid.Value pushUpVal = DoubleSolenoid.Value.kForward;
    DoubleSolenoid.Value pullDownVal = DoubleSolenoid.Value.kReverse;

    boolean moveToTarget = false;
    double targetPosition = 0.0;
    double moveSpeed = 1.0;
    double holdSpeed = 0.15;
    double lastDifference = 0.0;


    public ArmController(Robot robot)
    {
        super(robot);

        turnMotor.setInverted(true);
        encoder.reset();
    }
    @Override
    public void Reset()
    {
        CancelAutomatedMovement();
    }
    @Override
    public void Update(double deltaTime)
    {
        moveSpeed = SmartDashboard.getNumber(kMoveSpeed, moveSpeed);
        double acceptableError = 0.1;
        if (moveToTarget)
        {
            double difference = targetPosition - GetPosition();
            if (GetPosition() < targetPosition)
            {
                //turnMotor.set(moveSpeed);
                SetMovement(1);
                if(difference < acceptableError)
                {
                    moveToTarget = false;
                }
            }
            else
            {
                //turnMotor.set(-moveSpeed);
                SetMovement(-1);
                if(difference > acceptableError)
                {
                   moveToTarget = false;
                }
            }
            if (Math.signum(lastDifference) != Math.signum(difference))
            {
                moveToTarget = false;
            }
            lastDifference = difference;
        }

        if (DashboardUtils.IsButtonPressed(kMoveToTarget))
        {
            SetTargetPosition(
                //Keep in mind that the DB value is a fraction of the maximum range.
                SmartDashboard.getNumber(kTargetHeight, 0.0) * ENCODER_RANGE
            );
        }
        if (DashboardUtils.IsButtonPressed(kEncoderReset))
        {
            encoder.reset();
        }
    }
    /**
     * Sets the arm to move to a certain percentage of its range.
     * @param position 0-1, where 0 is the lowest position and 1 is the highest.
     */
    public void SetTargetPosition(double position)
    {
        moveToTarget = true;
        targetPosition = position;
        lastDifference = targetPosition - GetPosition();
    }
    public double GetTargetPosition()
    {
        return targetPosition;
    }
    public double GetPosition()
    {
        return encoder.getDistance() + SmartDashboard.getNumber(kEncoderBias, 0.0);
    }
    public void CancelAutomatedMovement()
    {
        moveToTarget = false;
    }
    ///Returns true if the arm is undergoing autonomous movement.
    public boolean IsAutoMoving()
    {
        return moveToTarget;
    }
    /**
     * Starts the arm moving in a certain direction
     * If direction is not 0, all automated movement is cancelled.
     * @param direction -1 for down, 0 to hold still, 1 for up
     */
    public void SetMovement(int direction)
    {
        if (direction > 0)
        {
            if (GetPosition() < ENCODER_RANGE)
            {
                turnMotor.set(moveSpeed);
            }
            else
            {
                direction = 0;
            }
            supportSoln.set(pushUpVal);
        }
        else if (direction < 0)
        {
            turnMotor.set(-moveSpeed);
            supportSoln.set(pullDownVal);
        }
        if (direction == 0)
        {
            if (!robot.InClimbMode() && supportSoln.get() == pullDownVal)
                turnMotor.set(holdSpeed); //Try to counteract gravity for an otherwise unsupported arm.
            else
                turnMotor.set(0.0); //This prevents the robot from tipping in climb mode
        }
    }

    @Override
    public void InitDashboard()
    {
        SmartDashboard.putNumber(kEncoder, 0.0);
        SmartDashboard.putBoolean(kEncoderReset, false);
        SmartDashboard.putNumber(kMoveSpeed, moveSpeed);
        SmartDashboard.putNumber(kHoldSpeed, holdSpeed);
        SmartDashboard.putBoolean(kMoveToTarget, false);
        SmartDashboard.putNumber(kTargetHeight, 0.0);
        SmartDashboard.putNumber(kEncoderBias, 0.0);
    }
    @Override
    public void SendToDashboard()
    {
        SmartDashboard.putNumber(kEncoder, encoder.getDistance());
    }
    @Override
    public void OnClimbingModeToggled() 
    {

    }
}