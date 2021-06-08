package frc.robot;

import edu.wpi.first.wpilibj.VictorSP;

import edu.wpi.first.wpilibj.Encoder;   
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class MastController extends Subsystem
{
    static final String kMastThrottle = "Mast/Throttle";
    static final String kMastHoldSpeed = "Mast/Hold Speed";
    static final String kEncoder = "Mast/Encoder Value";
    static final String kEncoderBias = "Mast/Encoder Bias";
    static final String kResetEncoder = "Mast/Reset Encoder";
    static final String kTargetHeight = "Mast/Target Height";
    static final String kMoveToTarget = "Mast/Move To Target";
    
    /**The number of encoder ticks at the mast's highest allowable position. */
    public static final int ENCODER_RANGE = 3400;
    public static final int TICKS_PER_INCH = 163;

    VictorSP mastMotor = new VictorSP(5); 
    Encoder encoder = new Encoder(4, 5);
    
    double throttle = 1.0;
    double mastHoldSpeed = 0.15;
    boolean moveToTarget = false;
    double targetHeight = 0.0;
    double lastHeight = 0.0;

    public MastController(Robot robot)
    {
        super(robot);
        encoder.reset();
    }
    @Override
    public void InitDashboard()
    {
        SmartDashboard.putNumber(kMastThrottle, throttle);
        SmartDashboard.putNumber(kMastHoldSpeed, mastHoldSpeed);
        SmartDashboard.putNumber(kEncoder, 0.0);
        SmartDashboard.putNumber(kEncoderBias, 0.0);
        SmartDashboard.putBoolean(kResetEncoder, false);
        SmartDashboard.putBoolean(kMoveToTarget, false);
        SmartDashboard.putNumber(kTargetHeight, 0.0);
    }
    @Override
    public void SendToDashboard()
    {
        SmartDashboard.putNumber(kEncoder, encoder.getDistance());
    }
    @Override
    public void Reset()
    {
        CancelAutomatedMovement();
    }
    /**
     * Sets the direction of the mast's motion.
     * If state is not 0, all automated movements are canceled.
     * If the mast exceeds its maximum height, its movement is automatically set to 0.
     * @param state -1 for down, 0 to stay still, 1 to go up
     */
    public void SetMovement(int state)
    {
        if (state > 0)
        {
            double heightLimit = ENCODER_RANGE;
            //While climbing, the mast cannot be allowed to exceed the top of the bot.
            if (robot.InClimbMode()) heightLimit = 0.0;
            if (GetHeight() < heightLimit) 
            {
                mastMotor.set(throttle);
            }
            else
            {
                mastMotor.set(mastHoldSpeed);
            }
        }
        else if (state < 0)
        {
            mastMotor.set(-throttle);
        }
        else //state == 0
        {
            //System.out.println(GetPosition());
            if (Math.abs(GetHeight()) < 0.1)
            {
                //When the mast is close enough to its original position,
                //Switch off the motor completely to save battery.
                mastMotor.set(0.0);
            }
            else if (GetHeight() > 0.0) 
            {            
                //If above the bot, drive slightly upwards to counteract gravity and stay still.
                mastMotor.set(mastHoldSpeed);
            }
            else
            {
                //If below the bot, push downwards to maintain height.
                mastMotor.set(-mastHoldSpeed);
            }
        }
    }
    
    /**
     * Tells the mast to automatically move to a certain height.
     * @param height Height to travel to in encoder ticks (NO LONGER A FRACTION).
     */
    public void SetTargetHeight(double height)
    {
        moveToTarget = true;
        targetHeight = height;
        lastHeight = GetHeight();
    }
    public double GetTargetHeight()
    {
        return targetHeight;
    }
    public void CancelAutomatedMovement()
    {
        moveToTarget = false;
    }
    /**
     * @return The height in encoder ticks, accounting for bias.
     * It is negative if the mast is below the robot.
    */
    public double GetHeight()
    {
        return encoder.getDistance() + SmartDashboard.getNumber(kEncoderBias, 0.0) * TICKS_PER_INCH;
    }

    @Override
    public void Update(double deltaTime)
    {
        throttle = SmartDashboard.getNumber(kMastThrottle, throttle);
        mastHoldSpeed = SmartDashboard.getNumber(kMastHoldSpeed, mastHoldSpeed);

        if (moveToTarget)
        {
            double difference = GetHeight() - targetHeight;
            if (difference > 0.0)
            {
                mastMotor.set(-throttle);
            }
            else if (difference < 0.0)
            {
                mastMotor.set(throttle);
            }
            else
            {
                mastMotor.set(0.0);
                moveToTarget = false;
            }
            //If it has already passed the target point, just go ahead and stop.
            double lastDifference = lastHeight - targetHeight;
            if (Math.signum(lastDifference) == -Math.signum(difference))
            {
                mastMotor.set(0.0);
                moveToTarget = false;
            }
            lastHeight = GetHeight();
        }

        if (DashboardUtils.IsButtonPressed(kMoveToTarget))
        {
            SetTargetHeight(
                //The dashboard value represents a fraction of the maximum height
                SmartDashboard.getNumber(kTargetHeight, 0.0) * ENCODER_RANGE 
            );
        }
        if (DashboardUtils.IsButtonPressed(kResetEncoder))
        {
            encoder.reset();
        }
    }

    @Override
    public void OnClimbingModeToggled()
    {
        if (!robot.InClimbMode())
        {
            if (GetHeight() < 0.0)
            {
                SetTargetHeight(0.0);
            }
        }
    }
}
