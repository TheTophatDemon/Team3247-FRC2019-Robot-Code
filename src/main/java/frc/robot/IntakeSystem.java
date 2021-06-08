package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class IntakeSystem extends Subsystem
{
    static final String kIntakeSpeed = "Intake/Intake Speed";
    static final String kOuttakeSpeed = "Intake/Outtake Speed";
    static final String kSidewaysSpeed = "Intake/Sideways Speed";

    Talon intakeMotor = new Talon(2);
    Spark sidewaysMotor = new Spark(3);

    double intakeSpeed = 0.6;
    double outtakeSpeed = 1.0;
    double sidewaysSpeed = 1.0;

    public IntakeSystem(Robot robot)
    {
        super(robot);
    }
    @Override
    public void Reset()
    {
        
    }

    @Override
    public void Update(double deltaTime)
    {
        intakeSpeed = SmartDashboard.getNumber(kIntakeSpeed, intakeSpeed);
        outtakeSpeed = SmartDashboard.getNumber(kOuttakeSpeed, outtakeSpeed);
        sidewaysSpeed = SmartDashboard.getNumber(kSidewaysSpeed, sidewaysSpeed);
    }
    /**
     * Moves the ball intaker from side to side along the lead screw
     * @param speed A value from -1 to 1 that is multiplied by the motor speed set from the dashboard to get the final motor input.
     */
    public void AdjustSideways(double speed)
    {
        sidewaysMotor.set(speed * sidewaysSpeed);
    }
    /**
     * Operates the rotating cylinder on the ball intake device.
     * @param direction -1 for reverse (spit mode), 0 to stop, 1 for forward (suck mode)
     */
    public void Intake(int direction)
    {
        if (direction > 0)
        {
            intakeMotor.set(intakeSpeed);
        }
        else if (direction < 0)
        {
            intakeMotor.set(-outtakeSpeed);
        }
        else //direction == 0
        {
            intakeMotor.set(0.0);
        }
    }

    @Override
    public void InitDashboard()
    {
        SmartDashboard.putNumber(kIntakeSpeed, intakeSpeed);
        SmartDashboard.putNumber(kOuttakeSpeed, outtakeSpeed);
        SmartDashboard.putNumber(kSidewaysSpeed, sidewaysSpeed);
    }
    @Override
    public void SendToDashboard()
    {

    }
    @Override
    public void OnClimbingModeToggled() 
    {
        
    }
}