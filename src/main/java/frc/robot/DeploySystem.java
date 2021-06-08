package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**For deploying the ball catcher from its starting position at the start of the match. */
class DeploySystem extends Subsystem
{
    static final String kDeploy = "Deploy Ball Catcher";
    static final String kFlipDeploy = "Flip When Deploying?";

    DoubleSolenoid solenoid = new DoubleSolenoid(4, 5);
    static final DoubleSolenoid.Value retracted = DoubleSolenoid.Value.kForward;
    static final DoubleSolenoid.Value expanded = DoubleSolenoid.Value.kReverse;
    double retractTimer;
    boolean flipOut;

    /**
     * Number of seconds since deployment sequence started.
     * If -1, it's not deploying.
     */
    double deployTimer;
    
    public DeploySystem(Robot robot)
    {
        super(robot);
        solenoid.set(retracted);
    }
    @Override
    public void Reset()
    {
        deployTimer = -1;
        retractTimer = 0.0;
        flipOut = false;
    }
    @Override
    public void Update(double deltaTime)
    {
        if (DashboardUtils.IsButtonPressed(kDeploy))
        {
            BeginDeploymentSequence(SmartDashboard.getBoolean(kFlipDeploy, false));
        }
        if (deployTimer > -1)
        {
            deployTimer += deltaTime;
            if (robot.mast.GetHeight() >= robot.mast.GetTargetHeight())
            {
                //System.out.println("Step 2!");
                robot.mast.SetMovement(0);
                robot.arm.SetTargetPosition(600.0);
                if (robot.arm.GetPosition() >= robot.arm.GetTargetPosition())
                {
                   // System.out.println("Step 3!");
                    robot.arm.CancelAutomatedMovement();
                    robot.arm.SetMovement(0);
                    robot.mast.SetTargetHeight(0.0);
                    DeployCatcher();
                    deployTimer = -1;
                }
            }
        }
        if (retractTimer > 0.0)
        {
            retractTimer -= deltaTime;
            if (retractTimer <= 0.0)
            {
                retractTimer = 0.0;
                solenoid.set(retracted);
            }
        }
    }
    /**
     * Brings out the ball catcher via solenoid. It must be clear of the bumpers first!
     */
    public void DeployCatcher()
    {
        solenoid.set(expanded);
        retractTimer = 1.5; //Will retract in one and a half seconds
    }
    /**
     * Performs an automatic deployment routine, raising the mast and the arm before deploying the catcher.
     * @param flipOut True to have the ball catcher flip out during deployment
     */
    public void BeginDeploymentSequence(boolean flipOut)
    {
        if (!IsDeploying())
        {
            deployTimer = 0.0;
            robot.mast.SetTargetHeight(400.0);
            this.flipOut = flipOut;
        }
        else
        {
            System.out.println("Already deploying!");
        }
    }
    /**
     * @return True if deployment sequence is underway
     */
    public boolean IsDeploying()
    {
        return deployTimer > -1.0;
    }
    @Override
    public void InitDashboard()
    {
        SmartDashboard.putBoolean(kDeploy, false);
        SmartDashboard.putBoolean(kFlipDeploy, false);
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