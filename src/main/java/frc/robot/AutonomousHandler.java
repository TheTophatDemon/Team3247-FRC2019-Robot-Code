package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.Filesystem;
/*import java.util.List;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDOutput;*/

//TODO: Hatch placement routine
//TODO: Navigation routine
class AutonomousHandler extends Subsystem
{
    static final String kOverrideToggle = "Toggle Teleop Override";
    static final String kOverriding = "Teleop Override?";

    DriveSystem drive;
    VisionHandler vision;

    boolean teleopOverride = true;

    //public WaypointLattice waypointLattice = new WaypointLattice();

    public AutonomousHandler(Robot robot)
    {
        super(robot);
        drive = robot.driveSystem;
        vision = robot.visionHandler;
    }
    @Override
    public void Reset()
    {
        //waypointLattice.LoadFromXMLFile(Filesystem.getDeployDirectory() + "/2019-field-waypoints.xml");
        //robot.deploySystem.BeginDeploymentSequence(true);
    }
    @Override
    public void Update(double deltaTime)
    {
        if (DashboardUtils.IsButtonPressed(kOverrideToggle))
        {
            teleopOverride = !teleopOverride;
        }
        if (teleopOverride)
        {
            robot.teleop.Update(deltaTime);
        }
    }

    @Override
    public void InitDashboard()
    {
        SmartDashboard.putBoolean(kOverrideToggle, false);
        SmartDashboard.putBoolean(kOverriding, false);
    }
    @Override
    public void SendToDashboard()
    {
        SmartDashboard.putBoolean(kOverriding, teleopOverride);
    }
    @Override
    public void OnClimbingModeToggled()
    {
        
    }
}