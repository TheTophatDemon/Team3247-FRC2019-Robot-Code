/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot 
{
  static final String kClimbing = "Is Climbing?";
  static final String kToggleClimb = "Toggle Climbing Mode";

  public Joystick controller = new Joystick(0);
  public Joystick controller2 = new Joystick(1);
  public DriveSystem driveSystem = new DriveSystem(this);
  public IntakeSystem intake = new IntakeSystem(this);
  public ArmController arm = new ArmController(this);
  public MastController mast = new MastController(this);
  public DeploySystem deploySystem = new DeploySystem(this);
  //Don't forget to add new subsystems to this array
  Subsystem[] subsystems = new Subsystem[]{
    driveSystem,
    intake,
    arm,
    mast,
    deploySystem
  };

  public VisionHandler visionHandler = new VisionHandler(this);
  public TeleopHandler teleop = new TeleopHandler(this);
  public AutonomousHandler autonomous = new AutonomousHandler(this);

  boolean climbingMode = false;

  /**Time in seconds since the robot started*/
  public double time;
  /**Number of seconds since the last robot update. Used in time-based calculations.*/
  public double deltaTime;
  long lastTime = System.nanoTime();

  @Override
  public void startCompetition() {
    try
    {
      super.startCompetition();
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }
  }

  @Override
  public void robotInit() 
  {
    controller.setXChannel(0);
    controller.setYChannel(1);
    controller.setZChannel(2);
    controller2.setXChannel(0);
    controller2.setYChannel(1);
    controller2.setZChannel(2);
    for (Subsystem s : subsystems)
    {
      s.Reset();
      s.InitDashboard();
    }
    teleop.Reset();
    teleop.InitDashboard();
    autonomous.Reset();
    autonomous.InitDashboard();
    SmartDashboard.putBoolean(kToggleClimb, false);
    SmartDashboard.putBoolean(kClimbing, false);
  }
  @Override
  public void robotPeriodic() 
  {
    //Calculate the number of seconds passed since the last time this method was called.
    long now = System.nanoTime();
    deltaTime = (now - lastTime) / 1000000000.0;
    lastTime = now;

    for (Subsystem s : subsystems)
    {
      s.Update(deltaTime);
      s.SendToDashboard();
    }
    if (DashboardUtils.IsButtonPressed(kToggleClimb))
    {
        ToggleClimbMode();
    }
    SmartDashboard.putBoolean(kClimbing, InClimbMode());
  }

  @Override
  public void teleopInit()
  {
    for (Subsystem s : subsystems)
    {
      s.Reset();
      s.InitDashboard(); //This is important! It resets buttons accidentally pressed with "enter" on disable.
    }
    teleop.Reset();
    teleop.InitDashboard();
  }
  @Override
  public void teleopPeriodic() 
  {
    teleop.Update(deltaTime);
    teleop.SendToDashboard();
  }

  @Override
  public void autonomousInit() 
  {
    for (Subsystem s : subsystems)
    {
      s.Reset();
      s.InitDashboard();
    }
    autonomous.Reset();
    autonomous.InitDashboard();
  }

  @Override
  public void autonomousPeriodic() 
  {
    autonomous.Update(deltaTime);
    autonomous.SendToDashboard();
  }

  @Override public void disabledInit() {}
  @Override public void testPeriodic() {}

  public boolean InClimbMode()
  {
    return climbingMode;
  }
  public void ToggleClimbMode()
  {
    climbingMode = !climbingMode;
    for (Subsystem s : subsystems)
    {
      s.OnClimbingModeToggled();
    }
  }
}
