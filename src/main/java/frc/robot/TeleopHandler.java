package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class TeleopHandler extends Subsystem
{
    /*static final String kPresetMode = "Preset Mode";
    static final String kPresetModeToggle = "Toggle Preset Mode";*/
    static final String kMastControl = "Mast Movement";

    /**
     * Stores information about the preset heights for the
     * rocket's hatch and ball areas and serves as
     * the indicator for which presets to use
    */
    static enum PresetMode
    {
        Hatch(
            new double[] {0.0, 0.0, 1600.0}, 
            new double[] {1000.0, 4000.0, 5800.0},
            new int[] {11, 9, 7},
            "Hatch"
        ),
        Ball(
            new double[] {0.0, 0.0, 3400.0},
            new double[] {3300.0, 5800.0, 6000.0},
            new int[] {12, 10, 8},
            "Ball"
        );
        public double[] mastHeights;
        public double[] armHeights;
        public String[] dashboardKeys;
        public int[] buttons;
        PresetMode(double[] mastHeights, double[] armHeights, int[] buttons, String name)
        {
            this.mastHeights = mastHeights;
            this.armHeights = armHeights;
            this.buttons = buttons;
            dashboardKeys = new String[3];
            for (int i = 0; i < dashboardKeys.length; i++)
                dashboardKeys[i] = name + " " + String.valueOf(i + 1);
        }
    }
    
    Joystick controller;
    Joystick controller2;
    
    boolean deployed;

    public TeleopHandler(Robot robot)
    {
        super(robot);
        controller = robot.controller;
        controller2 = robot.controller2;
    }
    @Override
    public void Reset()
    {
        deployed = false;
    }

    @Override
    public void Update(double deltaTime)
    {
        HandlePresets();
        ControlIntake();
        ControlArm();
        ControlMast();

        if (controller2.getRawButtonPressed(1))
        {
            robot.arm.CancelAutomatedMovement();
            robot.mast.CancelAutomatedMovement();
        }
        else if (controller2.getRawButtonPressed(2))
        {
            robot.ToggleClimbMode();
        }
        if (Math.abs(controller2.getZ()) > 0.5 && !deployed)
        {
            deployed = true;
            robot.deploySystem.BeginDeploymentSequence(false);
        }
        
        
        robot.driveSystem.TeleopDrive(controller.getY(), controller.getZ());
    }

    /**
     * Sets the arm and the mast to go to the given level of the rocket
     * @param presetMode Specifies whether the desired level is for balls or hatches
     */
    void ExecutePreset(PresetMode presetMode, int level)
    {
        robot.arm.SetTargetPosition(presetMode.armHeights[level]);
        robot.mast.SetTargetHeight(presetMode.mastHeights[level]);
    }

    void HandlePresets()
    {
        for (PresetMode pm : PresetMode.values())
        {
            for (int i = 0; i < pm.dashboardKeys.length; i++)
            {
                if (DashboardUtils.IsButtonPressed(pm.dashboardKeys[i])
                    || controller2.getRawButtonPressed(pm.buttons[i]))
                {
                    ExecutePreset(pm, i);
                    return;
                }
            }
        }
    }

    void ControlMast()
    {
        if (controller.getRawButton(5) ||
            SmartDashboard.getNumber(kMastControl, 0) >= 0.5 ||
            controller2.getY() < -0.25)
        {
            robot.mast.CancelAutomatedMovement();
            robot.mast.SetMovement(1);
        }
        else if (controller.getRawButton(3) ||
            SmartDashboard.getNumber(kMastControl, 0) <= -0.5 ||
            controller2.getY() > 0.25)
        {
            robot.mast.CancelAutomatedMovement();
            robot.mast.SetMovement(-1);
        }
        else
        {
            robot.mast.SetMovement(0);
        }
        SmartDashboard.putNumber(kMastControl, 0.0); //As soon as you let go of the mouse, the slider will reset
    }

    void ControlArm()
    {
        //System.out.println(controller.getPOV(0));
        switch(controller.getPOV(0))
        {
            case 0:
            case 45:
            case 315:
                robot.arm.CancelAutomatedMovement();
                robot.arm.SetMovement(1);
                break;
            case 180:
            case 135:
            case 225:
                robot.arm.CancelAutomatedMovement();
                robot.arm.SetMovement(-1);
                break;
            default:
                if (!robot.arm.IsAutoMoving()) robot.arm.SetMovement(0);
                break;
        }
    }

    void ControlIntake()
    {
        switch (controller.getPOV(0))
        {
            case 90:
                robot.intake.AdjustSideways(1.0);
                break;
            case 270:
                robot.intake.AdjustSideways(-1.0);
                break;
            default:
                robot.intake.AdjustSideways(0.0);
                break;
        }
        if (controller.getRawButton(4))
        {
            robot.intake.Intake(1);
        }
        else if (controller.getRawButton(6))
        {
            robot.intake.Intake(-1);
        }
        else
        {
            robot.intake.Intake(0);
        }
    }

    @Override
    public void OnClimbingModeToggled()
    {
        
    }
    @Override
    public void InitDashboard()
    {
        for (PresetMode pm : PresetMode.values())
        {
            for (int i = 0; i < pm.dashboardKeys.length; i++)
            {
                SmartDashboard.putBoolean(pm.dashboardKeys[i], false);
            }
        }
        SmartDashboard.putNumber(kMastControl, 0);
    }
    @Override
    public void SendToDashboard()
    {

    }
}