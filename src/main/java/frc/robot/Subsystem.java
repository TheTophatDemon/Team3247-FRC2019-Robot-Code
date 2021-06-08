package frc.robot;

abstract class Subsystem
{
    protected Robot robot;
    public Subsystem(Robot robot)
    {
        this.robot = robot;
    }
    /**Reset all state between robot modes */
    public abstract void Reset();
    /**Send all of your keys to the dashboard here so that they show up in Shuffleboard's NetworkTables tab */
    public abstract void InitDashboard();
    /**
     * Update the dashboard with information from the subsystem here.
     * You don't have to respond to Dashboard input from here.
    */
    public abstract void SendToDashboard();
    /**Called repeatedly during robotPeriodic() */
    public abstract void Update(double deltaTime);
    /**
     * A callback for when the robot switches to/from climbing mode.
     */
    public abstract void OnClimbingModeToggled();
}