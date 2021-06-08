package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains miscellaneous functions for handling the dashboard
 */
class DashboardUtils
{
    /**
     * Treats a SmartDashboard boolean like a push button, returning true the moment it's true
     * and then setting it to false immediately afterwards.
     */
    public static boolean IsButtonPressed(String key)
    {
        if (SmartDashboard.getBoolean(key, false) == true)
        {
            SmartDashboard.putBoolean(key, false); //Switch the button off when click is registered
            return true;
        }
        return false;
    } 
}