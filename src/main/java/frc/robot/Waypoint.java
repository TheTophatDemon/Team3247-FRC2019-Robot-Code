package frc.robot;

import java.util.List;
import java.util.LinkedList;
import java.lang.Comparable;

/**A point in a WaypointGrid used for navigation in Autonomous. */
class Waypoint implements Comparable<Waypoint>
{
    public Vector2 position;
    public List<Waypoint> neighbors = new LinkedList<Waypoint>();
    public String targetName = "";
    public boolean blocked = false;
    //The following variables are used to keep track of the waypoint's state while calculating paths
    //They are modified manually in order to save calculations and should not be messed with otherwise.
    public Waypoint parent;
    public double totalCost;
    public double pathCost;
    public double goalCost;
    public double turnCost;
    public boolean onFrontier;
    public boolean explored;
    public Waypoint(double x, double y)
    {
        position = new Vector2(x, y);
    }
    public void Reset()
    {
        totalCost = 0.0;
        goalCost = 0.0;
        pathCost = 0.0;
        turnCost = 0.0;
        onFrontier = false;
        explored = false;
        parent = null;
    }
    @Override
    public int compareTo(Waypoint other)
    {
        int comparison = (int)Math.signum(totalCost - other.totalCost);
        return comparison;
    }
}