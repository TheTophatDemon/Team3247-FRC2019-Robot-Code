package frc.robot;

/**
 * Holds measurements of the playable field area in meters.
 * Relative to the back left corner of the playable field area from the robot's point of view,
 * with the positive X axis being forward from that and the positive Y axis being right.
*/
class Field
{ 
    /**Size of the ENTIRE playable field area (meters)*/
    public static final Vector2 size = new Vector2(16.4592, 8.2296);
    /**Size of a single team's half of the field area (meters)*/
    public static final Vector2 teamSectionSize = new Vector2(8.2296, 8.2296);
}