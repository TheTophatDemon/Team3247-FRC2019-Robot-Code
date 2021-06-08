package frc.robot;

class Line
{
    public Vector2 origin = new Vector2(0.0, 0.0);
    public Vector2 direction = new Vector2(0.0, 0.0);
    /**
     * Initializes a line with COPIES of the two vectors passed in
     * @param origin Starting point
     * @param direction A normalized direction vector
     */
    public Line(Vector2 origin, Vector2 direction)
    {
        //Copy values from the parameters to our vectors
        this.origin.x = origin.x;
        this.origin.y = origin.y;
        this.direction.x = direction.x;
        this.direction.y = direction.y;
        /*
        Doing "this.origin = origin", etc. would make it so that
        the Vector2 objects passed in as parameters could not be modified without
        changing the line as well, because the line's origin now refers to the object
        passed as a parameter.
        */
    }
    /**
     * 
     * @param ox Origin x
     * @param oy Origin y
     * @param dx Direction x
     * @param dy Direction y
     */
    public Line(double ox, double oy, double dx, double dy)
    {
        origin.x = ox;
        origin.y = oy;
        direction.x = dx;
        direction.y = dy;
    }
}