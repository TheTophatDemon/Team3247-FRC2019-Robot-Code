package frc.robot;

/**
 * Holds two doubles, x and y. Can be used as a coordinate point or as an actual mathematical vector.
 */
class Vector2 
{
    double x;
    double y;
    public Vector2(double x, double y)
    {
        this.x = x;
        this.y = y;
    }
    public void add(Vector2 other)
    {
        this.x += other.x;
        this.y += other.y;
    }
    public Vector2 addedTo(Vector2 other)
    {
        return new Vector2(this.x + other.x, this.y + other.y);
    }
    public void subtract(Vector2 other)
    {
        this.x -= other.x;
        this.y -= other.y;
    }
    public Vector2 subtractedBy(Vector2 other)
    {
        return new Vector2(this.x - other.x, this.y - other.y);
    }
    public void multiply(Vector2 other)
    {
        this.x *= other.x;
        this.y *= other.y;
    }
    public void multiply(double scalar)
    {
        this.x *= scalar;
        this.y *= scalar;
    }
    public Vector2 multipliedBy(Vector2 other)
    {
        return new Vector2(this.x * other.x, this.y * other.y);
    }
    public Vector2 multipliedBy(double scalar)
    {
        return new Vector2(this.x * scalar, this.y * scalar);
    }
    public void divide(Vector2 other)
    {
        this.x /= other.x;
        this.y /= other.y;
    }
    public void divide(double scalar)
    {
        this.x /= scalar;
        this.y /= scalar;
    }
    public Vector2 dividedBy(Vector2 other)
    {
        return new Vector2(this.x / other.x, this.y / other.y);
    }
    public Vector2 dividedBy(double scalar)
    {
        return new Vector2(this.x / scalar, this.y / scalar);
    }
    /**
     * The dot product is positive if two directional vectors are facing about the same direction.
     * It is zero if they are perpendicular, and negative if they are facing away from eachother.
     * For normalized vectors, it is one if they are pointing the same direction and negative one
     * if they are pointing in opposite directions.
     */
    public double dotProduct(Vector2 other)
    {
        return (this.x * other.x) + (this.y * other.y);
    }
    /**
     * Changes the length of the vector to 1, maintaining its direction.
     */
    public void normalize()
    {
        double len = length();
        x /= len;
        y /= len;
    }
    /**Returns a new vector with the same direction but with a length of 1 */
    public Vector2 normalized()
    {
        double len = length();
        return new Vector2(x / len, y / len);
    }
    /**
     * Returns the euclidean distance from (0,0) to (x,y).
     * If it's a directional vector, then it's the length of the vector.
     */
    public double length()
    {
        return Math.sqrt(x * x + y * y);
    }
}