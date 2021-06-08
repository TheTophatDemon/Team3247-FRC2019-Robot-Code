package frc.robot;

/**
 * Represents an autonomous task in a sequence
 */
public interface AutoPhase
{
    /**
     * @return Returns true if task is complete.
     */
    public boolean Update(double deltaTime);
    /**
     * This is the PROPER way to end the phase prematurely.
     * Discard of resources and stop new threads here.
     */
    public void Close();
}