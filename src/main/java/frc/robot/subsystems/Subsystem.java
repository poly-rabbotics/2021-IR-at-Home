package frc.robot.subsystems;

public abstract class Subsystem {
    private boolean locked;
    public abstract void run();
    public abstract void reset();
    public void lock() {
        locked = true;
    }
    public void unlock() {
        locked = false;
    }
    public boolean getLocked() {
        return locked;
    }
}