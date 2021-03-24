package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

/***********************************
Commands are Threads. This is because when
multiple actions must happen in sequence, it is sometimes helpful for each action to block
out the action for several seconds. We can't normally do that in TimedRobot because
everything has to happen every 20 milliseconds. However, if everything is happening in a Thread, 
then concurrency permits us to wait for something to finish in one thread without stopping the whole
program.

You are allowed to start Commands inside of Commands. That should let you break down a 
complex task into smaller ones.
************************************/

public abstract class Command extends Thread {
    private int period;
    private boolean verbose;
    private double startTime;
    
    public Command(String name, int periodInMillis, boolean verbose) {
        //All Threads have names by which they can be identified.
        super(name);
        //Period is how many milliseconds pass between calls of whileRunning().
        this.period = periodInMillis;
        //If verbose is set to true, the console will be flooded with messages.
        this.verbose = verbose;
    }
    public Command(String name, int periodInMillis) {
        //Call the other version of this overloaded constructor.
        //The default value of verbose is false.
        this(name, periodInMillis, false);
    }
    /*
     * The method called periodically (with the period specified by instance variable period)
     * while the command is still running.
     */
    protected abstract void whileRunning();
    /*
    * This is how the Command checks if it is finished. When this returns true, the program
    * reaches the end of the run() method, and then any Commands that were waiting for this
    * command (because of the join() method) will start.
    */
    protected abstract boolean isFinished();
    /*
    * This is where any nested Commands should be started by calling the .start() method.
    * Any other initialization code (setting initial PID setpoints, etc.) that doesn't belong
    * in the constructor should be place here.
    * If you override onStart(), it is highly recommended that you still call it inside the 
    * child class version of the method using super.onStart().
    */
    protected void onStart() {
        if(verbose) {
            System.out.println(getName() + "_STARTING*********");
        }
        startTime = Timer.getFPGATimestamp();
    }
    /*
    * Tie up any loose ends here. For instance, if a Command makes a mechanism move, then 
    * when the Command finishes, you may want to make the mechanism stop moving.
    * If you override onFinish(), it is highly recommended that you still call it inside the 
    * child class version of the method using super.onFinish().
    */
    protected void onFinish() {
        if(verbose) {
            System.out.println(getName() + "_FINISHED***********");
        }
    }
    /*
    * If the robot code functions correctly, this function will never be called. Usually,
    * we should be able to ignore it.
    */
    protected void onInterrupted() {
        System.out.println(getName() + " INTERRUPTED*********");
    }
    /*
    * Overrides the run() method of the parent class Thread. Do not call run() directly, 
    * even though it is public. Call start() instead, in order to start the life cycle of
    * the Thread.
    * It should never be necessary to override run() in child classes of Command.
    */
    @Override
    public void run() {
        onStart();
        while(!isFinished()) {
            whileRunning();
            if(verbose) {
                System.out.print(getName() + " state: ");
                System.out.println(getState());
            }
            try {
                Thread.sleep(period);
            }
            catch(InterruptedException e) {
                onInterrupted();
            }
        }
        onFinish();
    }
    public double getTime() {
        return Timer.getFPGATimestamp() - startTime;
    }
}
