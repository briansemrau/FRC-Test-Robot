package org.usfirst.frc.team4737.robot.command;

/**
 * A command task that may be linked in a web to other commands based on success and failure conditions.
 *
 * @author Brian Semrau
 * @version 11/23/2016
 */
public abstract class Command {

    private Command successLink;
    private Command failureLink;

    private boolean successMet;
    private boolean failureMet;

    private boolean halts;

    /**
     * @param haltWhenConditionMet Whether or not the command should continue to function after a success or failure
     *                             condition has been met.<br> For example, a command to hold an arm steady may wish to
     *                             continue to function after a success condition is met, but a ball shooting mechanism
     *                             should halt after the success or failure condition is met.<br><br> <b>Note:</b> If
     *                             there is a command linked and it begins to function, this command will never execute
     *                             again. If two commands should run concurrently, use a CommandGroup.
     */
    public Command(boolean haltWhenConditionMet) {
        this.halts = haltWhenConditionMet;
    }

    /**
     * Attaches a command to take control if/when the active command meets the success condition.
     *
     * @param command
     */
    public final void linkSuccess(Command command) {
        this.successLink = command;
    }

    /**
     * Attaches a command to take control if/when the active command meets the failure condition.
     *
     * @param command
     */
    public final void linkFailure(Command command) {
        this.failureLink = command;
    }

    /**
     * Attaches a command to take control if/when the active command meets any condition.
     *
     * @param command
     */
    public final void linkCommand(Command command) {
        this.successLink = this.failureLink = command;
    }

    /**
     * Iterative run function for the command.
     *
     * @param dt The time passed since the last update
     */
    public final void update(double dt) {
        // If we ever succeed, only link to the next command
        if (successMet && successLink != null) {
            successLink.update(dt);
            return;
        }
        // If we ever fail, only link to the next command
        if (failureMet && failureLink != null) {
            failureLink.update(dt);
            return;
        }

        // If program isn't halted, update and recheck conditions
        if (!(halts || (successMet && failureMet))) {
            update_(dt);

            failureMet = failed();
            successMet = succeeded() && !failureMet;
        }// else {
        // do nothing, command is halted
        //}
    }

    /**
     * The update loop within the command. If the command halts or continues to a linked command, this method will never
     * be called again.
     *
     * @param dt The time passed since the previous update.
     */
    protected abstract void update_(double dt);

    /**
     * @return Returns true if the success condition is actively being met.
     */
    protected abstract boolean succeeded();

    /**
     * @return Returns true if the failure condition is actively being met.
     */
    protected abstract boolean failed();

}
