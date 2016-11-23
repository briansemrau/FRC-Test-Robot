package org.usfirst.frc.team4737.robot.command;

/**
 * A group of commands that function concurrently. This only meets the success condition if all commands in the group
 * do, and fails if any of the commands in the group fail.
 *
 * @author Brian Semrau
 * @version 11/23/2016
 */
public class CommandGroup extends Command {

    private Command[] commands;

    public CommandGroup(Command... commands) {
        super(false);
        this.commands = commands;
    }

    @Override
    protected void update_(double dt) {
        for (Command c : commands) {
            c.update(dt);
        }
    }

    /**
     * @return Returns true if all of the commands in the group currently meet the success condition.
     */
    @Override
    protected boolean succeeded() {
        for (Command c : commands)
            if (!c.succeeded()) return false;
        return true;
    }

    /**
     * @return Returns true if any commands in the group fail.
     */
    @Override
    protected boolean failed() {
        for (Command c : commands)
            if (c.failed()) return true;
        return false;
    }

}
