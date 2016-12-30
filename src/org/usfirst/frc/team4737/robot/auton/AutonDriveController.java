package org.usfirst.frc.team4737.robot.auton;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4737.robot.Robot;

/**
 * @author Brian Semrau
 * @version 12/19/2016
 */
public class AutonDriveController {

    private static final double wheelbase = (23 + 7.0 / 16.0) / 12.0 / 2.0;
    private static final double maxAccel = 5; // TODO measure

    private Encoder left;
    private Encoder right;

    private PIDController leftPID;
    private PIDController rightPID;

    private double T, leftJerk, rightJerk;
    private double startTime;

    public AutonDriveController(Encoder left, Encoder right, RobotDrive drive) {
        this.left = left;
        this.right = right;

        leftPID = new PIDController(1.0, 0.1, 0.0, 0.0, left, output -> {
            // Check if disabled
            if (Robot.getInstance().isDisabled()) disable();

            // Update setpoint
            double setpoint = getTarget(time() - startTime, leftJerk);
            leftPID.setSetpoint(setpoint);

            // Submit outputs
            drive.tankDrive(output, rightPID.get());

            // Update graph :D
            SmartDashboard.putString("leftCurve", "" + setpoint + ":" + left.getDistance());
        });
        leftPID.setAbsoluteTolerance(0);
        SmartDashboard.putData("leftPID", leftPID);

        rightPID = new PIDController(1.0, 0.1, 0.0, 0.0, right, output -> {
            // Check if disabled
            if (Robot.getInstance().isDisabled()) disable();

            // Update setpoint
            double setpoint = getTarget(time() - startTime, rightJerk);
            rightPID.setSetpoint(setpoint);

            // Submit outputs
            drive.tankDrive(leftPID.get(), output);

            // Update graph
            SmartDashboard.putString("rightCurve", "" + setpoint + ":" + right.getDistance());
        });
        rightPID.setAbsoluteTolerance(0);
        SmartDashboard.putData("rightPID", rightPID);
    }

    public void goDistance(double distance, double angle) {
        /*
         * Creates the coefficients for the position curves of both the left and right side of the robot.
         * The function was derived geometrically and graphs of the position curves can be seen at:
         * https://www.desmos.com/calculator/ugxskdjr6g
         */

        // Calculate the left/right total distance from the parameters and distance between wheels
        double leftDistance = distance - 2.0 * Math.PI / 360.0 * angle * wheelbase / 2.0;
        double rightDistance = distance + 2.0 * Math.PI / 360.0 * angle * wheelbase / 2.0;

        // Set the period as the period of the curve that takes the longest time
        double TL = Math.sqrt(Math.abs(leftDistance / (2.0 * maxAccel)));
        double TR = Math.sqrt(Math.abs(rightDistance / (2.0 * maxAccel)));
        T = Math.max(TL, TR);

        // Calculate the jerk coefficients for the position curves
        leftJerk = leftDistance / (2 * T * T * T);
        rightJerk = rightDistance / (2 * T * T * T);

        // Reset and enable the PID controllers
        leftPID.reset();
        rightPID.reset();
        left.reset();
        right.reset();

        leftPID.setSetpoint(0);
        rightPID.setSetpoint(0);
        leftPID.enable();
        rightPID.enable();
        startTime = time();
    }

    public boolean onTarget() {
        return time() - startTime > 4 * T && leftPID.onTarget() && rightPID.onTarget();
    }

    public boolean isEnabled() {
        return leftPID.isEnabled();
    }

    public void disable() {
        leftPID.disable();
        rightPID.disable();
    }

    private double getTarget(double t, double m) {
        /*
         * This is a piecewise function of a triangular-acceleration curve over time.
         * The function is derived from a triple integration of jerk.
         * See https://www.desmos.com/calculator/s3ximk5vzu for fancy graphs of what this looks like.
         */

        // Huh, it sure would be fun to put this in one line. Not like it would be much less readable.
        if (t < 0) {
            return 0;
        } else if (t < T) {
            return m * t * t * t / 6;
        } else if (t < 3 * T) {
            return -m * t * t * t / 6 + m * T * t * t - m * T * T * t + m * T * T * T / 3;
        } else if (t < 4 * T) {
            return m * t * t * t / 6 - 2 * m * T * t * t + 8 * m * T * T * t - 26 * m * T * T * T / 3;
        } else if (t > 4 * T) {
            return m * 4 * T * 4 * T * 4 * T / 6 - 2 * m * T * 4 * T * 4 * T + 8 * m * T * T * 4 * T - 26 * m * T * T * T / 3;
        }
        return 0;
    }

    private double time() {
        // Current time in seconds
        return System.nanoTime() / 1000000000.0;
    }

}
