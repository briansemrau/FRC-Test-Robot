package org.usfirst.frc.team4737.robot.auton;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4737.robot.Robot;

/**
 * @author Brian Semrau
 * @version 12/19/2016
 */
public class AutonDriveController {

    private static final double hwheelbase = (23 + 7.0 / 16.0) / 12.0 / 2.0;
    private static final double maxAccel = 5; // TODO measure

    private Encoder left;
    private Encoder right;

    private PIDController leftPID;
    private PIDController rightPID;

    private double T, mL, mR;
    private double startTime;

    public AutonDriveController(Encoder left, Encoder right, RobotDrive drive) {
        this.left = left;
        this.right = right;

        leftPID = new PIDController(0.0, 0.0, 0.0, 0.0, left, output -> {
            // Check if disabled
            if (Robot.getInstance().isDisabled()) disable();

            // Update setpoint
            double setpoint = getTarget(time() - startTime, mL);
            leftPID.setSetpoint(setpoint);

            // Submit outputs
            drive.tankDrive(output, rightPID.get());

            // Update graph :D
            SmartDashboard.putString("leftCurve", "" + setpoint + ":" + left.getDistance());
        });
        SmartDashboard.putData("leftPID", leftPID);

        rightPID = new PIDController(0.0, 0.0, 0.0, 0.0, right, output -> {
            // Check if disabled
            if (Robot.getInstance().isDisabled()) disable();

            // Update setpoint
            double setpoint = getTarget(time() - startTime, mR);
            rightPID.setSetpoint(setpoint);

            // Submit outputs
            drive.tankDrive(leftPID.get(), output);

            // Update graph
            SmartDashboard.putString("rightCurve", "" + setpoint + ":" + right.getDistance());
        });
        SmartDashboard.putData("rightPID", rightPID);
    }

    public void goDistance(double d, double angle) {
        double dL = d - 2 * Math.PI / 360 * angle * hwheelbase;
        double dR = d + 2 * Math.PI / 360 * angle * hwheelbase;
        double TL = Math.sqrt(Math.abs(dL / (2 * maxAccel)));
        double TR = Math.sqrt(Math.abs(dR / (2 * maxAccel)));
        T = Math.max(TL, TR);
        double o = maxAccel / T;
        mL = angle >= 0 ? o : o * (d - Math.abs(2 * Math.PI / 360 * angle * hwheelbase)) / d;
        mR = angle <= 0 ? o : o * (d - Math.abs(2 * Math.PI / 360 * angle * hwheelbase)) / d;
        startTime = time();

        leftPID.reset();
        rightPID.reset();
        left.reset();
        right.reset();

        leftPID.setSetpoint(0);
        rightPID.setSetpoint(0);
        leftPID.enable();
        rightPID.enable();
    }

    public boolean finished() {
        return !isEnabled() || (time() - startTime > 4 * T && leftPID.onTarget() && rightPID.onTarget());
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
         * See www.desmos.com/calculator/s3ximk5vzu for fancy graphs of what this looks like.
         */
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
        return System.nanoTime() / 1000000000.0;
    }

}
