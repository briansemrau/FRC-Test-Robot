package org.usfirst.frc.team4737.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

    /*
     * Global Constants
     */

    private static final double wheelbase = 23 + 7.0 / 16.0;

    /*
     * Sensors and Hardware
     */

    private AHRS navX;
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private CANTalon leftFrontTalon;
    private CANTalon leftRearTalon;
    private CANTalon rightFrontTalon;
    private CANTalon rightRearTalon;

    /*
     * Teleop Control
     */

    private RobotDrive robotDrive;
    private PIDController steerPID;

    private Joystick xboxCtrlr;

    /*
     * Autonomous Control
     */

    private PIDController leftPID;
    private PIDController rightPID;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public void robotInit() {
        System.out.println("hello ds, i'm a robot");

        // ############### Init sensors

        navX = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(0, 1, true);
        rightEncoder = new Encoder(2, 3, false);
        // Set encoders to units of feet
        double distancePerPulse = Math.PI * (4.0 / 12.0) / 360; // pi * d / (pulses per revolution)
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);
        
        initAutonPID();

        // ############### Init actuators

        leftFrontTalon = new CANTalon(11);
        leftRearTalon = new CANTalon(12);
        rightFrontTalon = new CANTalon(13);
        rightRearTalon = new CANTalon(14);

        robotDrive = new RobotDrive(leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon);

        // ############### Init Controls

        xboxCtrlr = new Joystick(0);

        // ############### LiveWindow

//        LiveWindow.addSensor("drive", "leftEncoder", leftEncoder);
//        LiveWindow.addSensor("drive", "rightEncoder", rightEncoder);
//
//        LiveWindow.setEnabled(true);
    }
    
    private void initAutonPID() {
    	double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double outputRange = 1;
        double tolerance = 0.0 / 12.0;

        // Left
        leftEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
        leftPID = new PIDController(kp, ki, kd, leftEncoder, output -> {
            if (!isAutonomous()) {
                leftPID.reset();
                rightPID.reset();
            }
            robotDrive.tankDrive(output, rightPID.get(), false);
        });
        leftPID.setOutputRange(-outputRange, outputRange);
        leftPID.setAbsoluteTolerance(tolerance);

        // Right
        rightEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
        rightPID = new PIDController(kp, ki, kd, rightEncoder, output -> {
            if (!isAutonomous()) {
                leftPID.reset();
                rightPID.reset();
            }
            robotDrive.tankDrive(leftPID.get(), output, false);
        });
        rightPID.setOutputRange(-outputRange, outputRange);
        rightPID.setAbsoluteTolerance(tolerance);
    }

    public void autonomousInit() {
        leftEncoder.reset();
        rightEncoder.reset();

        // Set distance to travel in feet
        leftPID.setSetpoint(0);
        rightPID.setSetpoint(0);

        leftPID.enable();
        rightPID.enable();
    }

    public void autonomousPeriodic() {
        // If a sequence of events is needed, control them here.
        // Otherwise, PIDs will run themselves

        SmartDashboard.putString("encoders", "" + leftEncoder.getDistance() + ":" + rightEncoder.getDistance());
        SmartDashboard.putData("leftPID", leftPID);
        SmartDashboard.putData("rightPID", rightPID);
    }

    public void teleopInit() {
        navX.setPIDSourceType(PIDSourceType.kRate);
        steerPID = new PIDController(0.00008, 0.0, 0.0, navX, output -> {
            if (!isOperatorControl()) steerPID.reset();
        });
    }

    public void teleopPeriodic() {
        double steer = xboxCtrlr.getRawAxis(0);
        double throttle = xboxCtrlr.getRawAxis(3) - xboxCtrlr.getRawAxis(2);

        boolean boost = xboxCtrlr.getRawButton(6);
        boolean drift = xboxCtrlr.getRawButton(5);

        if (Math.abs(steer) < 0.1) steer = 0;
        if (throttle < 0) steer = -steer;

        // TODO tune deadband
        if (boost || drift || Math.abs(throttle) > 0.1) {
            if (!steerPID.isEnabled())
                steerPID.enable();

            // TODO tune speed constants
            double driveSpeed = 0.5;
            double boostSpeed = 0.75;
            double boostConst = 0.25;
            double adjustedThrottle = throttle * (boost ? boostSpeed : driveSpeed) + (boost ? boostConst : 0);

            // TODO tune steering constants
            double defaultSteer = 200; // regular
            double boostSteer = 120; // wide
            double driftSteer = 300; // tight
            double adjustedSteer = (drift ? driftSteer : boost ? boostSteer : defaultSteer) * steer;

            steerPID.setSetpoint(adjustedSteer);

//            System.out.println(throttle + "   " + boost + "   " + adjustedThrottle);
            robotDrive.arcadeDrive(adjustedThrottle, -adjustedSteer / 360., false);
            xboxCtrlr.setRumble(RumbleType.kLeftRumble, (float) Math.abs(adjustedThrottle));
            xboxCtrlr.setRumble(RumbleType.kRightRumble, (float) Math.abs(adjustedThrottle));
        } else {
            // Disable steerPID while driving as to not accumulate the I term
            if (steerPID.isEnabled()) {
                steerPID.reset();
                steerPID.disable();
            }
            robotDrive.arcadeDrive(0, 0, false);
            xboxCtrlr.setRumble(RumbleType.kLeftRumble, 0);
            xboxCtrlr.setRumble(RumbleType.kRightRumble, 0);
        }

        SmartDashboard.putString("encoders", "" + leftEncoder.getDistance() + ":" + rightEncoder.getDistance());
    }

    public void testInit() {
    }

    public void testPeriodic() {
    }

    public void disabledInit() {
    }

    public void disabledPeriodic() {
    }
}
