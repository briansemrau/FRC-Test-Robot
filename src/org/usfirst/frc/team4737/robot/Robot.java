package org.usfirst.frc.team4737.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team4737.robot.auton.AutonDriveController;
import org.usfirst.frc.team4737.robot.oi.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

    private static Robot instance;

    public static Robot getInstance() {
        return instance;
    }

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

    private XboxController xboxCtrlr;

    /*
     * Autonomous Control
     */

    private AutonDriveController autonDrive;

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    public void robotInit() {
        System.out.println("=====================\nTrebor the Test Robot\n=====================");
        instance = this;

        // ############### Init sensors

        navX = new AHRS(SPI.Port.kMXP);

        leftEncoder = new Encoder(0, 1, true);
        rightEncoder = new Encoder(2, 3, false);

        // Set encoders to units of feet
        double distancePerPulse = Math.PI * (4.0 / 12.0) / 360; // pi * d / (pulses per revolution)
        leftEncoder.setDistancePerPulse(distancePerPulse);
        rightEncoder.setDistancePerPulse(distancePerPulse);

        // ############### Init actuators

        leftFrontTalon = new CANTalon(11);
        leftRearTalon = new CANTalon(12);
        rightFrontTalon = new CANTalon(13);
        rightRearTalon = new CANTalon(14);

        robotDrive = new RobotDrive(leftFrontTalon, leftRearTalon, rightFrontTalon, rightRearTalon);

        // ############### Init Controls

        autonDrive = new AutonDriveController(leftEncoder, rightEncoder, robotDrive);

        xboxCtrlr = new XboxController(0);
        xboxCtrlr.LS.X.setDeadzone(-0.2, 0.2);
        xboxCtrlr.LS.Y.setDeadzone(-0.2, 0.2);
        xboxCtrlr.RS.X.setDeadzone(-0.2, 0.2);
        xboxCtrlr.RS.Y.setDeadzone(-0.2, 0.2);

    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
    }

    public void teleopInit() {
    }

    public void teleopPeriodic() {
        // TODO retune all of drive code

        double driveSpeed = 1.0;

        double defaultSteer = 0.75; // regular steer
        double driftSteer = 1;      // tight steer
        double adjustSteer = 0.3;   // very slow steer

        double steer = xboxCtrlr.LS.X.get();
        double throttle = xboxCtrlr.RT.get() - xboxCtrlr.LT.get();
        if (throttle < 0) steer = -steer;

        boolean powersteer = xboxCtrlr.LB.get();
        boolean regularDrive = throttle != 0 && !powersteer;
        boolean adjusting = throttle == 0 && steer != 0;

        boolean driving = regularDrive || powersteer || adjusting;

        if (driving) {
            autonDrive.disable();

            if (regularDrive) {
                robotDrive.arcadeDrive(throttle * driveSpeed, -steer * defaultSteer);
            } else if (powersteer) {
                robotDrive.arcadeDrive(throttle * driveSpeed, -steer * driftSteer);
            } else if (adjusting) {
                robotDrive.arcadeDrive(0, -steer * adjustSteer);
            }
        } else {
            if (xboxCtrlr.X.get()) {
                autonDrive.goDistance(5, 90);
            } else if (xboxCtrlr.Y.get()) {
                autonDrive.goDistance(-5, -90);
            }
        }

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
