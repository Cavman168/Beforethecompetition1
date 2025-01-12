package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumOdoKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.trajectory.constraint.MecanumDriveKinematicsConstraint;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@Autonomous
public class TrajectorEnhanced extends LinearOpMode {

    public static final double TRACKWIDTH = 0.3683;
    public static final double CENTER_WHEEL_OFFSET = 0.05715;
    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double TICKS_PER_REV = 537.7;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    // Define drive motors and encoders
    private MotorEx lbMotor, rbMotor, lfMotor, rfMotor;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private MecanumDrive driveTrain;

    // Define Stopwatches
    Stopwatch sleepcounter1 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();

    // Distance of each wheel from center
    Translation2d lfdisfromcen = new Translation2d(0.1651, 0.1524);
    Translation2d rfdisfromcen = new Translation2d(0.2413, 0.15875);
    Translation2d lbdisfromcen = new Translation2d(0.2413, 0.2159);
    Translation2d rbdisfromcen = new Translation2d(0.1651, 0.19812);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        lfMotor = new MotorEx(hardwareMap, "LF Motor");
        rfMotor = new MotorEx(hardwareMap, "RF Motor");
        lbMotor = new MotorEx(hardwareMap, "LB Motor");
        rbMotor = new MotorEx(hardwareMap, "RB Motor");

        lbMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new MecanumDrive(lfMotor, rfMotor, lbMotor, rbMotor);

        // Initialize odometry encoders and set direction
        leftOdometer = lbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = lfMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = rbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        // Initialize odometry
        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        // Set up trajectory configuration
        MecanumDriveKinematics kinematics = new MecanumDriveKinematics(lfdisfromcen, rfdisfromcen, lbdisfromcen, rbdisfromcen);
        MecanumDriveKinematicsConstraint mdcc = new MecanumDriveKinematicsConstraint(kinematics, 0.1);
        TrajectoryConfig config = new TrajectoryConfig(0.2, 0.2);


        Trajectory exampltrj = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                Arrays.asList(new Translation2d(5, 2), new Translation2d(5, 4)),
                new Pose2d(12, 0, new Rotation2d(0)),
                config
        );



        RamseteController controller = new RamseteController(DriveConstants.B, DriveConstants.ZETA);

        Trajectory.State goal = exampltrj.sample(10);

        waitForStart();
        sleepcounter1.start();
        while (opModeIsActive() && !isStopRequested()) {
            // Control loop for drivetrain
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            /*double lfPower = drive + strafe + rotate;
            double rfPower = drive - strafe - rotate;
            double lbPower = drive - strafe + rotate;
            double rbPower = drive + strafe - rotate;

            lfMotor.motor.setPower(lfPower);
            rfMotor.motor.setPower(rfPower);
            lbMotor.motor.setPower(lbPower);
            rbMotor.motor.setPower(rbPower);*/

            // Odometry tracking
            Pose2d currentRobotPose = odometry.getPose();
              // Sample trajectory at 1 seconds
            ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
            MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);


            telemetry.addData("csharp is shint", adjustedSpeeds);
            telemetry.addData("wheelie", wheelSpeeds);

            lfMotor.motor.setPower(wheelSpeeds.frontLeftMetersPerSecond);
            rfMotor.motor.setPower(wheelSpeeds.frontRightMetersPerSecond);
            lbMotor.motor.setPower(wheelSpeeds.rearLeftMetersPerSecond);
            rbMotor.motor.setPower(wheelSpeeds.rearRightMetersPerSecond);

            // Telemetry updates
            if (gamepad1.x) {
                if (!sleepcounter2.isRunning()) {
                    sleepcounter2.start();
                    telemetry.addData("posO", odometry.getPose());
                    telemetry.update();
                }
            }
            if (sleepcounter2.elapsed(TimeUnit.MILLISECONDS) > 300) {
                if (sleepcounter2.isRunning()) {
                    sleepcounter2.reset();
                }
            }

            if (gamepad1.a) {
                if (!sleepcounter1.isRunning()) {
                    telemetry.addData("posM", odometry.getPose());
                    telemetry.update();
                    sleepcounter1.start();
                }
            }
            if (sleepcounter1.elapsed(TimeUnit.MILLISECONDS) > 300) {
                if (sleepcounter1.isRunning()) {
                    sleepcounter1.reset();
                }
            }

            odometry.updatePose();
            telemetry.addData("pos", odometry.getPose());
            telemetry.update();
        }
    }
}
