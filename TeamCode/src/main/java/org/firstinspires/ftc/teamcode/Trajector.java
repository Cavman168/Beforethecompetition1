/*package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumOdoKinematics;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.arcrobotics.ftclib.purepursuit.*;
import com.google.common.base.Stopwatch;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;


/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 *//*
@Disabled
//@Autonomous
public class Trajector extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 14.5;

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = 2.25;

    public static final double WHEEL_DIAMETER = 4.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    Stopwatch sleepcounter1 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();
    private MotorEx lbMotor, rbMotor, lfMotor, rfMotor;
    private MecanumDrive driveTrain;
    //private Motor intakeLeft, intakeRight, liftLeft, liftRight;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    Translation2d lfdisfromcen = new Translation2d(0.1651, 0.1524);
    Translation2d rfdisfromcen = new Translation2d(0.2413, 0.15875);
    Translation2d lbdisfromcen = new Translation2d(0.2413, 0.2159);
    Translation2d rbdisfromcen = new Translation2d(0.1651, 0.19812);


    @Override
    public void runOpMode() throws InterruptedException {
        lfMotor = new MotorEx(hardwareMap, "LF Motor");
        rfMotor = new MotorEx(hardwareMap, "RF Motor");
        lbMotor = new MotorEx(hardwareMap, "LB Motor");
        rbMotor = new MotorEx(hardwareMap, "RB Motor");

        lbMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new MecanumDrive(lfMotor, rfMotor, lbMotor, rbMotor);

        /*intakeLeft = new Motor(hardwareMap, "intake_left");
        intakeRight = new Motor(hardwareMap, "intake_right");
        liftLeft = new Motor(hardwareMap, "lift_left");
        liftRight = new Motor(hardwareMap, "lift_right");*//*

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = lbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = lfMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = rbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        //probably useless
        /*DoubleSupplier leftValue, rightValue, horizontalValue;

        leftValue = () -> TICKS_PER_REV(leftOdometer::getDistance.getCurrentPosition());
        rightValue = () -> TICKS_PER_REV(rightOdometer::getDistance.getCurrentPosition());
        horizontalValue = () -> TICKS_PER_REV(centerOdometer::getDistance.getCurrentPosition());*//*

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double lfPower = drive + strafe + rotate;
            double rfPower = drive - strafe - rotate;
            double lbPower = drive - strafe + rotate;
            double rbPower = drive + strafe - rotate;

            lfMotor.motor.setPower(lfPower);
            rfMotor.motor.setPower(rfPower);
            lbMotor.motor.setPower(lbPower);
            rbMotor.motor.setPower(rbPower);

            Pose2d currentRobotPose = odometry.getPose();

            //MecanumDriveKinematics kinematics = new MecanumDriveKinematics(0.254, 0.3048, 0.3048, 0.3302);

            TrajectoryConfig config = new TrajectoryConfig(DriveConstants.MAX_VELOCITY, DriveConstants.MAX_ACCELERATION);
            MecanumOdoKinematics kinematicsOdo = new MecanumOdoKinematics(lfdisfromcen, rfdisfromcen, lbdisfromcen, rbdisfromcen, 2, 2);

            Trajectory exampltrj = TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    Arrays.asList(new Translation2d(1, 1), new Translation2d(2, -1)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(3, 0, new Rotation2d(0)),
                    // Pass config
                    config
            );

            MecanumDriveKinematics kinematics = new MecanumDriveKinematics(lfdisfromcen, rfdisfromcen, lbdisfromcen, rbdisfromcen);
            RamseteController controller = new RamseteController(DriveConstants.B, DriveConstants.ZETA){
                Trajectory.State goal = exampltrj.sample(5);
                ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
                MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
            };



            if (gamepad1.x){
                if(!sleepcounter2.isRunning()){
                    sleepcounter2.start();

                    telemetry.addData("posO", odometry.getPose());
                    telemetry.update();
                }
            }
            if(sleepcounter2.elapsed(TimeUnit.MILLISECONDS) > 300){
                if(sleepcounter2.isRunning()){
                    sleepcounter2.reset();
                }
            }

            if(gamepad1.a){
                if(!sleepcounter1.isRunning()){

                    telemetry.addData("posM", odometry.getPose());
                    telemetry.update();
                    sleepcounter1.start();
                }
            }
            if(sleepcounter1.elapsed(TimeUnit.MILLISECONDS) > 300){
                if(sleepcounter1.isRunning()){
                    sleepcounter1.reset();
                }
            }


            odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            odometry.updatePose(); // update the position
            telemetry.addData("pos", odometry.getPose());
            telemetry.update();
        }

    }}*/