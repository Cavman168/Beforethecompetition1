package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.arcrobotics.ftclib.purepursuit.*;
import com.google.common.base.Stopwatch;

import java.util.concurrent.TimeUnit;


/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
//@Disabled
@Autonomous
public class DeadOdom extends LinearOpMode {

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
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    Stopwatch sleepcounter1 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();
    private MotorEx lbMotor, rbMotor, lfMotor, rfMotor;
    private MecanumDrive driveTrain;
    //private Motor intakeLeft, intakeRight, liftLeft, liftRight;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;



    @Override
    public void runOpMode() throws InterruptedException {
        lfMotor = new MotorEx(hardwareMap, "LF Motor");
        rfMotor = new MotorEx(hardwareMap, "RF Motor");
        lbMotor = new MotorEx(hardwareMap, "LB Motor");
        rbMotor = new MotorEx(hardwareMap, "RB Motor");
        DcMotor spin, spin2;
        int TGP = 0;
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        spin = spin2;
        spin.setTargetPosition(0);
        spin.setPower(1);
        spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lbMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        driveTrain = new MecanumDrive(lfMotor, rfMotor, lbMotor, rbMotor);

        /*intakeLeft = new Motor(hardwareMap, "intake_left");
        intakeRight = new Motor(hardwareMap, "intake_right");
        liftLeft = new Motor(hardwareMap, "lift_left");
        liftRight = new Motor(hardwareMap, "lift_right");*/

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
        leftOdometer = lbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = lfMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = rbMotor.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        //leftOdometer.setDirection(Motor.Direction.REVERSE);

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
        horizontalValue = () -> TICKS_PER_REV(centerOdometer::getDistance.getCurrentPosition());*/

        waitForStart();
        Waypoint O1 = new StartWaypoint(0,0);
        Waypoint O2 = new GeneralWaypoint(
                0, 3, 0.3,
                0.5, 8
        );
        Waypoint O3 = new EndWaypoint(
                0, 0, 0, 0.3,
                0.5, 8,
                12, 5
        );

        Waypoint p1 = new StartWaypoint(1, 1);
        Waypoint p2 = new GeneralWaypoint(
                4, 1.5, 0.3,
                0.5, 3
        );
        Waypoint p3 = new EndWaypoint(
                7, 2, 0, 0.3,
                0.8, 0.09,
                7, 3
        );
        Path o_path = new Path(O1, O2, O3);
        Path m_path = new Path(p1, p2, p3);
        m_path.init();
        o_path.init();

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

            if(gamepad1.dpad_up){
                TGP += 1;
            }
            if(gamepad1.dpad_down){
                TGP -= 1;
            }
            spin.setTargetPosition(TGP);
            //spin2.setPower(TGP);
            if (gamepad1.x){
                if(!sleepcounter2.isRunning()){
                    sleepcounter2.start();
                    o_path.followPath(driveTrain, odometry);
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
                    m_path.followPath(driveTrain, odometry);
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
            telemetry.addData("leftodom" ,leftOdometer.getDistance());
            telemetry.addData("centodom" ,centerOdometer.getDistance());
            telemetry.addData("rightodom" ,rightOdometer.getDistance());
            telemetry.addData("MAINodom", odometry);
            telemetry.addData("pos", odometry.getPose());
            telemetry.update();
        }

    }}