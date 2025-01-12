package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

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

import java.util.concurrent.TimeUnit;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@Autonomous
public class IdiotOdom extends LinearOpMode {

    public static final double TRACKWIDTH = 0.3683;
    public static final double CENTER_WHEEL_OFFSET = 0.05715;
    public static final double WHEEL_DIAMETER = 0.1016;
    public static final double TICKS_PER_REV = 537.7;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    double inchespersecond = 36; //originally 79 tested 88.5 on new chasis tested 48 on 0.6 power

    // Define drive motors and encoders
    private MotorEx lbMotor, rbMotor, lfMotor, rfMotor;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;
    private MecanumDrive driveTrain;

    double tarx;
    double tary;
    double tardegs = 0; //may not be used
    Pose2d tarpos;
    double tariny = 5;
    double tarinx = -87;

    double procposx;
    double procposy;

    // Define Stopwatches
    Stopwatch sleepcounter1 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter3 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter4 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter5 = Stopwatch.createUnstarted();

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

        lfMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        /*MecanumDriveKinematics kinematics = new MecanumDriveKinematics(lfdisfromcen, rfdisfromcen, lbdisfromcen, rbdisfromcen);
        MecanumDriveKinematicsConstraint mdcc = new MecanumDriveKinematicsConstraint(kinematics, 0.1);
        TrajectoryConfig config = new TrajectoryConfig(0.2, 0.2);*/





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
            tarpos = new Pose2d(tarx, tary, Rotation2d.fromDegrees(tardegs));
            Pose2d currentRobotPose = odometry.getPose();

            tarx = gamepad1.right_stick_x * 100;
            tary = -gamepad1.right_stick_y * 100; //turn 0 - 1 values to usable coordinates



            if(gamepad1.x){
                if(!sleepcounter3.isRunning()){

                    procposy = abs(tariny) / inchespersecond;
                    procposy = abs(tariny) * 1000;
                    procposx = abs(tarinx) / inchespersecond;
                    procposx = abs(tarinx) * 1000;
                    sleepcounter3.start();
                }
            }
            if(sleepcounter3.isRunning()){
                if(tariny > 0){
                    lbMotor.motor.setPower(0.6);
                    rbMotor.motor.setPower(0.6);
                    lfMotor.motor.setPower(0.6);
                    rfMotor.motor.setPower(0.6);
                }
                if(tariny < 0){
                    lbMotor.motor.setPower(-0.6);
                    rbMotor.motor.setPower(-0.6);
                    lfMotor.motor.setPower(-0.6);
                    rfMotor.motor.setPower(-0.6);
                }
            }
            if(sleepcounter3.isRunning()){
                if(sleepcounter3.elapsed(TimeUnit.MILLISECONDS) >= procposy){
                    lbMotor.motor.setPower(0);
                    rbMotor.motor.setPower(0);
                    lfMotor.motor.setPower(0);
                    rfMotor.motor.setPower(0);
                    sleepcounter4.start();
                    sleepcounter3.stop();
                }
            }
            if(sleepcounter4.isRunning()){
                if(tarinx > 0){
                    lbMotor.motor.setPower(0.6);
                    rbMotor.motor.setPower(-0.6);
                    lfMotor.motor.setPower(-0.6);
                    rfMotor.motor.setPower(0.6);
                }
                if(tarinx < 0){
                    lbMotor.motor.setPower(-0.6);
                    rbMotor.motor.setPower(0.6);
                    lfMotor.motor.setPower(0.6);
                    rfMotor.motor.setPower(-0.6);
                }
            }
            if(sleepcounter4.isRunning()){
                if(sleepcounter4.elapsed(TimeUnit.MILLISECONDS) >= procposx){
                    lbMotor.motor.setPower(0);
                    rbMotor.motor.setPower(0);
                    lfMotor.motor.setPower(0);
                    rfMotor.motor.setPower(0);
                    sleepcounter4.stop();

                }
            }

            if(gamepad1.a){
                if(tarpos.getY() != currentRobotPose.getY()){ //if robot position not equal to target robot position set power to motors
                    lbMotor.motor.setPower(0.4);
                    rbMotor.motor.setPower(0.4);
                    lfMotor.motor.setPower(0.4);
                    rfMotor.motor.setPower(0.4);
                }
                if(tarpos == currentRobotPose){
                    lbMotor.motor.setPower(0);
                    rbMotor.motor.setPower(0);
                    lfMotor.motor.setPower(0);
                    rfMotor.motor.setPower(0);
                }
                else{
                    if(tarpos.getY() < currentRobotPose.getY()){
                        lbMotor.motor.setPower(0);
                        rbMotor.motor.setPower(0);
                        lfMotor.motor.setPower(0);
                        rfMotor.motor.setPower(0);
                    }
                }
            }
            if(gamepad1.dpad_up){
                if(!sleepcounter5.isRunning()){
                    inchespersecond++;
                    sleepcounter5.start();
                }
            }
            if(gamepad1.dpad_down){
                if(!sleepcounter5.isRunning()){
                    inchespersecond--;
                    sleepcounter5.start();
                }
            }
            if(sleepcounter5.isRunning()){
                if(sleepcounter5.elapsed(TimeUnit.MILLISECONDS) >= 150){
                    sleepcounter5.reset();
                }
            }
            //lines: point1<= or >= position


            //reminder to future, make interrupted actions and avoided perimeters!
            //add waypoints system for percise weaving



            /*lfMotor.motor.setPower();
            rfMotor.motor.setPower();
            lbMotor.motor.setPower();
            rbMotor.motor.setPower();*/

            // Telemetry updates
            telemetry.addData("timor3", sleepcounter3.elapsed(TimeUnit.MILLISECONDS));
            telemetry.addData("timor4", sleepcounter4.elapsed(TimeUnit.MILLISECONDS));
            telemetry.addData("tarinx",tarinx);
            telemetry.addData("tariny",tariny);
            telemetry.addData("tarpos", tarpos);
            telemetry.addData("lbmp", lbMotor.motor.getPower());
            telemetry.addData("rbmp", rbMotor.motor.getPower());
            telemetry.addData("rfmp", rfMotor.motor.getPower());
            telemetry.addData("lfmp", lfMotor.motor.getPower());
            telemetry.addData("inpersec", inchespersecond);
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
