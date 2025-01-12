package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@Autonomous(name = "RightHook", group = "Sensor")
//@Disabled
public class RightHook extends LinearOpMode {
    // Create an instance of the sensor
    public SparkFunOTOS myOtos;
    DcMotor spin, spin2;
    MotorEx lbMotor, lfMotor, rbMotor, rfMotor;
    double speedtoggle = 0.5;
    double tarposx = -24; //reversed values
    double tarposy = 0.6;
    double powerfromposx = tarposx;
    double powerfromposy = tarposy;
    double xbuffer = 5; //prevbiousdly 15
    double ybuffer = 5;
    boolean xmoving = false;
    boolean ymoving = false;
    boolean allowprimx = true;
    boolean allowprimy = true;
    boolean hasrun = false;
    boolean shouldpull = false;
    boolean[] hasgonetoposition = new boolean[3];
    boolean doit = false;
    boolean hasactuallyran = false;
    double xreducer = 0.7;
    boolean xwithinrange = false;
    boolean ywithinrange = false;
    ExecutorService executor = Executors.newFixedThreadPool(2);
    Stopwatch[] stopwatches = new Stopwatch[9];
    @Override
    public void runOpMode() throws InterruptedException {
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "Lazor");
        lfMotor = new MotorEx(hardwareMap, "LF Motor");
        rfMotor = new MotorEx(hardwareMap, "RF Motor");
        lbMotor = new MotorEx(hardwareMap, "LB Motor");
        rbMotor = new MotorEx(hardwareMap, "RB Motor");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        lfMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        SparkFunOTOS Lazor = myOtos;
        Arrays.fill(stopwatches, Stopwatch.createUnstarted());
        executor.submit(this::moverobot);

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        //Do a thing
        moverobot();
        // Wait for the start button to be pressed
        waitForStart();

        // Loop until the OpMode ends
        while (opModeIsActive()) {
            if(!doit){
                Arrays.fill(hasgonetoposition, false);
                //Arrays.fill(stopwatches, Stopwatch.createUnstarted());
                spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spin.setTargetPosition(0);
                spin2.setTargetPosition(0);
                //stopwatches[0].start();
                doit = true;
            }
            MecanumDrive Drivetrain = new MecanumDrive(lfMotor, rfMotor, lbMotor, rbMotor);
            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Reset the tracking if the user requests it
            if (gamepad1.y) {
                myOtos.resetTracking();
            }

            // Re-calibrate the IMU if the user requests it
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }
            //drive
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double lfPower = drive + strafe + rotate;
            double rfPower = drive - strafe - rotate;
            double lbPower = drive - strafe + rotate;
            double rbPower = drive + strafe - rotate;
            if(gamepad1.left_bumper){
                speedtoggle = 1;
            }
            else{
                speedtoggle = 0.5;
            }
            lfMotor.motor.setPower(lfPower * speedtoggle);
            rfMotor.motor.setPower(rfPower * speedtoggle);
            lbMotor.motor.setPower(lbPower * speedtoggle);
            rbMotor.motor.setPower(rbPower * speedtoggle);
            //
            if(Math.abs(powerfromposx) > powerfromposx){
                xreducer *= -1;
            }
            //
            if(powerfromposx < -1){
                powerfromposx += 1;
            }
            if(powerfromposx > 1){
                powerfromposx -= 1;
            }
            if(powerfromposy < -1){
                powerfromposy += 1;
            }
            if(powerfromposy > 1){
                powerfromposy -= 1;
            }
            if(pos.x < tarposx - xbuffer || pos.x > tarposx + xbuffer || pos.y > tarposy - ybuffer || pos.y < tarposy + ybuffer || Math.abs(pos.x) + xbuffer < Math.abs(tarposx) || Math.abs(pos.x) - xbuffer > Math.abs(tarposx) || Math.abs(pos.y) + ybuffer < Math.abs(tarposy) || Math.abs(pos.y) - ybuffer > Math.abs(tarposy)){ //maybe implement math.abs instead of reversing <> operators // i did it

                //else{allowprimx = true;}
                /*if( Math.abs(pos.y) + ybuffer < Math.abs(tarposy) &&  allowprimy){  //wrong or
                    ymoving = true;
                    //Drivetrain.driveRobotCentric(powerfromposy, 0, 0);
                    lfMotor.motor.setPower(powerfromposy);
                    rfMotor.motor.setPower(powerfromposy);
                    lbMotor.motor.setPower(powerfromposy);
                    rbMotor.motor.setPower(powerfromposy);
                }
                else{ymoving = false;}
                if(Math.abs(pos.y) > Math.abs(tarposy) + 12 || Math.abs(pos.y) - ybuffer > Math.abs(tarposy)){
                    allowprimy = false;
                    lfMotor.motor.setPower(-powerfromposy);
                    rfMotor.motor.setPower(-powerfromposy);
                    lbMotor.motor.setPower(-powerfromposy);
                    rbMotor.motor.setPower(-powerfromposy);
                }*/
                /*if(Math.abs(pos.x) + xbuffer < Math.abs(tarposx) &&  allowprimx){
                    xmoving  = true;
                    if(!stopwatches[0].isRunning()){ stopwatches[0].start();}
                    //driveRobotCentric(0, powerfromposx, 0);
                    lfMotor.motor.setPower(powerfromposx);
                    rfMotor.motor.setPower(-powerfromposx);
                    lbMotor.motor.setPower(-powerfromposx);
                    rbMotor.motor.setPower(powerfromposx);
                }
                else {xmoving = false;}
                if(Math.abs(pos.x) - xbuffer > Math.abs(tarposx)){
                    allowprimx = false;
                    lfMotor.motor.setPower(-powerfromposx);
                    rfMotor.motor.setPower(powerfromposx);
                    lbMotor.motor.setPower(powerfromposx);
                    rbMotor.motor.setPower(-powerfromposx);
                }*/
                if(!stopwatches[0].isRunning()){stopwatches[0].start();}
                if(pos.x > tarposx - xbuffer && !xwithinrange){
                    lfMotor.motor.setPower(-0.5);
                    rfMotor.motor.setPower(0.5);
                    lbMotor.motor.setPower(0.5);
                    rbMotor.motor.setPower(-0.5);
                }
                if(pos.x < tarposx + xbuffer && !xwithinrange){
                    lfMotor.motor.setPower(0.5);
                    rfMotor.motor.setPower(-0.5);
                    lbMotor.motor.setPower(-0.5);
                    rbMotor.motor.setPower(0.5);
                }
                if(pos.x > tarposx - xbuffer && pos.x < tarposx + xbuffer){
                    xwithinrange = true;
                }
                else{xwithinrange = false;}
                if(pos.y > tarposy - ybuffer && !ywithinrange){
                    lfMotor.motor.setPower(-0.5);
                    rfMotor.motor.setPower(-0.5);
                    lbMotor.motor.setPower(-0.5);
                    rbMotor.motor.setPower(-0.5);
                }
                if(pos.y < tarposy + ybuffer && !ywithinrange){
                    lfMotor.motor.setPower(0.5);
                    rfMotor.motor.setPower(0.5);
                    lbMotor.motor.setPower(0.5);
                    rbMotor.motor.setPower(0.5);
                }
                if(pos.y < tarposy + ybuffer && pos.y > tarposy - ybuffer){
                    ywithinrange = true;
                }
                else{ywithinrange = false;}
                if(xwithinrange && ywithinrange){
                    lfMotor.motor.setPower(0);
                    rfMotor.motor.setPower(0);
                    lbMotor.motor.setPower(0);
                    rbMotor.motor.setPower(0);
                }
                //else{allowprimy = true;}
                /*if(!xmoving && !ymoving){
                    //Drivetrain.driveRobotCentric(0,0,0);
                    lfMotor.motor.setPower(0);
                    rfMotor.motor.setPower(0);
                    lbMotor.motor.setPower(0);
                    rbMotor.motor.setPower(0);
                    //speedtoggle = 0;
                }*/
            }
            if(stopwatches[0].isRunning()){ //multi-pos w interrupts based on guava stopwatch
                if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 3500 && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) < 5500){
                    if(pos.h < -2){
                        xbuffer = 8;
                        ybuffer = 8;
                        lbMotor.motor.setPower(-0.3);
                        lfMotor.motor.setPower(-0.3);
                        rbMotor.motor.setPower(0.3);
                        rfMotor.motor.setPower(0.3);
                    }
                    if(pos.h > 2){
                        xbuffer = 8;
                        ybuffer = 8;
                        lbMotor.motor.setPower(0.3);
                        lfMotor.motor.setPower(0.3);
                        rbMotor.motor.setPower(-0.3);
                        rfMotor.motor.setPower(-0.3);
                    }
                    if(pos.h < 2 && pos.h > -2){
                        xbuffer = 4;
                        ybuffer = 4;
                        lfMotor.motor.setPower(0);
                        rfMotor.motor.setPower(0);
                        lbMotor.motor.setPower(0);
                        rbMotor.motor.setPower(0);
                    }
                    spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spin.setPower(1);
                    spin2.setPower(1);
                    if(!stopwatches[1].isRunning() && !stopwatches[2].isRunning() && !stopwatches[4].isRunning() && !stopwatches[6].isRunning()){
                        //set first step in position
                        spin.setTargetPosition(-252);
                        stopwatches[4].start();
                    }
                    if(stopwatches[4].isRunning()){
                        //all other steps in the position
                        if(stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 300 && !hasgonetoposition[0]){
                            spin2.setTargetPosition(740);
                            hasgonetoposition[0] = true;
                        }
                        if(stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 800 && !hasgonetoposition[1]){
                            spin2.setTargetPosition(1524);
                            hasgonetoposition[1] = true;
                        }
                        if(shouldpull){
                            spin2.setTargetPosition(911);
                            hasgonetoposition[2] = true;
                        }
                    }
                    if(stopwatches[4].isRunning() && stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 15000){ // reset/sleep for positioning on upper rung for specimen
                        stopwatches[4].reset();
                    }
                }

                if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 5600 && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) < 8000){
                    tarposx = -25;
                    tarposy = 28;
                    allowprimx = true;
                    allowprimy = true;
                }
                if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 7200 && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) < 11000){
                    if(shouldpull){
                        spin2.setTargetPosition(1067);
                        hasgonetoposition[2] = true;
                    }
                    if(1 == 1){
                        shouldpull = true;
                    }
                }
                if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 14000 && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) < 17500){
                    tarposy = -1;
                    allowprimy = true;
                }
                if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 18500 /*&& stopwatches[0].elapsed(TimeUnit.MILLISECONDS) < 21000*/){
                    tarposx = 11;
                    allowprimx = true;
                }
                /*if(stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 26000){
                    spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //CHECK THIS
                }*/
            }

            /*Waypoint w1 = new StartWaypoint(-10, 0);
            Waypoint w2 = new GeneralWaypoint(-20, 0, 0.3,0.25, 3);
            Waypoint w3 = new EndWaypoint(-40, 0, 0 , 0.3, 0.25, 3, 2, 20);
            Path thispath = new Path(w1, w2, w3);
            thispath.init();
            //thispath.loop(pos.x, pos.y, pos.h);
            MecanumDrive Drivetrain = new MecanumDrive(lfMotor, rfMotor, lbMotor, rbMotor);



            while (!thispath.isFinished()) {
                pos = myOtos.getPosition();
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
                telemetry.update();
                if (thispath.timedOut())
                    throw new InterruptedException("Timed out");

                // return the motor speeds
                double speeds[] = thispath.loop(pos.x, pos.y,pos.h);


                Drivetrain.driveRobotCentric(speeds[0], speeds[1], speeds[2]);

            }*/
            //

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            // Update the telemetry on the driver station
            telemetry.update();
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    private void moverobot(){
        if(powerfromposx < -1){
            powerfromposx += 1;
        }
        if(powerfromposx > 1){
            powerfromposx -= 1;
        }
        if(powerfromposy < -1){
            powerfromposy += 1;
        }
        if(powerfromposy > 1){
            powerfromposy -= 1;
        }
    }
}