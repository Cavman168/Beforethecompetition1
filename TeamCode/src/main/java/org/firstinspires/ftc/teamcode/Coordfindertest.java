package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
//@TeleOp(name="Coordfindertest", group="TeleOp")
public class Coordfindertest extends OpMode {

    // Declare motors
    DcMotor lbMotor, rbMotor, lfMotor, rfMotor;
    boolean spintoggle = false;
    long lastToggleTime = 0;  // Variable to keep track of time for debouncing
    short configtestcycle = 0;
    boolean configtesttoggle = false;
    float speedreducer = 0.5F; //reduce robot violence by dividing speeds by this
    double coordx;
    double coordy;
    double coordrelativex;
    double coordrelativey;
    boolean peanutbutter; //silly variable
    long ycounterfirst = 0;
    long xcounterfirst = 0;
    long ycounterfinish = 0;
    long xcounterfinish = 0;
    long ycounter = 0;
    long xcounter = 0;
    long curtime = System.currentTimeMillis();
    long start = System.currentTimeMillis();
    @Override
    public void init() {
        // Initialize motors
        initDrive();
    }

    @Override
    public void loop() {
        // Drive section
        driveRobot();
    }

    @Override
    public void stop() {
        // Stop the robot drive
        stopDriveMotors();
    }

    // Drive section methods
    private void initDrive() {
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");


        // Reverse left back motor
        lbMotor.setDirection(DcMotor.Direction.REVERSE);

        //create and set bool for config testing toggle


    }

    private void driveRobot() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        //VIOLENT Mode
        if(gamepad1.left_bumper){
            speedreducer = 1F; //remove limiter
        }
        else{
            speedreducer = 0.5F; //relimit
        }

        if(peanutbutter){

        }

        double lfPower = drive + strafe + rotate;
        double rfPower = drive - strafe - rotate;
        double lbPower = drive - strafe + rotate;
        double rbPower = drive + strafe - rotate;

        //HAMBURGER MODE
        String hamburger = "hamburger";
        if (gamepad1.b) {
            telemetry.addData("hamburger", hamburger);
        }
        //enable motor config testing
        if (gamepad1.a){
            if (configtesttoggle){
                configtesttoggle = false;
            }
            else{
                configtesttoggle = true;
            }
        }


        if (gamepad1.b){
            configtestcycle++;
            if (configtestcycle > 4){
                configtestcycle = 1;
            }
        }
        if (configtestcycle > 4){
            configtestcycle = 1;
        }

        //config testing
        if (configtesttoggle){
            if (configtestcycle == 1){
                lfPower = 1;
                lbPower = 0;
                rfPower = 0;
                rbPower = 0;
            }
            else{
                if(configtestcycle == 2){
                    lfPower = 0;
                    lbPower = 1;
                    rfPower = 0;
                    rbPower = 0;
                }//ive spent way too much time on this stupid code that shouldve took 5 secs
                else{
                    if(configtestcycle == 3){
                        lfPower = 0;
                        lbPower = 0;
                        rfPower = 1;
                        rbPower = 0;
                    }
                    else{
                        if(configtestcycle == 4){
                            lfPower = 0;
                            lbPower = 0;
                            rfPower = 0;
                            rbPower = 1;
                        }
                    }
                }
            }
        }

        if (spintoggle) {
            lfPower = 1;
            rfPower = 1;
            lbPower = 1;
            rbPower = 0.3;
        }

        // Set motor power
        lfMotor.setPower(lfPower);
        rfMotor.setPower(rfPower);
        lbMotor.setPower(lbPower);
        rbMotor.setPower(rbPower);



        //config testing

        telemetry.addData("Status", "Running");
        telemetry.addData("LF Power", lfPower);
        telemetry.addData("RF Power", rfPower);
        telemetry.addData("LB Power", lbPower);
        telemetry.addData("RB Power", rbPower);
        telemetry.addData("CTT", configtesttoggle);
        telemetry.addData("CTC", configtestcycle);
        telemetry.addData("xcounter", xcounter);
        telemetry.addData("x", coordx);

        telemetry.update();
        long finish = System.currentTimeMillis();
        long elapsedtime = finish - start;
        if (lbPower == 1){
            if (rbPower == 1){
                xcounterfirst = curtime;
                sleep(100);
                xcounterfinish = curtime;
                xcounter = xcounterfinish - xcounterfirst;
            }
        }
        coordx = xcounter * 0.001;
        //System.out.println(elapsedtime);
    }


    private void stopDriveMotors() {
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);
    }
}
