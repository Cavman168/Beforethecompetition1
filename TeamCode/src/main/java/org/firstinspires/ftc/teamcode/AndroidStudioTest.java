package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(name="AndroidStudioTest", group="TeleOp")
public class AndroidStudioTest extends OpMode {

    // Declare motors
    DcMotor lbMotor, rbMotor, lfMotor, rfMotor, spin, spin2;
    Servo vaccum;
    boolean spintoggle = false;
    long lastToggleTime = 0;  // Variable to keep track of time for debouncing
    short configtestcycle = 0;
    boolean configtesttoggle = false;
    float speedreducer = 0.5F; //reduce robot violence by dividing speeds by this
    int TGP; //Target Position
    Stopwatch sleepcounter1 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();
    String placeholder = " ";
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
        //vaccum = hardwareMap.get(Servo.class, "vaccum");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");


        // Reverse left back motor
        //lbMotor.setDirection(DcMotor.Direction.REVERSE);
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        //spin2.setDirection(DcMotorSimple.Direction.REVERSE);
        spin.setTargetPosition(0);
        spin2.setTargetPosition(0);
        //spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //spin.setPower(1);
        //spin2.setPower(1);


        //set default positions

        //vaccum.setPosition(0);
        TGP = 0;



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

        double lfPower = drive + strafe + rotate * speedreducer;
        double rfPower = drive - strafe - rotate * speedreducer;
        double lbPower = drive - strafe + rotate * speedreducer;
        double rbPower = drive + strafe - rotate * speedreducer;

        //HAMBURGER MODE
        String hamburger = "hamburger";
        if (gamepad1.b) {
            telemetry.addData("hamburger", hamburger);
        }
        //enable motor config testing
        if (gamepad1.a){
            configtesttoggle = !configtesttoggle;
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
        if(gamepad1.dpad_up){
            if(!sleepcounter1.isRunning()){
                sleepcounter1.start();
                TGP += 1;
            }

        }
        if(gamepad1.dpad_down){
            if (!sleepcounter2.isRunning()){
                sleepcounter2.start();
                TGP -= 1;
            }

        }
        if(sleepcounter1.elapsed(TimeUnit.MILLISECONDS) > 130){
            if(sleepcounter1.isRunning()){
                sleepcounter1.reset();
            }
        }
        if(sleepcounter2.elapsed(TimeUnit.MILLISECONDS) > 130){
            if(sleepcounter2.isRunning()){
                sleepcounter2.reset();
            }
        }
        /*if(spin.getCurrentPosition() != spin2.getCurrentPosition()){ //prevent inaccuracy due to uneven positionings
            int positiondiff = abs(spin.getCurrentPosition() - spin2.getCurrentPosition());
            if(positiondiff > 10){ //allow 3 buffer units before going into 'careful' mode
                spin.setPower(0.1);
                spin2.setPower(0.1);
            }
            else{
                spin.setPower(1);
                spin2.setPower(1);
            }
        }*/



        spin.setTargetPosition(TGP);
        spin2.setTargetPosition(spin.getCurrentPosition());
        spin = spin2;
        /*if(gamepad1.dpad_right){
            vaccum.setPosition(vaccum.getPosition() + 0.01);
        }
        if(gamepad1.dpad_left){
            vaccum.setPosition(vaccum.getPosition() - 0.01);
        }*/



        //telemetry

        telemetry.addData("Status", "Running");
        telemetry.addData("LF Power", lfPower);
        telemetry.addData("RF Power", rfPower);
        telemetry.addData("LB Power", lbPower);
        telemetry.addData("RB Power", rbPower);
        telemetry.addData("CTT", configtesttoggle);
        telemetry.addData("CTC", configtestcycle);
        //telemetry.addData("ededededede", vaccum.getPosition());
        telemetry.addData("TGP", TGP);
        telemetry.addData("spin1tarpos", spin.getTargetPosition());
        telemetry.addData("spin2tarpos", spin2.getTargetPosition());
        telemetry.addData("spin2pos", spin2.getCurrentPosition());
        telemetry.addData("spinpos", spin.getCurrentPosition());
        telemetry.addData("sleeps1", sleepcounter1);
        telemetry.update();
    }

    private void stopDriveMotors() {
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        lbMotor.setPower(0);
        rbMotor.setPower(0);
    }
}
