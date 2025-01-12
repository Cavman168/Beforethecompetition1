package org.firstinspires.ftc.teamcode;

import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TheIs extends OpMode {
    DcMotor spin, spin2, lfMotor, lbMotor, rfMotor, rbMotor, slide;
    Servo Grabber;
    Stopwatch tarcounter = Stopwatch.createUnstarted();
    Stopwatch tarcounter2 = Stopwatch.createUnstarted();
    Stopwatch tarcounter3 = Stopwatch.createUnstarted();
    Stopwatch tarcounter4 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter = Stopwatch.createUnstarted();
    Stopwatch sleepcounter2 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter3 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter4 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter5 = Stopwatch.createUnstarted();
    Stopwatch sleepcounter6 = Stopwatch.createUnstarted();
    Stopwatch tartimer = Stopwatch.createUnstarted();
    boolean DoReverse = true; // this will be inversed on first run
    boolean drivetoggle = true;
    boolean grabtoggle = false;
    boolean slidetoggle = false;
    boolean hasreset = false;
    boolean slidetoggleverif;
    int interval = 10;

    @Override
    public void init() {
        initrobot();
    }

    @Override
    public void loop() {
        mainloop();
    }

    @Override
    public void stop() {
        super.stop();
    }
    private  void initrobot(){
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        spin.setTargetPosition(0);
        spin2.setTargetPosition(0);
        //lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }// heyy... me from the past here, typing this cus i dont actually want to be wroking rn... today is right after the first meet and im tired af so im counting on you future me to actually do the work. Thanks!
    private  void mainloop(){
        if(!drivetoggle){
            if(gamepad1.left_stick_y > 0){
                spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spin.setPower(gamepad1.left_stick_y);
            }
            if(gamepad1.right_stick_y > 0){
                spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spin2.setPower(gamepad1.right_stick_y);
            }
        }
        if(drivetoggle){
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double lfPower = drive + strafe + rotate;
            double rfPower = drive - strafe - rotate;
            double lbPower = drive - strafe + rotate;
            double rbPower = drive + strafe - rotate;
            /*double lfPower = drive + strafe + rotate;
            double rfPower = drive - strafe - rotate;
            double lbPower = drive + strafe + rotate;
            double rbPower = drive - strafe - rotate;*/

            lfMotor.setPower(lfPower);
            rfMotor.setPower(rfPower);
            lbMotor.setPower(lbPower);
            rbMotor.setPower(rbPower);
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin.setPower(1);
            spin2.setPower(1);
            if(hasreset == false){
                spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hasreset = true;
            }
        }

        if(gamepad1.b){
            if(!tartimer.isRunning()){
                spin.setTargetPosition(1180);
                tartimer.start();
            }
        }
        if(tartimer.isRunning()){
            if(tartimer.elapsed(TimeUnit.SECONDS) == 4){
                spin.setTargetPosition(70);
            }
            if(tartimer.elapsed(TimeUnit.SECONDS) == 6){
                spin2.setTargetPosition(1830);
                spin.setTargetPosition(-1020);
            }
        }
        if(tartimer.isRunning()){
            if(tartimer.elapsed(TimeUnit.SECONDS) >= 10){
                tartimer.reset();
            }

        }

        if(gamepad1.y){
            if(!sleepcounter.isRunning()){
                drivetoggle = !drivetoggle;
                sleepcounter.start();
            }
        }
        if(sleepcounter.isRunning()){
            if(sleepcounter.elapsed(TimeUnit.MILLISECONDS) > 100){
                sleepcounter.reset();
            }
        }
        /*telemetry.addData("spiin power", spin.getPower());
        telemetry.addData("spiin power 2", spin2.getPower());
        telemetry.update();*/

        if(gamepad1.x){
            spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad1.dpad_up){
            if(!tarcounter.isRunning()){
                spin.setTargetPosition(spin.getTargetPosition() + interval);
                spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin.setPower(1);
                tarcounter.start();
            }
        }

        if(tarcounter.isRunning()){
            if(tarcounter.elapsed(TimeUnit.MILLISECONDS) > 30){
                tarcounter.reset();
            }
        }

        if(gamepad1.dpad_down){
            if(!tarcounter2.isRunning()){
                spin.setTargetPosition(spin.getTargetPosition() - interval);
                spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin.setPower(1);
                tarcounter2.start();
            }
        }

        if(tarcounter2.isRunning()){
            if(tarcounter2.elapsed(TimeUnit.MILLISECONDS) > 30){
                tarcounter2.reset();
            }
        }
        if(gamepad2.dpad_up){
            if(!tarcounter3.isRunning()){
                spin2.setTargetPosition(spin2.getTargetPosition() + interval);
                spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin2.setPower(1);
                tarcounter3.start();
            }
        }

        if(tarcounter3.isRunning()){
            if(tarcounter3.elapsed(TimeUnit.MILLISECONDS) > 100){
                tarcounter3.reset();
            }
        }

        if(gamepad2.dpad_down){
            if(!tarcounter4.isRunning()){
                spin2.setTargetPosition(spin2.getTargetPosition() - interval);
                spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin2.setPower(1);
                tarcounter4.start();
            }
        }

        if(tarcounter4.isRunning()){
            if(tarcounter4.elapsed(TimeUnit.MILLISECONDS) > 100){
                tarcounter4.reset();
            }
        }


        if(sleepcounter.isRunning()){
            if(sleepcounter.elapsed(TimeUnit.MILLISECONDS) > 150){
                sleepcounter.reset();
            }
        }


        if(gamepad2.a){
            if(!sleepcounter5.isRunning()){
                grabtoggle = !grabtoggle;
                sleepcounter5.start();
            }
        }
        if(sleepcounter5.isRunning()){
            if(sleepcounter5.elapsed(TimeUnit.MILLISECONDS) > 300){
                sleepcounter5.reset();
            }
        }
        if(grabtoggle){
            if(!sleepcounter2.isRunning()){
                Grabber.setPosition(0.7);
                sleepcounter2.start();
            }
        }
        if(sleepcounter2.isRunning()){
            if(sleepcounter2.elapsed(TimeUnit.MILLISECONDS) > 500){
                sleepcounter2.reset();
            }
        }
        if(!grabtoggle){
            if(!sleepcounter3.isRunning()){
                Grabber.setPosition(0.3);
                sleepcounter3.start();
            }
        }
        if(sleepcounter3.isRunning()){
            if(sleepcounter3.elapsed(TimeUnit.MILLISECONDS) > 500){
                sleepcounter3.reset();
            }
        }

        if(gamepad2.right_trigger > 0){
            slide.setPower(-gamepad2.right_trigger);
        }
        if(gamepad2.right_trigger == 0){
            slide.setPower(gamepad2.left_trigger);
        }
        if(!slidetoggle){
            slide.setPower(0.2);
        }
        if(gamepad2.right_trigger > 0){
            if(gamepad2.left_trigger > 0){
                slidetoggle = true;
            }
            else{
                slidetoggleverif = false;
            }
        }
        if(gamepad2.b){
            if(!sleepcounter6.isRunning()){
                spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin.setPower(1);
                spin2.setPower(1);
                //spin.setTargetPosition(-150);
                spin.setTargetPosition(-3100);
                spin2.setTargetPosition(150);
                sleepcounter6.start();
            }
        }
        if(sleepcounter6.isRunning()){
            /*if(sleepcounter6.elapsed((TimeUnit.MILLISECONDS)) > 1000){
                spin2.setTargetPosition(650);
                spin.setTargetPosition(-350);
            }
            if(sleepcounter6.elapsed(TimeUnit.MILLISECONDS) > 2000){
                spin2.setTargetPosition(1050);
            }
            if(sleepcounter6.elapsed(TimeUnit.MILLISECONDS) > 3000){
                spin.setTargetPosition(-3100);
                spin2.setTargetPosition(6500);
            }*/
            if(sleepcounter6.elapsed(TimeUnit.MILLISECONDS) > 2000){
                spin2.setTargetPosition(6500);
            }
            if(sleepcounter6.elapsed(TimeUnit.MILLISECONDS) > 4500){
                sleepcounter6.reset();
            }
        }
        if(!slidetoggleverif && slide.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            slidetoggle = false;
        }
        if(slidetoggle = false){
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        telemetry.addData("spintarpos", spin.getTargetPosition());
        telemetry.addData("spinpos", spin.getCurrentPosition());
        telemetry.addData("spin2tarpos", spin2.getTargetPosition());
        telemetry.addData("spin2pos", spin2.getCurrentPosition());
        telemetry.addData("drivetoggle", drivetoggle);
        telemetry.addData("grabtoggle", grabtoggle);
        telemetry.addData("grabpos", Grabber.getPosition());
        telemetry.update();
    }

}
