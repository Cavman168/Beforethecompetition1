package org.firstinspires.ftc.teamcode;

import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class The extends OpMode {
    //declare variables
    DcMotor spin, spin2;
    Stopwatch tarcounter = Stopwatch.createUnstarted();
    Stopwatch tarcounter2 = Stopwatch.createUnstarted();

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
    private void initrobot(){
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        //init copde ere
    }
    private void mainloop(){
        if(gamepad1.dpad_up){
            if(!tarcounter.isRunning()){
                tarcounter.start();
            }
        }

        if(tarcounter.isRunning()){
            spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin.setPower(1);
            spin2.setPower(1);
        }

        if(!tarcounter.isRunning()){
            if(!tarcounter2.isRunning()){
                spin.setTargetPosition(spin.getCurrentPosition());
                spin2.setTargetPosition(spin2.getCurrentPosition());
                spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if(tarcounter.isRunning()){
            if(tarcounter.elapsed(TimeUnit.MILLISECONDS) > 100){
                tarcounter.reset();
            }
        }
        if (gamepad1.dpad_left){
            spin.setPower(0.8);
        }

        if(gamepad1.dpad_down){
            if(!tarcounter2.isRunning()){
                tarcounter2.start();
            }
        }

        if(tarcounter2.isRunning()){
            spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin.setPower(-1);
            spin2.setPower(-1);
        }

        if(tarcounter2.isRunning()){
            if(tarcounter2.elapsed(TimeUnit.MILLISECONDS) > 100){
                tarcounter2.reset();
            }
        }

    }
}
