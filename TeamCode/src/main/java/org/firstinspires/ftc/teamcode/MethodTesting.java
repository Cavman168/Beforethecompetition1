package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@TeleOp
public class MethodTesting extends OpMode {
    DcMotor spin, spin2, lfMotor, lbMotor, rfMotor, rbMotor, slide;
    Servo Grabber, light;
    CRServo suck, suck2;
    DistanceSensor Radar;
    Stopwatch[] stopwatches = new Stopwatch[10];
    int positionindex = 0;

    @Override
    public void init() {
        initrobot();
    }

    @Override
    public void loop() {
        run();
        runstopwatches();
    }

    private void initrobot(){
        Arrays.fill(stopwatches, Stopwatch.createUnstarted());
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        suck = hardwareMap.get(CRServo.class, "suck");
        suck2 = hardwareMap.get(CRServo.class, "suck2");
        Radar = hardwareMap.get(DistanceSensor.class, "Radar");
    }
    private void run(){
        if(gamepad1.dpad_up && !stopwatches[0].isRunning()){
            positionindex++;
            stopwatches[0].start();
        }
        if(gamepad1.dpad_down && !stopwatches[0].isRunning()){
            positionindex--;
            stopwatches[0].start();
        }
        if(!stopwatches[1].isRunning() && gamepad1.a){
            //GoTo.pos(positionindex);
            stopwatches[1].start();
        }
        telemetry.addData("index", positionindex);
        telemetry.update();
    }
    private void runstopwatches(){
        if(stopwatches[0].isRunning() && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 200){
            stopwatches[0].reset();
        }
        if(stopwatches[1].isRunning() && stopwatches[1].elapsed(TimeUnit.MILLISECONDS) > 200){
            stopwatches[1].reset();
        }
    }
}
