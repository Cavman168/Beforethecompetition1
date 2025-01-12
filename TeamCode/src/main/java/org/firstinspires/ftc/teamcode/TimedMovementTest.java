package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TimedMovementTest extends OpMode {
    DcMotor lbMotor, rbMotor, lfMotor, rfMotor, spin, spin2;
    Stopwatch timer = Stopwatch.createStarted();
    Stopwatch timer1 = Stopwatch.createUnstarted();

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

    private void initDrive() {
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");

        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*if(!timer.isRunning()){
            if(timer.elapsed(TimeUnit.SECONDS) == 0){
                timer.start();
            }
        }
        if(timer.elapsed(TimeUnit.SECONDS) >= 1){
            timer.stop();
        }
        if(timer.isRunning()){
            lbMotor.setPower(1);
            lfMotor.setPower(1);
            rfMotor.setPower(1);
            rbMotor.setPower(1);
        }
        else{
            lbMotor.setPower(0);
            lfMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);
        }*/
        telemetry.addData("timor,", timer.elapsed(TimeUnit.SECONDS));
    }

    private void driveRobot() {
        if(gamepad1.a){
            if(!timer1.isRunning()){
                timer1.start();
            }
        }
        if(timer1.isRunning()){
            lbMotor.setPower(0.6);
            lfMotor.setPower(0.6);
            rfMotor.setPower(0.6);
            rbMotor.setPower(0.6);
        }
        else{
            lbMotor.setPower(0);
            lfMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);
        }
        if(timer1.isRunning()){
            if(timer1.elapsed(TimeUnit.SECONDS) >= 1){
                timer1.stop();
                lbMotor.setPower(0);
                lfMotor.setPower(0);
                rfMotor.setPower(0);
                rbMotor.setPower(0);
            }
        }
        telemetry.addData("timor1", timer1);
    }


    private void stopDriveMotors() {

    }
}
