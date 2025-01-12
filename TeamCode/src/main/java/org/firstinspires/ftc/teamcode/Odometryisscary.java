package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.google.common.base.Stopwatch;

@TeleOp(name="OdometryIsScary", group="TeleOp")
public class Odometryisscary extends OpMode {
    // Declare variables
    DcMotor lbMotor, rbMotor, lfMotor, rfMotor;
    double speedreducer = 0.5;
    int isthisrunning = 0;
    Stopwatch count1 = Stopwatch.createUnstarted();

    //cavab was here
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
    private void initDrive(){
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        //reverse motors
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    // I love java!


    private void driveRobot(){

        //main run method
        //begin standard drive
        double drive = -gamepad1.left_stick_y * speedreducer;
        double strafe = gamepad1.left_stick_x * speedreducer;
        double rotate = gamepad1.right_stick_x * speedreducer;

        //VIOLENT Mode
        if(gamepad1.left_bumper){
            speedreducer = 1; //remove limiter
        }
        else{
            speedreducer = 0.5; //relimit
        }

        isthisrunning++;

        double lfPower = drive + strafe + rotate;
        double rfPower = drive - strafe - rotate;
        double lbPower = drive - strafe + rotate;
        double rbPower = drive + strafe - rotate;




        lfMotor.setPower(lfPower);
        rfMotor.setPower(rfPower);
        lbMotor.setPower(lbPower);
        rbMotor.setPower(rbPower);

        if(lbPower > 0){
            if(rbPower > 0){
                if (!count1.isRunning()){
                    count1.start();
                }
            }
        }
        if(lbPower < 0){
            if(rbPower < 0){
                if (!count1.isRunning()){
                    count1.start();
                }
            }
        }
        if(lbPower == 0){
            if(rbPower == 0){
                if (!count1.isRunning()){
                    count1.start();
                }
            }
        }



        //end standard drive

        //that was a lie... maybe ill learn kotlin
        telemetry.addData("isthisrunningh??", isthisrunning);
        telemetry.addData("count1", count1);
    }
    private  void stopDriveMotors(){
        //stop motors

    }
}
