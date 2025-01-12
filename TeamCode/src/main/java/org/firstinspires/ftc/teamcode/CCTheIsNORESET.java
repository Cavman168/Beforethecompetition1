package org.firstinspires.ftc.teamcode;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
@TeleOp
public class CCTheIsNORESET extends OpMode {
    DcMotor spin, spin2, lfMotor, lbMotor, rfMotor, rbMotor, slide;
    Servo Grabber, light;
    CRServo suck, suck2;
    DistanceSensor Radar;

    Stopwatch[] stopwatches = new Stopwatch[13];
    boolean drivetoggle = true;
    boolean hasreset = false;
    boolean manualtoggle = false;
    boolean canextendslide = false;
    boolean grabtoggle = false;
    boolean[] hasgonetoposition = new boolean[3];
    boolean shouldpull = false;
    boolean waittopull = false;
    boolean IsCorrectRange;
    double speedtoggle = 0.5;
    double gradientspeed = 0.0001;
    double gradientbuffer = 0.030;
    double lightspeed;

    String placeholder = " ";


    @Override
    public void init() {
        initrobot();
        for (int i = 0; i < stopwatches.length; i++) { //make an int "i" = 0 then for every time that i is less than the length of the array increment i set the array equal to i to be stopwatch.createunstarted
            stopwatches[i] = Stopwatch.createUnstarted();
        }
        Arrays.fill(hasgonetoposition, false);
    }

    @Override
    public void loop() {
        drivecode();
        armcode();
        runstopwatches();
        slidecode();
        grabbercode();
        telemetry();
        suckcode();
        lightcode();
    }

    public void stop(){
        super.stop();
    }
    private void initrobot(){
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        Grabber = hardwareMap.get(Servo.class, "Grabber");
        suck = hardwareMap.get(CRServo.class, "suck");
        suck2 = hardwareMap.get(CRServo.class, "suck2");
        light =  hardwareMap.get(Servo.class, "light");
        Radar = hardwareMap.get(DistanceSensor.class, "Radar");
        spin.setTargetPosition(0);
        spin2.setTargetPosition(0);
        slide.setTargetPosition(0);
        //lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        suck.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    private void drivecode(){
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

            if(gamepad1.left_bumper){
                speedtoggle = 1;
            }
            else{
                speedtoggle = 0.5;
            }

            lfMotor.setPower(lfPower * speedtoggle);
            rfMotor.setPower(rfPower * speedtoggle);
            lbMotor.setPower(lbPower * speedtoggle);
            rbMotor.setPower(rbPower * speedtoggle);
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin.setPower(1);
            spin2.setPower(1);
        }
    }
    private void armcode(){
        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad2.dpad_up || gamepad2.dpad_down && !manualtoggle){
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad1.a && !stopwatches[0].isRunning()){
            manualtoggle = !manualtoggle;
            stopwatches[0].start();
        }
        if(manualtoggle){ //manually control arm using joysticks
            drivetoggle = false;
            spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spin.setPower(gamepad1.left_stick_y);
            spin2.setPower(gamepad1.right_stick_y);
        }
        else{
            drivetoggle = true;
        }
        if(gamepad1.x && gamepad1.start){
            spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //manual pos setting NOT manual controlling
        if(gamepad1.dpad_up && !manualtoggle && !stopwatches[1].isRunning()){
            spin.setTargetPosition(spin.getCurrentPosition() + 20);
            stopwatches[1].start();
        }
        if(gamepad1.dpad_down && !manualtoggle && !stopwatches[1].isRunning()){
            spin.setTargetPosition(spin.getCurrentPosition() - 20);
            stopwatches[1].start();
        }
        //spin 2 manual pos setting
        if(gamepad2.dpad_up && !manualtoggle && !stopwatches[2].isRunning()){
            spin2.setTargetPosition(spin2.getCurrentPosition() + 20);
            stopwatches[2].start();
        }
        if(gamepad2.dpad_down && !manualtoggle && !stopwatches[2].isRunning()){
            spin2.setTargetPosition(spin2.getCurrentPosition() - 20);
            stopwatches[2].start();
        }
        //auto positions
        //hang specimen on top rung
        if(!stopwatches[1].isRunning() && !stopwatches[2].isRunning() && !stopwatches[4].isRunning() && gamepad2.a && !stopwatches[6].isRunning()){
            waittopull = false;
            //set slide to zero
            canextendslide = true;
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            slide.setTargetPosition(0);
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
            if(stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 1450 && !hasgonetoposition[2] && shouldpull){
                spin2.setTargetPosition(1067);
                hasgonetoposition[2] = true;
            }
        }
        //toggle cycle to hang specimen on top rung\
        if(gamepad2.a && !stopwatches[5].isRunning() && stopwatches[4].isRunning()){
            stopwatches[5].start();
        }
        if(gamepad2.a && waittopull){
            shouldpull = !shouldpull;
        }
        //top basket
        if(gamepad2.b && !stopwatches[6].isRunning()){
            //spin.setTargetPosition(-450);
            spin.setTargetPosition(-432);
            spin2.setTargetPosition(800);
            stopwatches[10].reset();
            stopwatches[6].start();
        }
        if(stopwatches[6].isRunning()){
            if(stopwatches[6].elapsed(TimeUnit.MILLISECONDS) > 750){
                //spin2.setTargetPosition(2066); //potential better pos instead
                spin2.setTargetPosition(2283);
            }
            if(stopwatches[6].elapsed(TimeUnit.MILLISECONDS) > 2500){
                canextendslide = true;
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                slide.setTargetPosition(2280);

            }
        }
        // reach into 'submersible'
        if(gamepad2.y && !stopwatches[7].isRunning()){
            spin2.setTargetPosition(170);
            canextendslide = true;
            slide.setTargetPosition(0);
            stopwatches[10].start();
            stopwatches[7].start();
        }
        if(stopwatches[7].isRunning()){
            if(stopwatches[7].elapsed(TimeUnit.MILLISECONDS) > 800){
                spin.setTargetPosition(809); //955
                spin2.setTargetPosition(35); //143
            }
            if(stopwatches[7].elapsed(TimeUnit.MILLISECONDS) > 1100){
                spin.setTargetPosition(1522);
                spin2.setTargetPosition(-505);
                //slide.setTargetPosition(1070);
            }
        }
        //suck from wall
        if(slide.getTargetPosition() <= 0 && gamepad2.right_trigger > 0){
            spin.setTargetPosition(286); //-252
            spin2.setTargetPosition(325); //879
        }
        if(slide.getTargetPosition() <= 0 && gamepad2.left_trigger > 0){
            spin2.setTargetPosition(480); //potentially incorrect, i guessed //922
        }
        //hang robot
        if(gamepad1.x && !stopwatches[11].isRunning()){
            canextendslide = false;
            spin.setTargetPosition(-779);
            spin2.setTargetPosition(2193);
            stopwatches[11].start();
        }
        if(stopwatches[11].isRunning()){
            if(stopwatches[11].elapsed(TimeUnit.MILLISECONDS) > 2000){
                spin.setTargetPosition(-732);
                spin2.setTargetPosition(1013);
            }
        }
    }
    private void slidecode(){
        /*if(canextendslide && gamepad2.right_trigger > 0){
            slide.setPower(gamepad2.right_trigger);
        }
        if(canextendslide && gamepad2.right_trigger < 0){
            slide.setPower(gamepad2.left_trigger);
        }*/
        if(canextendslide){
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }
        if(!canextendslide){
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setPower(0);
        }
        if(gamepad2.dpad_right && !stopwatches[6].isRunning() && canextendslide){
            //canextendslide = true;
            slide.setPower(1);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition(slide.getTargetPosition() + 20);
        }
        if(gamepad2.dpad_left && !stopwatches[6].isRunning() && canextendslide){
            //canextendslide = true;
            slide.setPower(1);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setTargetPosition(slide.getTargetPosition() - 20);
        }
    }
    private void grabbercode(){
        if(gamepad2.x && !stopwatches[3].isRunning()){
            grabtoggle = !grabtoggle;
            stopwatches[3].start();
        }
        if(grabtoggle){
            Grabber.setPosition(1);
        }
        if(!grabtoggle){
            Grabber.setPosition(0.3);
        }
    }
    private void suckcode(){
        if(gamepad2.left_bumper && !gamepad2.right_bumper){
            suck.setPower(-1);
            suck2.setPower(-1);
        }
        if(gamepad2.right_bumper && !gamepad2.left_bumper){
            suck.setPower(1);
            suck2.setPower(1);
        }
        if(gamepad2.left_bumper && gamepad2.right_bumper){
            suck.setPower(0);
            suck2.setPower(0);
        }
    }
    private void lightcode(){
        /*if(!stopwatches[8].isRunning() && stopwatches[8].elapsed(TimeUnit.MILLISECONDS) == 0){
            light.setPosition(0.277);
            stopwatches[8].start();
        }
        if(stopwatches[8].isRunning() && light.getPosition() != 0.722 && light.getPosition() < 0.723){
            light.setPosition(light.getPosition() + gradientspeed * 10);
        }
        if(stopwatches[9].isRunning() && light.getPosition() != 0.277 && light.getPosition() > 0.276){
            light.setPosition(light.getPosition() - gradientspeed * 10);
        }*/
        if(!stopwatches[8].isRunning()){
            if(Radar.getDistance(DistanceUnit.INCH) > 7 && Radar.getDistance(DistanceUnit.INCH) < 27.9 && !IsCorrectRange){ // 7.87
                light.setPosition(0.5 - Radar.getDistance(DistanceUnit.INCH) * 0.01 + 0.0787);
            }
            if(Radar.getDistance(DistanceUnit.INCH) > 27.9){
                light.setPosition(0.611);
            }
            if(Radar.getDistance(DistanceUnit.INCH) < 4){
                light.setPosition(0.279);
            }
            if(Radar.getDistance(DistanceUnit.CM) >= 17.5 && Radar.getDistance(DistanceUnit.CM) <= 21){
                IsCorrectRange = true;
                light.setPosition(1);
            }
            else{IsCorrectRange = false;}
            stopwatches[8].start();
        }
    }

    private void runstopwatches(){
        if(stopwatches[0].isRunning() && stopwatches[0].elapsed(TimeUnit.MILLISECONDS) > 350){ //sleep timer for setting the manual toggle
            stopwatches[0].reset();
        }
        if(stopwatches[1].isRunning() && stopwatches[1].elapsed(TimeUnit.MILLISECONDS) > 50){ //sleep for spin(1) manual pos setting
            stopwatches[1].reset();
        }
        if(stopwatches[2].isRunning() && stopwatches[2].elapsed(TimeUnit.MILLISECONDS) > 50){ //sleep for spin2 manual pos setting
            stopwatches[2].reset();
        }
        if(stopwatches[3].isRunning() && stopwatches[3].elapsed(TimeUnit.MILLISECONDS) > 200){ //sleep for setting grab toggle
            stopwatches[3].reset();
        }
        if(stopwatches[4].isRunning() && stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 7500 /*1500*/ && shouldpull || stopwatches[4].elapsed(TimeUnit.MILLISECONDS) > 20000){ // reset/sleep for positioning on upper rung for specimen
            hasgonetoposition[0] = false;
            hasgonetoposition[1] = false; //reset for next runthrough | could potentially be replaced with arrays.fill
            hasgonetoposition[2] = false;
            shouldpull = false;
            canextendslide = false;
            stopwatches[4].reset();
        }
        if(stopwatches[5].isRunning() && stopwatches[5].elapsed(TimeUnit.MILLISECONDS) > 1200){ //sleep for shouldpull toggle
            waittopull = true;
            stopwatches[5].reset();
        }
        if (stopwatches[6].isRunning() && stopwatches[6].elapsed(TimeUnit.MILLISECONDS) > 3750){ // reset/sleep for top basket position
            stopwatches[6].reset();
        }
        if(stopwatches[7].isRunning() && stopwatches[7].elapsed(TimeUnit.MILLISECONDS) > 1100){
            stopwatches[7].reset();
        }
        /*if(stopwatches[8].isRunning() && stopwatches[8].elapsed(TimeUnit.MILLISECONDS) > Radar.getDistance(DistanceUnit.INCH) * 0.1){
            stopwatches[8].reset();
        }
        if(!stopwatches[9].isRunning() && light.getPosition() == 0.722 || !stopwatches[9].isRunning() && light.getPosition() >= 0.722 - gradientbuffer && light.getPosition() <= 0.722 + gradientbuffer){ //reverse gradient for rgb led
            stopwatches[8].stop();
            stopwatches[9].start();
        }
        if(stopwatches[9].isRunning() && stopwatches[9].elapsed(TimeUnit.MILLISECONDS) > 6000 && light.getPosition() == 0.277 || stopwatches[9].isRunning() && light.getPosition() >= 0.277 - gradientbuffer && light.getPosition() <= 0.277 + gradientbuffer){ //if finished gradient pattern reset the gradient
            stopwatches[8].reset();
            stopwatches[9].reset();
        }*/
        if(stopwatches[8].isRunning() && stopwatches[8].elapsed(TimeUnit.MILLISECONDS) > 25){
            stopwatches[8].reset();
        }
        if(stopwatches[10].isRunning() && slide.getTargetPosition() <= 0 && stopwatches[10].elapsed(TimeUnit.MILLISECONDS) > 8500){ //anti illegal slide for submersible
            canextendslide = false;
            stopwatches[10].reset();
        }
        if(stopwatches[11].isRunning() && stopwatches[11].elapsed(TimeUnit.MILLISECONDS) > 5500){
            stopwatches[11].reset();
        }
    }
    private void telemetry(){
        telemetry.addData("Actual Positions", placeholder);
        telemetry.addData("spin current pos", spin.getCurrentPosition());
        telemetry.addData("spin2 current pos", spin2.getCurrentPosition());
        telemetry.addData("slide current pos", slide.getCurrentPosition());
        telemetry.addData("Target Positions", placeholder);
        telemetry.addData("spin target pos", spin.getTargetPosition());
        telemetry.addData("spin2 target pos", spin2.getTargetPosition());
        telemetry.addData("slide target pos", slide.getTargetPosition());
        telemetry.addData("etc", placeholder);
        telemetry.addData("Should pull?", shouldpull);
        telemetry.addData("lightpos", light.getPosition());
        telemetry.addData("stop9elpsmillis", stopwatches[9].elapsed(TimeUnit.MILLISECONDS));
        telemetry.addData("distance", Radar.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
