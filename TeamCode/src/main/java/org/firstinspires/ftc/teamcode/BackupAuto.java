package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;


@Autonomous
public class BackupAuto extends OpMode {
    double inpersec = 30.9; //36
    double tarposx = -24;
    double tarposy = 0; //19
    double xbuffer = 5; //prevbiousdly 15
    double ybuffer = 5;
    double processedx;
    double processedy;
    boolean xisnegative = false;
    boolean yisnegative = false;
    boolean xhasrun = false;
    boolean yhasrun = false;
    double posx;
    double posy;
    boolean shouldpull = false;
    boolean[] hasgonetoposition = new boolean[3];
    long time;
    String isdefault = " ";
    Stopwatch XTimer = Stopwatch.createUnstarted();
    Stopwatch YTimer = Stopwatch.createUnstarted();
    Stopwatch ActionsTimer = Stopwatch.createUnstarted();
    Stopwatch[] stopwatches = new Stopwatch[5];
    DcMotor spin, spin2, lfMotor, lbMotor, rfMotor, rbMotor, slide;
    Servo Grabber, light;
    CRServo suck, suck2;
    @Override
    public void init() {
        initrobot();

    }

    @Override
    public void loop() {
        main();
    }
    private void initrobot(){
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        suck = hardwareMap.get(CRServo.class, "suck");
        suck2 = hardwareMap.get(CRServo.class, "suck2");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setTargetPosition(0);
        spin2.setTargetPosition(0);
        Arrays.fill(stopwatches, Stopwatch.createUnstarted());
        Arrays.fill(hasgonetoposition, false);
    }
    private void main(){
        time = ActionsTimer.elapsed(TimeUnit.MILLISECONDS);
        if(Math.abs(tarposx) > tarposx){
            xisnegative = true;
        }
        if(Math.abs(tarposy) > tarposy){
            yisnegative = true;
        }
        if(xisnegative){
            processedx = (Math.abs(tarposx) + Math.abs(posx) * -1) / inpersec;
            if(!XTimer.isRunning() && !xhasrun){
                XTimer.start();
            }
            if(XTimer.isRunning()){
                lfMotor.setPower(-0.6);
                lbMotor.setPower(0.6);
                rfMotor.setPower(0.6);
                rbMotor.setPower(-0.6);
            }
        }
        if(!xisnegative){
            processedx = (Math.abs(tarposx) - Math.abs(posx) * -1) / inpersec;
            if(!XTimer.isRunning() && !xhasrun){
                XTimer.start();
            }
            if(XTimer.isRunning()){
                lfMotor.setPower(0.6);
                lbMotor.setPower(-0.6);
                rfMotor.setPower(-0.6);
                rbMotor.setPower(0.6);
            }
        }
        if(yisnegative){
            processedy = Math.abs((Math.abs(tarposy) + Math.abs(posy) * -1)) / inpersec;
            if(!YTimer.isRunning() && !yhasrun && xhasrun){
                YTimer.start();
            }
            if(YTimer.isRunning()){
                lfMotor.setPower(-0.6);
                lbMotor.setPower(-0.6);
                rfMotor.setPower(-0.6);
                rbMotor.setPower(-0.6);
            }
        }
        if(!yisnegative){
            processedy = (Math.abs(tarposy) + Math.abs(posy) * -1) / inpersec;
            if(!YTimer.isRunning() && !yhasrun && xhasrun){
                YTimer.start();
            }
            if(YTimer.isRunning()){
                lfMotor.setPower(0.6);
                lbMotor.setPower(0.6);
                rfMotor.setPower(0.6);
                rbMotor.setPower(0.6);
            }
        }
        if(XTimer.isRunning() && XTimer.elapsed(TimeUnit.SECONDS) >= processedx){
            xhasrun = true;
            posx = tarposx;
            XTimer.reset();
        }
        if(YTimer.isRunning() && YTimer.elapsed(TimeUnit.SECONDS) >= processedy){
            yhasrun = true;
            posy = tarposy;
            YTimer.reset();
        }
        if(xhasrun && yhasrun || !XTimer.isRunning() && !YTimer.isRunning()){
            lfMotor.setPower(0);
            lbMotor.setPower(0);
            rfMotor.setPower(0);
            rbMotor.setPower(0);
        }
        if(yhasrun && xhasrun){
            if(!ActionsTimer.isRunning()){
                ActionsTimer.start();
            }
            if(ActionsTimer.isRunning()){
                /*switch((int) ActionsTimer.elapsed(TimeUnit.MILLISECONDS)){
                    case 3500:
                        spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        spin.setPower(1);
                        spin2.setPower(1);
                        if(!stopwatches[1].isRunning() && !stopwatches[2].isRunning() && !stopwatches[4].isRunning()){
                            //set first step in position
                            spin.setTargetPosition(-252);
                            stopwatches[4].start();
                        }
                    case 7600:
                        tarposx = -25;
                        tarposy = 29;
                        resetvalues();
                    case 13000:
                        if(shouldpull){
                            spin2.setTargetPosition(1067);
                            hasgonetoposition[2] = true;
                        }
                        shouldpull = true;
                    default:
                         isdefault = "hello...";
                }*/
                if(ActionsTimer.elapsed(TimeUnit.MILLISECONDS) > 3500 && ActionsTimer.elapsed(TimeUnit.MILLISECONDS) <  5500){
                    spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spin.setPower(1);
                    spin2.setPower(1);
                    if(!stopwatches[1].isRunning() && !stopwatches[2].isRunning() && !stopwatches[4].isRunning()){
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
                            spin2.setTargetPosition(1544); //1524
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
                if(time > 7600 && time < 13000){
                    tarposy = 29.5;
                    if(tarposx < posx){
                        xisnegative = true;
                    } else if (tarposx > posx){
                        xisnegative = false;
                    }
                    if(tarposy < posy){
                        yisnegative = true;
                    } else if(tarposy > posy){
                        yisnegative = false;
                    }
                    if(yhasrun && tarposx != posx){
                        xhasrun = false;
                    }
                    if(xhasrun && tarposy != posy){
                        yhasrun = false;
                    }
                }
                if(time > 13000 && time < 14300){
                    if(shouldpull){
                        spin2.setTargetPosition(1067);
                        hasgonetoposition[2] = true;
                    }
                    shouldpull = true;
                }
                if(time > 14300 && time < 16000){
                    tarposy = 0.5;
                    if(tarposx < posx){
                        xisnegative = true;
                    } else if (tarposx > posx){
                        xisnegative = false;
                    }
                    if(tarposy < posy){
                        yisnegative = true;
                    } else if(tarposy > posy){
                        yisnegative = false;
                    }
                    if(yhasrun && tarposx != posx){
                        xhasrun = false;
                    }
                    if(tarposy != posy){
                        yhasrun = false;
                    }
                }
                if(time > 17000){
                    tarposx = 10;
                    if(tarposx < posx){
                        xisnegative = true;
                    } else if (tarposx > posx){
                        xisnegative = false;
                    }
                    if(tarposy < posy){
                        yisnegative = true;
                    } else if(tarposy > posy){
                        yisnegative = false;
                    }
                    if(yhasrun && tarposx != posx){
                        xhasrun = false;
                    }
                    if(xhasrun && tarposy != posy){
                        yhasrun = false;
                    }
                }
            }
        }
        telemetry.addData("st1", XTimer.elapsed(TimeUnit.SECONDS));
        telemetry.addData("st2", YTimer.elapsed(TimeUnit.SECONDS));
        telemetry.addData(" ", isdefault);
        telemetry.update();
    }
    public void resetvalues(){
        if(tarposx < posx){
            xisnegative = true;
        } else if (tarposx > posx){
            xisnegative = false;
        }
        if(tarposy < posy){
            yisnegative = true;
        } else if(tarposy > posy){
            yisnegative = false;
        }
        if(yhasrun && tarposx != posx){
            xhasrun = false;
        }
        if(xhasrun && tarposy != posy){
            yhasrun = false;
        }
    }
}
