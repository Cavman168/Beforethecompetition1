package org.firstinspires.ftc.teamcode;

import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;
@TeleOp
public class timertest extends OpMode {

    Stopwatch counter = Stopwatch.createUnstarted();
    Boolean timer1 = TimerMethod.CreateStarted(3000);
    int w;

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

    }
    private void mainloop(){

        if(!counter.isRunning()){
            counter.start();
        }
        if(counter.isRunning()){
            if(counter.elapsed(TimeUnit.MILLISECONDS) >= 3000){
                counter.reset();
            }
        }
        if(timer1){
            w++;
        }
        telemetry.addData( "timer1",timer1);
        telemetry.addData("counterelapsed", counter.elapsed(TimeUnit.MILLISECONDS));
        telemetry.addData("counterisruning", counter.isRunning());
        telemetry.addData("w", w);
        telemetry.update();
    }
}
