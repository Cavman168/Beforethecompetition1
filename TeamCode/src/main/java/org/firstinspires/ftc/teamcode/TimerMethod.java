package org.firstinspires.ftc.teamcode;

import com.google.common.base.Stopwatch;

import java.util.concurrent.TimeUnit;

public class TimerMethod {
    private static boolean sleep = false;
    private static boolean startstop;
    public static boolean CreateStarted(double timemillis){
        Stopwatch stopwatch = Stopwatch.createStarted();
        if(stopwatch.elapsed(TimeUnit.MILLISECONDS) >= timemillis){
            stopwatch.reset();
        }
        if(stopwatch.isRunning()){
            sleep = false;
        }
        if(!stopwatch.isRunning()){
            sleep = true;
            stopwatch.start();
        }
        return sleep; //return true once every timemillis
    }
    public static boolean CreateUnstarted(double timemillis){
        Stopwatch stopwatch = Stopwatch.createUnstarted();
        if(startstop){
            stopwatch.start();
        }
        if(stopwatch.elapsed(TimeUnit.MILLISECONDS) >= timemillis){
            stopwatch.stop();
        }
        if(!startstop){
            stopwatch.stop();
        }
        return stopwatch.isRunning();
    }
    public static void start(){
        startstop = true;
    }
    public static void stop(){
        startstop = false;
    }



}
