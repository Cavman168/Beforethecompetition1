package org.firstinspires.ftc.teamcode;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@TeleOp
public class DRYTheIs extends OpMode {
    // Hardware components
    DcMotor spin, spin2, lfMotor, lbMotor, rfMotor, rbMotor, slide;
    Servo Grabber;

    // Toggles and intervals
    boolean drivetoggle = true;
    boolean grabtoggle = false;
    boolean slidetoggle = false;
    int interval = 50;

    // Stopwatches for timing
    private final Stopwatch[] stopwatches = new Stopwatch[6];
    private final Object stopwatchLock = new Object(); // Lock for thread-safe access

    // ExecutorService for managing threads
    ExecutorService executor = Executors.newFixedThreadPool(3);

    @Override
    public void init() {
        initRobot();

        // Initialize stopwatches
        for (int i = 0; i < stopwatches.length; i++) {
            stopwatches[i] = Stopwatch.createUnstarted();
        }

        // Start background threads
        executor.submit(this::timerManagement); // Handle timers in a separate thread
        executor.submit(this::telemetryUpdater); // Handle telemetry in another thread
    }

    @Override
    public void loop() {
        handleDriveToggle();
        handleDriving();
        handleSpinControls();
        handleGrabber();
        handleSlide();
    }

    @Override
    public void stop() {
        // Shutdown executor service and stop all threads
        executor.shutdownNow();
        super.stop();
    }

    private void initRobot() {
        // Map motors and servo to configuration
        lbMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        rbMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin2 = hardwareMap.get(DcMotor.class, "Spin2");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        Grabber = hardwareMap.get(Servo.class, "Grabber");

        // Configure motor directions
        slide.setDirection(DcMotor.Direction.REVERSE);
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void handleDriveToggle() {
        // Toggles drive mode on 'Y' button press with a short delay
        if (gamepad1.y) {
            drivetoggle = !drivetoggle;
            safeResetStopwatch(0); // Reset timer to avoid rapid toggling
            safeStartStopwatch(0); // Start stopwatch to delay the next toggle
        }
    }

    private void handleDriving() {
        if (drivetoggle) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            setMotorPowers(drive + strafe + rotate, drive - strafe - rotate,
                    drive - strafe + rotate, drive + strafe - rotate);
        }
    }

    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        lfMotor.setPower(lf);
        rfMotor.setPower(rf);
        lbMotor.setPower(lb);
        rbMotor.setPower(rb);
    }

    private void handleSpinControls() {
        if (gamepad1.dpad_up) {
            spin.setTargetPosition(spin.getTargetPosition() + interval);
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin.setPower(1);
        }
        if (gamepad1.dpad_down) {
            spin.setTargetPosition(spin.getTargetPosition() - interval);
            spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spin.setPower(1);
        }
    }

    private void handleGrabber() {
        if (gamepad2.a) {
            grabtoggle = !grabtoggle;
            Grabber.setPosition(grabtoggle ? 0.7 : 0.3);
            safeResetStopwatch(1); // Reset stopwatch for grabber
            safeStartStopwatch(1); // Start stopwatch to delay the next toggle
        }
    }

    private void handleSlide() {
        double slidePower = gamepad2.right_trigger > 0 ? -gamepad2.right_trigger : gamepad2.left_trigger;
        slide.setPower(slidePower);
    }

    private void timerManagement() {
        try {
            while (!Thread.currentThread().isInterrupted()) {
                // Example timer management, logging elapsed times
                long elapsed = safeElapsedMillis(0);
                telemetry.addData("Elapsed Time (ms)", elapsed);
                Thread.sleep(50); // Polling delay
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted state
        }
    }

    private void telemetryUpdater() {
        try {
            while (!Thread.currentThread().isInterrupted()) {
                // Continuously update telemetry
                telemetry.addData("Spin Position", spin.getCurrentPosition());
                telemetry.addData("Spin2 Position", spin2.getCurrentPosition());
                telemetry.addData("Grabber Position", Grabber.getPosition());
                telemetry.update();
                Thread.sleep(100); // Update interval
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore interrupted state
        }
    }

    // Synchronized Stopwatch methods
    private void safeStartStopwatch(int index) {
        synchronized (stopwatchLock) {
            if (!stopwatches[index].isRunning()) {
                stopwatches[index].start();
            }
        }
    }

    private void safeResetStopwatch(int index) {
        synchronized (stopwatchLock) {
            if (stopwatches[index].isRunning()) {
                stopwatches[index].reset();
            }
        }
    }

    private long safeElapsedMillis(int index) {
        synchronized (stopwatchLock) {
            return stopwatches[index].elapsed(TimeUnit.MILLISECONDS);
        }
    }
}
