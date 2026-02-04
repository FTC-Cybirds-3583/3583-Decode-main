package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class ZZPIDshooterTuning extends OpMode
{
    public DcMotorEx shooter;
    public double high = 1500; //P=7.6 F=15.2
    // 2000;P= 7.3 F = 15.1
    public double med = 1200; //P = 9.92 F = 15.19
    public double low = 600;
    double targetVelocity = med;
    double P = 200;
    double F = 12.86;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01};
    int step_index = 1;
    ElapsedTime time = new ElapsedTime();
    boolean waiting = false;
    float ms_to_full = 0;
    float time_to_fix = 0;
    int initial_velocity = 0;
    float timeratio = 0;
    float best_time = 0;
    boolean done_lap = false;
    float endBattery = 0;

    float best_avg = -999999;
    boolean got_elapsed_time = false;


    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()){
            targetVelocity += 100;
        }
        if (gamepad1.xWasPressed()) {
            targetVelocity -= 100;
        }

        if(gamepad1.bWasPressed())
        {
            step_index = (step_index + 1) % step_sizes.length;
        }

        if (gamepad1.dpadLeftWasPressed())
        {
            F -= step_sizes[step_index];
        }
        if (gamepad1.dpadRightWasPressed())
        {
            F += step_sizes[step_index];
        }

        if (gamepad1.dpadUpWasPressed())
        {
            P += step_sizes[step_index];
        }
        if (gamepad1.dpadDownWasPressed())
        {
            P -= step_sizes[step_index];
        }

        float actualTarget = (float) targetVelocity;
        if (gamepad1.left_bumper) {
            actualTarget -= 300; // simulates a shot
            resetTimer();
        }
        if (gamepad1.right_bumper) {
            actualTarget += 500;
        }
        double curVelocity = shooter.getVelocity();
        double error = targetVelocity - curVelocity;

        if (gamepad1.aWasPressed()) {
            ms_to_full = 0;
            time.reset();
            waiting = true;
        }
        if (gamepad1.a) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
            shooter.setVelocity(actualTarget);
        } else {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(0);
            shooter.setVelocity(0);
        }

        if (waiting && shooter.getVelocity() == targetVelocity) {
            waiting = false;
            ms_to_full = (float) time.milliseconds();

        }

        telemetry.addData("MS TO FULL: ", ms_to_full);
        telemetry.addData("Target Velocity: ", actualTarget);
        telemetry.addData("Current Velocity: ", curVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Tuning P: ", "%4f (D-pad U/D)", P);
        telemetry.addData("Tuning F: ", "%4f (D-pad L/R)", F);
        telemetry.addData("Step Size: ", "%4f (B button)", step_sizes[step_index]);




    }

    public void resetTimer() {
        time.reset();
        got_elapsed_time = false;
        time_to_fix = 0;
        initial_velocity = (int) shooter.getVelocity();
    }
}
