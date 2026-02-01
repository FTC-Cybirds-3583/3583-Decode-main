package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@TeleOp
public class ZZCybirdsPIDshooterTuning extends Zkely
{
    public DcMotorEx shooter;
    float best_avg = -999999;

    public double med = 1200; //P = 9.92 F = 15.19
    double targetVelocity = med;
    double P = 275;
    double F = 12.86;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int step_index = 1;
    boolean got_elapsed_time = false;
    float time_to_fix = 0;
    int initial_velocity = 0;
    float timeratio = 0;
    float best_time = 0;
    boolean done_lap = false;
    float endBattery = 0;
    float startBattery = 0;
    float best_points = 0;




    ElapsedTime time = new ElapsedTime();


    @Override
    public void runOpMode() {
        //DESMOS NUMBERS
        //P = 6.35296*(10^-8)*(Vel^2.40218) + 10.58787 WRONG
        //P = -0.00026*Vel + 11.525

        //F = 1.85007*(10^-12)*(Vel^3.64017) + 12.87594


        // 750 // 11V
        //P - FORGOT
        //F - 15.3

        // 750 //13.1V
        //P - 11.1
        //F - 12.93

        //1000 //13.07
        //P - 11.61
        //F - 13.03

        // 1500 // 11V
        //P - 13.295
        //F - 15.49
        // 1500 // 13.3V
        //P - 11.02
        //F - 13.55
        zkely_init();
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        waitForStart();
        while (opModeIsActive() && !done_lap) {
            update_imu();
            teleLoop();
            power_dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y,0.5);
            telemetry.update();
        }
        sleepMS(5000);
        telemetry.addData(": DONE LAP!","");
        telemetry.addData(": Best Average",best_avg);
        telemetry.addData(": Best Points",best_points);
        telemetry.addData(": P",P);
        telemetry.addData(": F",F);
        telemetry.addData(": Target Vel",targetVelocity);
        telemetry.addData(": Start Battery",startBattery);
        telemetry.addData(": End Battery",endBattery);
        telemetry.update();

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);
        shooter.setVelocity(0);

        while (opModeIsActive()) {

        }
    }

    public void resetTimer() {
        time.reset();
        got_elapsed_time = false;
        time_to_fix = 0;
        initial_velocity = (int) shooter.getVelocity();
    }
    public void teleLoop() {

        if (gamepad1.yWasPressed()) {
            targetVelocity += 100;
            resetTimer();
            if (targetVelocity > 2000) {
                targetVelocity = 2000;
            }
        }
        if (gamepad1.xWasPressed()) {
            targetVelocity -= 100;
            resetTimer();
            if (targetVelocity < 100) {
                targetVelocity = 100;
            }
        }
        if(gamepad1.bWasPressed())
        {
            step_index = (step_index + 1) % step_sizes.length;
        }
        if(gamepad1.aWasPressed())
        {
            startBattery = (float) currentVoltage;
            F_do_a_lap(19,7,1);
        }

        if (gamepad1.dpadLeftWasPressed())
        {
            resetTimer();
            F -= step_sizes[step_index];
        }
        if (gamepad1.dpadRightWasPressed())
        {
            resetTimer();
            F += step_sizes[step_index];
        }

        if (gamepad1.dpadUpWasPressed())
        {
            resetTimer();
            P += step_sizes[step_index];
        }
        if (gamepad1.dpadDownWasPressed())
        {
            resetTimer();
            P -= step_sizes[step_index];
        }

        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
        int actualTargetVel = (int) targetVelocity;
        if (gamepad1.options) {
            actualTargetVel += 500;
        }
        if (gamepad1.share) {
            actualTargetVel -= 500;
        }
        shooter.setVelocity(actualTargetVel);
        double curVelocity = shooter.getVelocity();
        double error = targetVelocity - curVelocity;
        if (Math.abs(error) <= 15 && !got_elapsed_time) {
            got_elapsed_time = true;
            time_to_fix = (float) time.milliseconds();
            timeratio = (float) (time_to_fix/Math.abs((targetVelocity-initial_velocity)));
        }

        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Current Velocity: ", curVelocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Tuning P: ", "%4f (D-pad U/D)", P);
        telemetry.addData("Tuning F: ", "%4f (D-pad L/R)", F);
        telemetry.addData("Step Size: ", "%4f (B button)", step_sizes[step_index]);
        telemetry.addData("time to fix / distance",timeratio);
        telemetry.addData("distance",apriltag_distance());



    }
    public void F_do_a_lap(float start, float min, float increment) {
        telemetry.update();
        best_time = 0;
        float best_p = 0;
        best_points = 0;
        float best_diff = 9999;
        for (float temp_p = start; temp_p > min; temp_p -= increment) {
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData(": DOING F LAP (STEP (1/3)","");
            telemetry.addData("current increment ",increment);
            telemetry.addData("current P ",P);
            telemetry.addData("testing F ",temp_p);
            telemetry.addData("current best F ",best_p);
            telemetry.addData("current best time",best_time);
            telemetry.addData("current best avg",best_avg);
            telemetry.addData("current best points",best_points);
            telemetry.addData("target",targetVelocity);
            telemetry.addData("Slowing!","");
            telemetry.update();
            while (shooter.getVelocity() > 0) {
                //wait
            }
            if (gamepad1.left_trigger > 0.2) {
                break;
            }

            PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.0,temp_p);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
            shooter.setVelocity(targetVelocity);
            resetTimer();
            while (Math.abs(targetVelocity - shooter.getVelocity()) > 10) {

                telemetry.addData(": DOING F LAP (STEP 2/3)","");
                telemetry.addData("current increment ",increment);
                telemetry.addData("current P ",P);
                telemetry.addData("testing F ",temp_p);
                telemetry.addData("current best F ",best_p);
                telemetry.addData("current best time",best_time);
                telemetry.addData("current best avg",best_avg);
                telemetry.addData("current best points",best_points);
                telemetry.addData("target",targetVelocity);
                telemetry.addData("current",shooter.getVelocity());
                telemetry.update();
                if (time.milliseconds() > 3000) {
                    break;
                }
                //wait until up to speed
            }

            time_to_fix = (float) time.milliseconds();
            float avg = 0;
            float points = 0;
            for (int ms_left = 6000; ms_left > 0; ms_left -= 25) {
                sleepMS(25);
                if (Math.abs(shooter.getVelocity()-targetVelocity) <= 10) {
                    points += 1;
                }
                float ms_passed = (6000-ms_left);

                avg += shooter.getVelocity();
                telemetry.addData(": DOING F LAP (STEP 3/3)","");
                telemetry.addData("current increment ",increment);
                telemetry.addData("current points ",points);
                telemetry.addData("current P ",P);
                telemetry.addData("testing F ",temp_p);
                telemetry.addData("current best F ",best_p);
                telemetry.addData("current best time",best_time);
                telemetry.addData("current best avg",best_avg);
                telemetry.addData("current best points",best_points);
                telemetry.addData("target",targetVelocity);
                telemetry.addData("current",shooter.getVelocity());
                telemetry.addData("ms left",ms_left);
                telemetry.update();
                //wait while up to speed
            }
            avg = avg/50;
            /*if (Math.abs(targetVelocity-avg) < best_diff) {
                best_time = time_to_fix;
                best_avg = avg;
                best_diff = (float) Math.abs(targetVelocity-avg);
                best_p = temp_p;
            }*/
            if (points > best_points) {
                best_time = time_to_fix;
                best_points = points;
                best_p = temp_p;
            }

        }
        if (increment == 0.01f) {
            endBattery = (float) currentVoltage;
            F = best_p;
            done_lap = true;
        } else if (increment == 0.1f) {
            F_do_a_lap(best_p+0.05f,best_p-0.05f,0.01f);
        } else if (increment == 1f) {
            F_do_a_lap(best_p+0.5f,best_p-0.5f,0.1f);
        }

    }

}
