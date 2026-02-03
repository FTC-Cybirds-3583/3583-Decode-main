package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "MainEncoderAuto")

public class MainEncoderAuto extends Zkely {
    private final ElapsedTime runtime = new ElapsedTime();
    int vel;
    float mod_amount = 10;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01};
    int step_index = 1;
    float ticks_to_stop = 0;
    int last_tag = 23;

    public void farAuto() {
        zkely_init();
        vel = 1750;
        waitForStart();
        run_updates();

        ElapsedTime time_since_start = new ElapsedTime();

        start_outtake_velocity_auto();
        while (limelight_teleop_circle(true) > 0.1 && opModeIsActive() && time_since_start.milliseconds() < 2500) {
            run_updates();
            telemetry.addData("correct angle", correction_angle);
            telemetry.update();
        }

        telemetry.addData("past limelight", "");
        telemetry.update();
        ElapsedTime wait_time = new ElapsedTime();
        while (opModeIsActive() && !motor_at_velocity(outtake, outtake_velocity, 20) && wait_time.milliseconds() < 1500) {

            telemetry.addData("cycle wait", "");
            telemetry.update();
        }
        telemetry.addData("cycle shoot", "");
        telemetry.update();
        shootAutoVelocity();

        run_updates();

        float old_true_yaw = true_yaw;
        float angle_to_180 = (180 - Math.abs(old_true_yaw));
        posTurn(angle_to_180 / 90, 1000, (int) (-1 * Math.signum(old_true_yaw)), 0);
        posStrafe(1.2f, 1000, (int) Math.signum(old_true_yaw), 0);
    }

    public void closeAuto(boolean check_obelisk) {
        zkely_init();
        vel = 1700;
        waitForStart();
        run_updates();

        intake.setPower(intake_dir);
        posStraight(1.75f, 2500, -1, 0.8f);
        start_outtake_velocity_auto();
        new_limelight_target_auto();
        set_tag_team();

        telemetry.addData("past limelight", "");
        telemetry.update();
        ElapsedTime wait_time = new ElapsedTime();
        while (opModeIsActive() && !motor_at_velocity(outtake, outtake_velocity, 20) && wait_time.milliseconds() < 1500) {

            telemetry.addData("cycle wait", "");
            telemetry.update();
        }
        telemetry.addData("cycle shoot", "");
        telemetry.update();
        shootAutoVelocity();

        telemetry.addData("moving on", "");
        telemetry.update();
        if (check_obelisk) {
            posTurn(0.6f, 1000, side_modifier * -1, 1);
            limelight_read_auto();
            posTurn(0.6f, 1000, side_modifier, 1);
        } else {
            current_tag = 23;
        }

        cycle_tag(current_tag);

        int next_tag = 23;
        if (last_tag == 23) {
            next_tag = 22;
        }
        cycle_tag(next_tag);

        posStrafe(1.2f, 2500, side_modifier, 1);
    }

    public void cycle_tag(int tag) {
        //PERFECT ON 12.8
        last_tag = tag;
        float turn_mod = 0.4775f;
        float strafe_mod = 0.66f;
        if (voltageSensor.getVoltage() < 13.4) {
            turn_mod += (float) (13.4 - voltageSensor.getVoltage()) * (0.047f); //perfect at ~13.2
            strafe_mod += (float) (13.4 - voltageSensor.getVoltage()) * (0.03f);
        }
        posJoystick(1, vel, side_modifier * strafe_mod, -0.05f, side_modifier * turn_mod, 0);

        float intake_straight_amt = 1.2f;
        if (tag < 22) {
            intake_straight_amt += 0.35f;
        }
        posStrafe((23 - tag), vel, side_modifier, 0);
        startIntake();
        posStraight(intake_straight_amt, 1000, 1, 1);
        stopIntake();
        intake.setPower(intake_dir);
        posStraight(intake_straight_amt, vel, -1, 1);
        posStrafe((23 - tag), vel, -1 * side_modifier, 0);

        posJoystick(1, vel, side_modifier * -strafe_mod, 0.05f, side_modifier * -turn_mod, 0);

        start_outtake_velocity_auto();
        new_limelight_target_auto();

        telemetry.addData("past limelight", "");
        telemetry.update();
        ElapsedTime wait_time = new ElapsedTime();
        while (opModeIsActive() && !motor_at_velocity(outtake, outtake_velocity, 20) && wait_time.milliseconds() < 1500) {

            telemetry.addData("cycle wait", "");
            telemetry.update();
        }
        telemetry.addData("cycle shoot", "");
        telemetry.update();
        shootAutoVelocity();
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}

