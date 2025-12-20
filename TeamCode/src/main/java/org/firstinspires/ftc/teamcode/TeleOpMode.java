package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import java.lang.reflect.Array;
@TeleOp(name="Zestacular Teleop")
public class TeleOpMode extends Zkely
{

    boolean r_bump_1 = false;
    boolean l_bump_1 = false;
    boolean dpad_up_1 = false;
    boolean dpad_down_1 = false;

    @Override

    public void init() {
        zkely_init();

        r_bump_1 = false;
        l_bump_1 = false;
        dpad_up_1 = false;
        dpad_down_1 = false;
    };

    public void do_p1_things() {
        p1_fine_speed_control();
        slide_control();
        intake_control();

        if (!limelight_target(gamepad1.left_stick_button, true)) {
            if (!limelight_target(gamepad1.right_stick_button,false)) {
                power_dual_joy_control(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y, speed);
            }
        }

    }
    public void p1_fine_speed_control() {
        if (gamepad1.left_bumper) {
            max_outtake_power = close_max_outtake_power;
        }
        if (gamepad1.right_bumper) {
            max_outtake_power = far_max_outtake_power;
        }
    }
    public void slide_control() {
        if (gamepad1.dpad_up && !dpad_up_1) {
            posSlide(slide_up_pos,500);
        }
        if (gamepad1.dpad_down && !dpad_down_1) {
            posSlide(slide_down_pos,100);
        }
        dpad_up_1 = gamepad1.dpad_up;
        dpad_down_1 = gamepad1.dpad_down;
    }
    public void intake_control() {
        intake.setPower(intake_dir * gamepad1.right_trigger);
        if (gamepad1.right_trigger > 0.2) {
            midtake.setPower(midtake_dir * 1);
        }
        outtake.setPower(outtake_dir * gamepad1.left_trigger*max_outtake_power);

        telemetry.addData("starting yaw", robot_starting_yaw);
        telemetry.addData("team",team);
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        update_imu();
        do_p1_things();
        do_p2_things();

        if (gamepad1.b) {
            midtake.setPower(1);
            midtake_2.setPower(1);
        } else if (gamepad1.a) {
            midtake.setPower(-1);
            midtake_2.setPower(-1);
        } else {
            midtake.setPower(0);
            midtake_2.setPower(0);
        }
        if (gamepad1.x) {
            innertake.setPosition(innertake_up_pos);
        }
        if (gamepad1.y) {
            innertake.setPosition(innertake_down_pos);
        }
        innertake.setPosition(innertake.getPosition());
        if (gamepad1.start) {
            team = "R";
        }
        if (gamepad1.back) {
            team = "B";
        }

    }
}

