package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.lang.reflect.Array;
@TeleOp(name="Zestacular Teleop")
public class TeleOpMode extends Zkely
{

    boolean r_bump_1 = false;
    boolean l_bump_1 = false;
    boolean dpad_up_1 = false;
    boolean dpad_down_1 = false;

    @Override

    public void runOpMode() {
        zkely_init();
        r_bump_1 = false;
        l_bump_1 = false;
        dpad_up_1 = false;
        dpad_down_1 = false;
        brakeSlides();

        waitForStart();
        speed = 1;

        while (opModeIsActive()) {
            teleLoop();
        }
    };
    public void bumpers() {

    }

    public void do_p1_things() {
        slide_control();
        intake_control();
        bumpers();
        boolean button = gamepad1.right_bumper;
        if (!limelight_teleop_circle(button)) {
            float rx = gamepad1.right_stick_x;
            if (button) { rx = 0;}
            power_dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,rx,gamepad1.right_stick_y,speed);
        }

    }
    public void slide_control() {
        if (gamepad1.dpad_up && !dpad_up_1) {
            posSlide(slide_up_pos,500);
        }
        if (!rightSlide.isBusy() && !leftSlide.isBusy()) {
            brakeSlides();
        }
        dpad_up_1 = gamepad1.dpad_up;
        dpad_down_1 = gamepad1.dpad_down;
    }
    public void intake_control() {
        intake.setPower(intake_dir * gamepad1.right_trigger);
        if (gamepad1.right_trigger > 0.2) {
            midtake.setPower(midtake_dir * midtake_power);
            if (gamepad1.left_trigger < 0.2) {
                midtake_2.setPower(midtake_dir * -1 * midtake_power);
            }
        }
        if (gamepad1.left_trigger > 0.2) {
            outtake.setVelocity(outtake_velocity);
            //outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //outtake.setPower(0.7);
        } else {
            outtake.setPower(0);
        }

    }

    public void do_p2_things() {
        //To do
    }

    public void teleLoop() {
        if (gamepad1.dpad_left) {
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else {
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        run_updates();
        do_p1_things();
        do_p2_things();

        if (gamepad1.b) {
            if (gamepad1.left_trigger < 0.2 || (gamepad1.left_trigger > 0.2 && outtake.getCurrent(CurrentUnit.MILLIAMPS) < 2000)) {
                midtake.setPower(midtake_dir * midtake_power);
                midtake_2.setPower(midtake_dir * midtake_power);
            }
        } else if (gamepad1.a) {
            midtake.setPower(-1*midtake_dir * midtake_power);
            midtake_2.setPower(-1*midtake_dir * midtake_power);
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

        telemetry.addData("outtake vel",outtake.getVelocity());
        telemetry.addData("outtake target vel",outtake_velocity);
        telemetry.addData("distance",apriltag_distance);
        telemetry.addData("outtake current",outtake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("front right current",rightFront.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("front left current",leftFront.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("back right current",rightRear.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("back left current",leftRear.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("intake current",intake.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("left slide current",leftSlide.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("right slide current",rightSlide.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();
        limit_power();
    }
}

