package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.ejml.dense.block.decomposition.qr.BlockHouseHolder_FDRB;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Array;
@TeleOp(name="Zestacular Teleop")
public class TeleOpMode extends Zkely
{
    @Override

    public void init() {
        zkely_init();
    };

    public void do_p1_things() {
        p1_fine_speed_control();
        slide_control();
        intake_control();

        if (!limelight_target()) {
            power_dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y,speed);
        }
    }
    public void p1_fine_speed_control() {
        if (gamepad1.right_bumper) {
            if (!r_bump_1) {
                speed+=speed_fine_inc;
            }
            r_bump_1=true;
        } else {
            r_bump_1 = false;
        }
        if (gamepad1.left_bumper) {
            if (!l_bump_1) {
                speed-=speed_fine_inc;
            }
            l_bump_1=true;
        } else {
            l_bump_1 = false;
        }
    }
    public void slide_control() {
        if (gamepad1.dpad_up) {
            slide_target_pos = slide_up_pos;
        }
        if (gamepad1.dpad_down) {
            slide_target_pos = slide_down_pos;
        }
        if (gamepad1.dpad_left) {
            slide_target_pos = 0;

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (leftSlide.isBusy() || slide_target_pos > 50) {
            leftSlide.setPower(1.0);
        } else {
            leftSlide.setPower(0);
        }
        if (rightSlide.isBusy() || slide_target_pos > 50) {
            rightSlide.setPower(1.0);
            telemetry.addData("power is 0 on right slide", false);
        } else {
            rightSlide.setPower(0);
            telemetry.addData("power is 0 on right slide", true);
        }
        if (leftSlide.getPower() > 0) {
            leftSlide.setTargetPosition(slide_target_pos);
        }
        if (rightSlide.getPower() > 0) {
            rightSlide.setTargetPosition(slide_target_pos);
        }
    }
    public void intake_control() {
        intake.setPower(-gamepad1.right_trigger);
        outtake.setPower(-gamepad1.left_trigger);
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        update_imu();
        if (gamepad1.a) {
            posDrive(1300,1000);
        }
        do_p1_things();
        do_p2_things();



    }
}

