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

        if (!limelight_target(gamepad1.left_stick_button, 180)) {
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
        intake.setPower(-gamepad1.right_trigger);
        outtake.setPower(-gamepad1.left_trigger*max_outtake_power);
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        telemetry.addData("midpos",midtake.getPosition());
        update_imu();
        do_p1_things();
        do_p2_things();

        if (gamepad1.a) {
            midtake.setPosition(midtake_up_pos);
        }
        if (gamepad1.b) {
            midtake.setPosition(midtake_down_pos);
        }
        if (gamepad1.x) {
            innertake.setPosition(innertake_up_pos);
        }
        if (gamepad1.y) {
            innertake.setPosition(innertake_down_pos);
        }

    }
}

