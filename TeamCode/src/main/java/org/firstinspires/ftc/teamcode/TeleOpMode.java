package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
@TeleOp(name="Cybirds-Tele-1")
public class TeleOpMode extends OpMode
{
    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;

    double speed;

    double speed_fine_inc = 0.05;

    boolean r_bump_1 = false;
    boolean l_bump_1 = false;

    String testString = "hi";

    @Override

    public void init() {
        rightRear = hardwareMap.dcMotor.get("backright");
        leftRear = hardwareMap.dcMotor.get("backleft");
        rightFront = hardwareMap.dcMotor.get("frontright");
        leftFront = hardwareMap.dcMotor.get("frontleft");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        speed = 0.5;
        r_bump_1=false;
        l_bump_1=false;
    }

    public float avg(float[] nums){
        int numonums = nums.length;
        float tot = 0;
        for (int i=0; i<numonums; i++) {
            tot+=nums[i];
        }
        tot/=numonums;
        return tot;
    }

    public void dual_joy_control() {
        /*TABLE OF INP
             LX  LY  RX
        RR   +   -   -
        RL   +   +   -
        FR   -   -   -
        FL   -   +   -
        */
        rightRear.setPower(-speed*(gamepad1.left_stick_x-gamepad1.left_stick_y+gamepad1.right_stick_x));
        leftRear.setPower(-speed*(gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x));
        rightFront.setPower(-speed*(-gamepad1.left_stick_x-gamepad1.left_stick_y+gamepad1.right_stick_x));
        leftFront.setPower(-speed*(-gamepad1.left_stick_x+gamepad1.left_stick_y+gamepad1.right_stick_x));
    }
    public void drive(double speed) {
        /*TABLE OF INP
             LX  LY  RX
        RR   +   -   -
        RL   +   +   -
        FR   -   -   -
        FL   -   +   -
        */
    rightRear.setPower(-speed);
    leftRear.setPower(speed);
    rightFront.setPower(-speed);
    leftFront.setPower(speed);
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


    public void do_p1_things() {
        p1_fine_speed_control();
        dual_joy_control();
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        do_p1_things();
        do_p2_things();
        telemetry.addData("running?", true);
        telemetry.update();
    }
}

