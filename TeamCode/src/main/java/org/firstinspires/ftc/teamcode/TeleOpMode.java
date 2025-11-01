package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Array;
@TeleOp(name="Cybirds-Tele-1")
public class TeleOpMode extends OpMode
{
    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor intake;
    Limelight3A limelight;
    int slide_up_pos = 2350;
    int slide_down_pos = 0;
    int slide_target_pos = 100;
    double speed;
    double speed_fine_inc = 0.05;
    boolean r_bump_1 = false;
    boolean l_bump_1 = false;

    @Override

    public void init() {
        rightRear = hardwareMap.dcMotor.get("backright");
        leftRear = hardwareMap.dcMotor.get("backleft");
        rightFront = hardwareMap.dcMotor.get("frontright");
        leftFront = hardwareMap.dcMotor.get("frontleft");
        rightSlide = hardwareMap.dcMotor.get("rightslide");
        leftSlide = hardwareMap.dcMotor.get("leftslide");
        intake = hardwareMap.dcMotor.get("intake");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set slides to target their current position
        slide_target_pos = 0;
        //initialise slide encoder doodads :3
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(slide_target_pos);
        rightSlide.setTargetPosition(slide_target_pos);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        //initialise limelight on pipeline 0
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100000000);
        limelight.start();

        limelight.pipelineSwitch(0);
        //set drive speed at 0.5 initially
        speed = 0.5;
        //initialise bumpers as "not pressed"
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

    public void dual_joy_control(float left_stick_x,float left_stick_y,float right_stick_x,float right_stick_y) {
        /*TABLE OF INP
             LX  LY  RX
        RR   +   -   -
        RL   +   +   -
        FR   -   -   -
        FL   -   +   -
        */

        //don't ask how this works, it is wonderful
        rightRear.setPower(-speed*(left_stick_x-left_stick_y+right_stick_x));
        leftRear.setPower(-speed*(left_stick_x+left_stick_y+right_stick_x));
        rightFront.setPower(-speed*(-left_stick_x-left_stick_y+right_stick_x));
        leftFront.setPower(-speed*(-left_stick_x+left_stick_y+right_stick_x));
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
        if (gamepad1.a) {
            slide_target_pos = slide_up_pos;
        }
        if (gamepad1.b) {
            slide_target_pos = slide_down_pos;
        }
        if (gamepad1.dpad_down) {
            slide_target_pos -= 25;
        }
        if (gamepad1.dpad_up) {
            slide_target_pos += 25;
            /*if (slide_target_pos > 10000) {
                slide_target_pos = 0;
            }*/
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
            leftSlide.setPower(0.1);
        }
        if (rightSlide.isBusy() || slide_target_pos > 50) {
            rightSlide.setPower(1.0);
        } else {
            rightSlide.setPower(0.1);
        }
        leftSlide.setTargetPosition(slide_target_pos);
        rightSlide.setTargetPosition(slide_target_pos);

    }

    public void intake_control() {
        if (gamepad1.right_trigger > 0.2) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }

    public boolean limelight_things() {
        LLResult result = limelight.getLatestResult();

        telemetry.addData("result exists",result != null);
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("result valid",true);
                if (gamepad1.left_trigger > 0.2) {
                    telemetry.addData("fid0", result.getFiducialResults().get(0).getTargetPoseCameraSpace());
                    float lx = 0;
                    float ly = 0;
                    float rx = 0;

                    if (result.getTy() < 5) {
                        ly= -1;
                    }
                    if (result.getTx() > 2) {
                        lx = 1;
                    } else if (result.getTx() < -2) {
                        lx = -1;
                    }
                    if (result.getBotpose().getOrientation().getYaw() > 130) {
                        rx = 1;
                    } else if (result.getBotpose().getOrientation().getYaw() < 130) {
                        rx = -1;
                    }
                    dual_joy_control(lx,ly,rx,0);

                    return true;
                }
            } else {

                telemetry.addData("result valid",false);
            }
        }
        return false;
    }

    public void do_p1_things() {
        p1_fine_speed_control();
        slide_control();
        intake_control();
        intake_control();

        if (!limelight_things()) {
            dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);
        }
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        do_p1_things();
        do_p2_things();
        telemetry.update();
    }
}

