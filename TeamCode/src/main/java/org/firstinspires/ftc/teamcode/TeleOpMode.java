package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Array;
@TeleOp(name="Zestacular Teleop")
public class TeleOpMode extends OpMode
{
    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightSlide;
    DcMotor leftSlide;
    DcMotor intake;
    DcMotor outtake;
    IMU.Parameters myIMUparameters;
    IMU imu;
    YawPitchRollAngles robotOrientation;
    Pose3D last_botpose;
    int last_tag;

    // Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
    double robot_yaw;
    double robot_pitch;
    double robot_roll;
    Limelight3A limelight;
    int slide_up_pos = 2350;
    int slide_down_pos = -5;
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
        outtake = hardwareMap.dcMotor.get("outtake");
        imu = hardwareMap.get(IMU.class, "imu");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set slides to target their current position
        slide_target_pos = -5;
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
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        //initialise limelight on pipeline 0
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        //set drive speed at 0.5 initially
        speed = 0.5;
        //initialise bumpers as "not pressed"
        r_bump_1=false;
        l_bump_1=false;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                    AxesReference.INTRINSIC,
                                    AxesOrder.ZYX,
                                    AngleUnit.DEGREES,
                                    90f,
                                    0f,
                                    0f,
                                    0l
                                )
                )
        );
        imu.initialize(myIMUparameters);
    };

    public float avg(float[] nums){
        int numonums = nums.length;
        float tot = 0;
        for (int i=0; i<numonums; i++) {
            tot+=nums[i];
        }
        tot/=numonums;
        return tot;
    }

    public void dual_joy_control(float left_stick_x,float left_stick_y,float right_stick_x,float right_stick_y,double s) {
        /*TABLE OF INP
             LX  LY  RX
        RR   +   -   -
        RL   +   +   -
        FR   -   -   -
        FL   -   +   -
        */

        //don't ask how this works, it is wonderful
        rightRear.setPower(s*(left_stick_x-left_stick_y-right_stick_x));
        leftRear.setPower(s*(left_stick_x+left_stick_y-right_stick_x));
        rightFront.setPower(s*(-left_stick_x-left_stick_y-right_stick_x));
        leftFront.setPower(s*(-left_stick_x+left_stick_y-right_stick_x));
        telemetry.addData("rightRear", s*(left_stick_x-left_stick_y-right_stick_x));
        telemetry.addData("leftRear", s*(left_stick_x+left_stick_y-right_stick_x));
        telemetry.addData("rightFront", s*(-left_stick_x-left_stick_y-right_stick_x));
        telemetry.addData("leftFront", s*(-left_stick_x+left_stick_y-right_stick_x));
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

    public boolean limelight_things() {
        LLResult result = limelight.getLatestResult();

        telemetry.addData("result exists",result != null);
        if (result != null) {
            float lx = 0;
            float ly = 0;
            float rx = 0;
            float target_ty = 18.5f;
            float target_tx = 2.1f;
            float target_yaw = -135;
            if (last_tag == 24) {
                target_yaw = 135;
            }
            telemetry.addData("lastTag",last_tag);
            telemetry.addData("target_yaw",target_yaw);
            double tx_var = 1;
            double ty_var = 1;
            double yaw_var = 2;
            double l_speed = 0.5f;
            if (result.isValid()) {
                last_tag = result.getFiducialResults().get(0).getFiducialId();
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("result valid",true);
                telemetry.addData("fiducial",result.getFiducialResults().get(0).getFiducialId());
                if (gamepad1.left_stick_button) {
                    telemetry.addData("fid0", result.getFiducialResults().get(0).getTargetPoseCameraSpace());

                    if (result.getTy() < target_ty-ty_var) {
                        ly= -1;
                    } else if (result.getTy() > target_ty + ty_var) {
                        ly = 1;
                    }
                    if (Math.abs(result.getTy() - target_ty) < 2) {
                        ly = ly*0.2f;
                    }

                    if (result.getTx() > target_tx+tx_var) {
                        lx = 1;
                    } else if (result.getTx() < target_tx-tx_var) {
                        lx = -1;
                    }
                    if (Math.abs(result.getTx() - target_tx) < 2) {
                        lx = lx*0.2f;
                    }

                    if (target_yaw > 0) {
                        if (robot_yaw < target_yaw-yaw_var) {
                            rx = -1;
                        } else if (robot_yaw > target_yaw+yaw_var) {
                            rx = 1;
                        }
                    } else {
                        if (robot_yaw < -target_yaw-yaw_var) {
                            rx = -1;
                        }
                        if (robot_yaw > (-target_yaw + yaw_var)) {
                            rx = 1;
                        }
                    }
                    if (angle_distance((float) robot_yaw,target_yaw) < 45) {
                        rx = rx*0.2f;
                    }

                    dual_joy_control(lx,ly,rx,0,l_speed);

                    return true;
                }
                last_botpose = botpose;
            } else {
                if (last_botpose != null && gamepad1.left_stick_button) {
                    l_speed = 0.75d;
                    if (target_yaw > 0) {
                        if (robot_yaw < target_yaw-yaw_var) {
                            rx = -1;
                        } else if (robot_yaw > target_yaw+yaw_var) {
                            rx = 1;
                        }
                    } else {
                        if (robot_yaw < -target_yaw-yaw_var) {
                            rx = -1;
                        }
                        if (robot_yaw > (-target_yaw + yaw_var)) {
                            rx = 1;
                        }
                    }

                    if (rx == 0) {
                        if (last_botpose.getPosition().x < -0.7) {
                            lx = 1;
                        } else if (last_botpose.getPosition().x > -0.3) {
                            lx = -1;
                        }
                    }

                    if (angle_distance((float) robot_yaw,target_yaw) < 45) {
                        rx = rx*0.2f;
                    }
                    dual_joy_control(lx,ly,rx,0,l_speed);
                    return true;
                }
                telemetry.addData("result valid",false);
            }
        }
        return false;
    }
    public float angle_distance(float angle_1,float angle_2) {
        angle_1 = (angle_1 % 360 + 360) % 360;
        angle_2 = (angle_2 % 360 + 360) % 360;
        float dist = Math.abs(angle_1 - angle_2);

        return Math.min(dist,360-dist);
    }
    public float map(float min_out, float max_out,float min_in,float max_in,float val_in) {
        return min_out + ((max_out-min_out) / (max_in - min_in)) * (val_in - min_in);
    }

    public void do_p1_things() {
        p1_fine_speed_control();
        slide_control();
        intake_control();

        if (!limelight_things()) {
            dual_joy_control(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y,speed);
        }
    }

    public void update_imu() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robot_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        robot_pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        robot_roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        telemetry.addData("yaw",robot_yaw);
        telemetry.addData("pitch",robot_pitch);
        telemetry.addData("roll",robot_roll);
    }

    public void do_p2_things() {
        //To do
    }

    @Override
    public void loop() {
        update_imu();
        do_p1_things();
        do_p2_things();
        telemetry.update();
    }
}

