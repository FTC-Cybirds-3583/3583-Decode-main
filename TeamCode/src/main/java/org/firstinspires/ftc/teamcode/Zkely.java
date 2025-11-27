package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class Zkely extends OpMode
{
    DcMotorEx rightRear;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx leftFront;
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    DcMotor intake;
    DcMotor outtake;
    IMU.Parameters myIMUparameters;
    IMU imu;
    YawPitchRollAngles robotOrientation;
    Pose3D last_botpose;
    int last_tag;
    int current_tag;
    int tolerance = 3;
    int rfDefDir = -1;
    int lfDefDir = -1;
    int rbDefDir = 1;
    int lbDefDir = -1;
    int posDriveWait= 1600;

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
    int posDriveStraightSize = 1000; // js about perfect
    int posDriveStrafeSize = 1075; // between 1060 and 1100
    int posDriveTurnSize = 960; // js about perfect
    float max_outtake_power = 0.5f;

    public void zkely_init() {
        current_tag = 23;

        rightRear = hardwareMap.get(DcMotorEx.class,"backright");
        leftRear = hardwareMap.get(DcMotorEx.class,"backleft");
        rightFront = hardwareMap.get(DcMotorEx.class,"frontright");
        leftFront = hardwareMap.get(DcMotorEx.class,"frontleft");

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake");
        imu = hardwareMap.get(IMU.class, "imu");

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftRear.setVelocity(0);
        rightFront.setVelocity(0);
        leftFront.setVelocity(0);
        rightRear.setVelocity(0);

        leftRear.setTargetPositionTolerance(4);
        rightFront.setTargetPositionTolerance(4);
        leftFront.setTargetPositionTolerance(4);
        rightRear.setTargetPositionTolerance(4);

        //set slides to target their current position
        slide_target_pos = 0;
        //initialise slide encoder doodads :3
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(slide_target_pos);
        rightSlide.setTargetPosition(slide_target_pos);
        leftSlide.setVelocity(0);
        rightSlide.setVelocity(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        posSlide(slide_down_pos,800);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        //initialise limelight on pipeline 0
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        //set drive speed at 0.5 initially
        speed = 0.5;
        //initialise bumpers as "not pressed"

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                    AxesReference.INTRINSIC,
                                    AxesOrder.ZYX,
                                    AngleUnit.DEGREES,
                                    90f,
                                    0f,
                                    0f,
                                    0L
                                )
                )
        );
        imu.initialize(myIMUparameters);
    }
    public void power_dual_joy_control(float left_stick_x,float left_stick_y,float right_stick_x,float right_stick_y,double s) {
        //don't ask how this works, it is wonderful
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setPower(rbDefDir*s*(left_stick_x-left_stick_y-right_stick_x));
        leftRear.setPower(lbDefDir*s*(-left_stick_x-left_stick_y+right_stick_x));
        rightFront.setPower(rfDefDir*s*(-left_stick_x-left_stick_y-right_stick_x));
        leftFront.setPower(lfDefDir*s*(left_stick_x-left_stick_y+right_stick_x));
        telemetry.addData("rightRear", rightRear.getPower());
        telemetry.addData("leftRear", leftRear.getPower());
        telemetry.addData("rightFront", rightFront.getPower());
        telemetry.addData("leftFront", leftFront.getPower());
    }
    public void posStraight(float position, int velocity, int direction,float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStraightSize),velocity,direction,direction,direction,direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void posStrafe(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStrafeSize),velocity,-direction,direction,direction,-direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void posTurn(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveTurnSize),velocity,-direction,direction,-direction,direction);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void posJoystick(float position, int velocity, float left_stick_x,float left_stick_y, float right_stick_x,float wait) {
        if (position < 0) {
            position = Math.abs(position);
            left_stick_x = left_stick_x * -1;
            right_stick_x = right_stick_x * -1;
            left_stick_y = left_stick_y * -1;
        }
        float rbDir = ((left_stick_x-left_stick_y-right_stick_x));
        float lbDir = ((-left_stick_x-left_stick_y+right_stick_x));
        float rfDir = ((-left_stick_x-left_stick_y-right_stick_x));
        float lfDir = ((left_stick_x-left_stick_y+right_stick_x));
        posDrive(Math.round(position*posDriveStraightSize),velocity,rfDir,lfDir,rbDir,lbDir);
        try {
            sleep(Math.round(position*wait*posDriveWait));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void posDrive(int position, int velocity,float rfDir, float lfDir, float rbDir, float lbDir) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setTargetPosition(Math.round(position * rfDir * rfDefDir));
        leftFront.setTargetPosition(Math.round(position * lfDir * lfDefDir));
        rightRear.setTargetPosition(Math.round(position * rbDir * rbDefDir));
        leftRear.setTargetPosition(Math.round(position * lbDir * lbDefDir));

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setTargetPositionTolerance(tolerance);
        leftFront.setTargetPositionTolerance(tolerance);
        rightRear.setTargetPositionTolerance(tolerance);
        leftRear.setTargetPositionTolerance(tolerance);

        rightFront.setVelocity(velocity);
        leftFront.setVelocity(velocity);
        rightRear.setVelocity(velocity);
        leftRear.setVelocity(velocity);
    }
    public void posSlide(int position, int velocity) {
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide.setTargetPosition(position);
        leftSlide.setTargetPosition(position);

        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setTargetPositionTolerance(tolerance);
        leftSlide.setTargetPositionTolerance(tolerance);

        rightSlide.setVelocity(velocity);
        leftSlide.setVelocity(velocity);
    }
    public boolean limelight_read() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (!result.getFiducialResults().isEmpty()) {
                current_tag = result.getFiducialResults().get(0).getFiducialId();
                return true;
            }
        }
        return false;
    }
    public boolean limelight_target(boolean go,float starting_yaw) {
        //STARTING YAW USES RACING AWAY FROM RED GOAL = 0, SO AUTOBR USES 180
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LLResult result = limelight.getLatestResult();
        float lx = 0;
        float ly = 0;
        float rx = 0;
        telemetry.addData("result exists",result != null);
        if (result != null && !result.getFiducialResults().isEmpty()) {
            float target_ty = 18.5f;
            float target_tx = 2.1f;
            float target_yaw = (180 - starting_yaw);
            telemetry.addData("lastTag",last_tag);
            telemetry.addData("target_yaw",target_yaw);
            double tx_var = 0.5;
            double ty_var = 0.5;
            double yaw_var = 1.2;
            double l_speed = 0.5;
            if (result.isValid()) {
                last_tag = result.getFiducialResults().get(0).getFiducialId();
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("result valid",true);
                telemetry.addData("fiducial",result.getFiducialResults().get(0).getFiducialId());
                if (go) {
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

                    power_dual_joy_control(lx,ly,rx,0,l_speed);

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
                    power_dual_joy_control(lx,ly,rx,0,l_speed);
                }
                telemetry.addData("result valid",false);
            }
        }
        if (lx != 0 || ly != 0 || rx != 0) {
            return true;
        }
        power_dual_joy_control(0,0,0,0,0);
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
    public void update_imu() {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robot_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        robot_pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        robot_roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        telemetry.addData("yaw",robot_yaw);
        telemetry.addData("pitch",robot_pitch);
        telemetry.addData("roll",robot_roll);
        telemetry.addData("leftRearPos",leftRear.getCurrentPosition());
        telemetry.addData("leftFrontPos",leftFront.getCurrentPosition());
        telemetry.addData("rightFrontPos",rightFront.getCurrentPosition());
        telemetry.addData("rightRearPos",rightRear.getCurrentPosition());
    }

}

