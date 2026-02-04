package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Map;

@Disabled
public abstract class Zkely extends LinearOpMode
{
    DcMotorEx rightRear;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx leftFront;
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    DcMotorEx intake;
    DcMotorEx outtake;
    IMU.Parameters myIMUparameters;
    IMU imu;
    YawPitchRollAngles robotOrientation;
    CRServo midtake;
    CRServo midtake_2;
    Servo innertake;

    PIDFCoefficients PosDriveCo = new PIDFCoefficients(10,0,0,10);
    float midtake_power = 1f;
    int manual_velocity = 1300;
    boolean manual = false;
    double innertake_up_pos = 0;
    double innertake_down_pos = 0.525;
    Pose3D last_botpose;
    int intake_dir = -1;
    int midtake_dir = 1;
    int outtake_dir = -1;
    int last_tag;
    int current_tag;
    int tolerance = 3;
    int rfDefDir = 1;
    int lfDefDir = 1;
    int rbDefDir = 1;
    int lbDefDir = 1;
    int posDriveWait= 1300;

    // Now use these simple methods to extract each angle
// (Java type double) from the object you just created:
    double robot_yaw;
    double robot_pitch;
    double robot_roll;
    Limelight3A limelight;
    int slide_up_pos = 2100;
    int slide_down_pos = -5;
    int slide_target_pos = 50;
    double speed;
    double speed_fine_inc = 0.05;
    int posDriveStraightSize = 715; // js about perfect // old 1000
    int posDriveStrafeSize = 785; // old 1075
    int posDriveTurnSize = 685; // old 960

    float outtake_power = 1.0f;
    float current_ty = 0;
    float current_tx = 0;
    static float robot_starting_yaw = 180;

    VoltageSensor voltageSensor;
    double currentVoltage;
    static String team = "N";

    float true_yaw = 0;
    float target_yaw = 0;
    float correction_angle = 0;
    float apriltag_distance = 0;
    float target_distance = 0;
    int outtake_velocity = 1200;
    boolean latest_result_valid = false;
    PIDFCoefficients outtake_pidf = new PIDFCoefficients(280,0,0,15);
    LLResultTypes.FiducialResult latest_fiducial;
    int tag_team = 0;

    int side_modifier = 1;
    float startingCurrent = 0;
    public void zkely_init() {

        current_tag = 23;
        tag_team = 0;

        rightRear = hardwareMap.get(DcMotorEx.class,"backright");
        leftRear = hardwareMap.get(DcMotorEx.class,"backleft");
        rightFront = hardwareMap.get(DcMotorEx.class,"frontright");
        leftFront = hardwareMap.get(DcMotorEx.class,"frontleft");

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        imu = hardwareMap.get(IMU.class, "imu");

        midtake = hardwareMap.crservo.get("midtake");
        midtake_2 = hardwareMap.crservo.get("midtake_2");
        innertake = hardwareMap.servo.get("innertake");

        innertake.setPosition(innertake_down_pos);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        startingCurrent = (float) voltageSensor.getVoltage();

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
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

        brakeSlides();

        //initialise limelight on pipeline 0
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        //set drive speed at 0.5 initially
        speed = 1;
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

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void brakeSlides() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
    public void power_dual_joy_control(float left_stick_x,float left_stick_y,float right_stick_x,float right_stick_y,double s) {
        //don't ask how this works, it is wonderful
        set_motor_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        set_motor_zero_power_behaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Math.abs(right_stick_x) > 0.5) {
            s = 1;
        }
        rightRear.setPower(Math.min(rbDefDir*s*(left_stick_x-left_stick_y-right_stick_x),1));
        leftRear.setPower(Math.min(lbDefDir*s*(-left_stick_x-left_stick_y+right_stick_x),1));
        rightFront.setPower(Math.min(rfDefDir*s*(-left_stick_x-left_stick_y-right_stick_x),1));
        leftFront.setPower(Math.min(lfDefDir*s*(left_stick_x-left_stick_y+right_stick_x),1));
    }
    public void shootAutoPower(boolean close) {
        float power = outtake_power;
        if (currentVoltage > 12.8) {
            power *= 0.8125;
        } else if (currentVoltage > 12.6) {
            power *= 0.8575;
        } else if (currentVoltage > 12.5) {
            power *= 0.935;
        } else if (currentVoltage > 11.7) {
            power *= 0.965;
        } else if (currentVoltage > 11.3) {
            power *= 0.975;
        }
        startShootingPower(power*0.97f);
        sleepMS(1300);
        outtake.setPower(outtake_dir * power);
        sleepMS(3700);
    }
    public void shootAutoVelocity() {
        run_updates();
        startShootingVelocity(outtake_velocity);
        sleepMS(3500);
        stopShooting();
    }
    public void startOuttakeVelocity(float velocity) {
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,outtake_pidf);
        outtake.setVelocity(velocity);
    }
    public void startShootingVelocity(float velocity) {
        telemetry.addData("start shoot vel",outtake_velocity);
        //innertake.setPosition(innertake_down_pos);
        midtake.setPower(midtake_dir * -1 * midtake_power);
        sleepMS(300);
        intake.setPower(intake_dir * 1);
        midtake.setPower(midtake_dir * midtake_power);
        midtake_2.setPower(midtake_dir * midtake_power);
        startOuttakeVelocity(velocity);

    }
    public void startShootingPower(float power) {
        innertake.setPosition(innertake_down_pos);
        midtake.setPower(midtake_dir * -1 * midtake_power);
        sleepMS(300);
        intake.setPower(intake_dir * 1);
        midtake.setPower(midtake_dir * midtake_power);
        midtake_2.setPower(midtake_dir * midtake_power);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setPower(outtake_dir * power);
    }
    public void stopShooting() {
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
        midtake.setPower(0);
        midtake_2.setPower(0);
        outtake.setPower(0);
    }
    public void startIntake() {
        innertake.setPosition(innertake_up_pos);
        sleepMS(300);
        intake.setPower(intake_dir * 1);
        midtake.setPower(midtake_dir * midtake_power);
        midtake_2.setPower(midtake_dir*-1*midtake_power);
    }
    public void stopIntake() {
        innertake.setPosition(innertake_down_pos);
        sleepMS(300);
        intake.setPower(0);
        midtake.setPower(0);
        midtake_2.setPower(0);
    }
    public void sleepMS(int ms) {
        sleep(ms);
    }
    public boolean motorBelowTolerance(DcMotorEx motor,float tolerance) {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < tolerance;
    }
    public boolean motorMoving(DcMotorEx motor,int tolerance) {
        return Math.abs(motor.getVelocity()) > tolerance;
    }
    public boolean any_drive_motor_busy(float tolerance) {
        //return rightRear.isBusy() || rightFront.isBusy() || leftRear.isBusy() || leftFront.isBusy();
        boolean bool = motorBelowTolerance(rightRear,tolerance) && motorBelowTolerance(leftRear,tolerance) && motorBelowTolerance(leftFront,tolerance) && motorBelowTolerance(rightFront,tolerance);
        return !bool;
    }
    public boolean any_drive_motor_moving() {
        int tolerance = 50;
        //return rightRear.isBusy() || rightFront.isBusy() || leftRear.isBusy() || leftFront.isBusy();
        boolean bool = motorMoving(rightRear,tolerance) || motorMoving(leftRear,tolerance) || motorMoving(leftFront,tolerance) || motorMoving(rightFront,tolerance);
        return bool;
    }
    public void posStraight(float position, int velocity, int direction,float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStraightSize),velocity,direction,direction,direction,direction,60);
    }
    public void posStrafe(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveStrafeSize),velocity,-direction,direction,direction,-direction,50);
    }
    public void posTurn(float position, int velocity, int direction, float wait) {
        if (position < 0) {
            position = Math.abs(position);
            direction = direction * -1;
        }
        posDrive(Math.round(position*posDriveTurnSize),velocity,-direction,direction,-direction,direction,45);
    }

    public void posJoystick(float position, int velocity, float left_stick_x,float left_stick_y, float right_stick_x,float wait) {

        if (position < 0) {
            position = Math.abs(position);
            left_stick_x = left_stick_x * -1;
            right_stick_x = right_stick_x * -1;
            left_stick_y = left_stick_y * -1;
        }
        left_stick_x *= (float) (posDriveStrafeSize+30) /posDriveStraightSize;
        right_stick_x *= (float) (posDriveTurnSize+15) /posDriveStraightSize;
        float rbDir = ((left_stick_x-left_stick_y-right_stick_x));
        float lbDir = ((-left_stick_x-left_stick_y+right_stick_x));
        float rfDir = ((-left_stick_x-left_stick_y-right_stick_x));
        float lfDir = ((left_stick_x-left_stick_y+right_stick_x));
        posDrive(Math.round(position*posDriveStraightSize),velocity,rfDir,lfDir,rbDir,lbDir,60);
    }

    public void posDrive(int position, int velocity,float rfDir, float lfDir, float rbDir, float lbDir,float stop_tolerance) {
        //float pos_battery_mod = (float) (0.107f*voltageSensor.getVoltage() - 0.325f);
        //float float_position = position * pos_battery_mod;
        //position = (int) Math.floor((double) float_position);

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

        rightFront.setPositionPIDFCoefficients(PosDriveCo.p);
        leftFront.setPositionPIDFCoefficients(PosDriveCo.p);
        rightRear.setPositionPIDFCoefficients(PosDriveCo.p);
        leftRear.setPositionPIDFCoefficients(PosDriveCo.p);

        rightFront.setVelocityPIDFCoefficients(PosDriveCo.p,PosDriveCo.i,PosDriveCo.d,PosDriveCo.f);
        leftFront.setVelocityPIDFCoefficients(PosDriveCo.p,PosDriveCo.i,PosDriveCo.d,PosDriveCo.f);
        rightRear.setVelocityPIDFCoefficients(PosDriveCo.p,PosDriveCo.i,PosDriveCo.d,PosDriveCo.f);
        leftRear.setVelocityPIDFCoefficients(PosDriveCo.p,PosDriveCo.i,PosDriveCo.d,PosDriveCo.f);

        rightFront.setTargetPositionTolerance(tolerance);
        leftFront.setTargetPositionTolerance(tolerance);
        rightRear.setTargetPositionTolerance(tolerance);
        leftRear.setTargetPositionTolerance(tolerance);

        rightFront.setVelocity(velocity);
        leftFront.setVelocity(velocity);
        rightRear.setVelocity(velocity);
        leftRear.setVelocity(velocity);
        ElapsedTime time_since_posdrive = new ElapsedTime();
        while (opModeIsActive() && any_drive_motor_busy(stop_tolerance)) {
            if (time_since_posdrive.milliseconds() > (double) (posDriveWait * position)/750 + 500) {
                break;
            }
        }
        sleepMS(150);
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
    public void posOuttake(int position, int velocity) {
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake.setTargetPosition(position);

        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtake.setTargetPositionTolerance(tolerance);

        outtake.setVelocity(velocity);
    }

    public boolean limelight_read() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (!result.getFiducialResults().isEmpty()) {
                current_tag = result.getFiducialResults().get(0).getFiducialId();
                return false;
            }
        }
        return true;
    }
    public float limelight_teleop_circle(boolean go,float tolerance) {
        if (!limelight.getLatestResult().isValid() || !go) { return -1; };
        if (latest_fiducial.getFiducialId() != 20 && latest_fiducial.getFiducialId() != 24) { return -2; };
        float tx_stick = Math.signum(correction_angle);

        tx_stick *= Math.abs(map(0f, 1, 0, 35, Math.abs(correction_angle)));
        if (Math.abs(tx_stick) < tolerance) {
            tx_stick = 0;
            set_motor_powers(0);
        }
        power_dual_joy_control(gamepad1.left_stick_x, gamepad1.left_stick_y, tx_stick, gamepad1.right_stick_y, speed);

        return Math.abs(tx_stick);
    }
    public boolean new_limelight_target() {
        run_updates();
        if (current_tag != 20 && current_tag != 24) { return true; };

        set_motor_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        set_motor_zero_power_behaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        float ty_target = 6f;
        float tx_target = 0;
        float tag_yaw = 130;
        if (current_tag == 20) { tag_yaw = -130; }

        float rx = Math.signum(true_yaw - tag_yaw);
        float ly = Math.signum(current_ty-ty_target);
        float lx = Math.signum(current_tx );

        rx *= Math.abs(map(0.04f, 1, 0, 70, Math.abs(angle_distance(tag_yaw,true_yaw))));
        ly *= Math.abs(map(0.02f, 1, 0, 8, Math.abs(current_ty-ty_target)));
        lx *= Math.abs(map(0.02f, 1, 0, 10, Math.abs(current_tx-tx_target)));

        if (Math.abs(rx) < 0.1f) { rx = 0;}
        if (Math.abs(ly) < 0.1f) { ly = 0;}
        if (Math.abs(lx) < 0.1f) { lx = 0;}

        power_dual_joy_control(lx,ly,rx,0,1);

        if (rx == 0 && ly == 0 && lx == 0) {
            return false;
        }
        return true;
    }

    public boolean limelight_target(boolean go,boolean close) {
        //STARTING YAW USES RACING AWAY FROM RED GOAL = 0, SO AUTOBR USES 180
        set_motor_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        set_motor_zero_power_behaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        LLResult result = limelight.getLatestResult();
        float lx = 0;
        float ly = 0;
        float rx = 0;
        if (result != null && !result.getFiducialResults().isEmpty()) {
            float target_ty = 12f;
            float target_tx = 4f;
            float target_yaw = 130;
            if (close == true) {
                if (team == "R") {
                    target_yaw = 131.5f ;
                } else if (team == "B") {
                    target_yaw = -130;
                }
            } else {
                if (team == "R") {
                    target_yaw = 155;
                } else {
                    target_yaw = -155;
                }
            }
            if (team == "R") {
                target_tx = 5.3f;
            } else {
                target_tx = 8.5f;
            }
            double tx_var = 0.175;
            double ty_var = 0.25;
            double yaw_var = 1.25;
            double l_speed = 0.4;
            if (result.isValid()) {
                telemetry.addData("tx",result.getTx());
                last_tag = result.getFiducialResults().get(0).getFiducialId();
                Pose3D botpose = result.getBotpose();
                telemetry.addData("Botpose", botpose.toString());

                true_yaw = (float) botpose.getOrientation().getYaw();

                telemetry.addData("fiducial",result.getFiducialResults().get(0).getFiducialId());
                if (go) {
                    telemetry.addData("fid0", result.getFiducialResults().get(0).getTargetPoseCameraSpace());

                    if (result.getTy() < target_ty-ty_var) {
                        ly= -1;
                    } else if (result.getTy() > target_ty + ty_var) {
                        ly = 1;
                    }
                    if ((float) Math.abs(result.getTy() - target_ty) < 8) {
                        ly = ly * (float) Math.abs(map(0.2f, 1, 0, 8, (float) Math.abs(result.getTy() - target_ty)));
                    }

                    if (result.getTx() > target_tx+tx_var) {
                        lx = 1;
                    } else if (result.getTx() < target_tx-tx_var) {
                        lx = -1;
                    }
                    if ((float) Math.abs(result.getTx() - target_tx) < 8) {
                        lx = lx * (float) Math.abs(map(0.2f, 1, 0, 8, (float) Math.abs(result.getTx() - target_tx)));
                    }

                    if (Math.abs(angle_distance((float) true_yaw,target_yaw)) > yaw_var) {
                        telemetry.addData("LESS THAN YAW VAR", true);
                        if (angle_distance(true_yaw,target_yaw) > 0) {
                            rx = -1;
                        } else {
                            rx = 1;
                        }
                    } else {
                        rx = 0;
                    }

                    if (angle_distance((float) true_yaw,target_yaw) < 45) {
                        rx = rx * (float) Math.abs(map(0.1f, 1, 0, 45, (float) Math.abs(angle_distance((float) true_yaw,target_yaw))));
                    }
                    telemetry.addData("lx",lx);
                    telemetry.addData("rx",rx);
                    if (Math.abs(lx) < 0.275f && Math.abs(rx) < 0.175f) {
                        lx = 0;
                        rx = 0;
                    }

                    //this is sketchy, update later
                    if (!close) {
                        lx = 0;
                        ly = 0;
                    }

                    power_dual_joy_control(lx,ly,rx,0,l_speed);

                }
                last_botpose = botpose;
            } else {
                if (last_botpose != null && go) {
                    l_speed = 0.75d;
                    if (Math.abs(angle_distance((float) true_yaw,target_yaw) )> yaw_var) {
                        if (angle_distance(true_yaw,target_yaw) > 0) {
                            rx = -1;
                        } else {
                            rx = 1;
                        }
                    } else {
                        rx = 0;
                    }

                    if (angle_distance((float) true_yaw,target_yaw) < 50) {
                        if (close) {
                            rx = rx * 0.2f;
                        } else {
                            rx = rx * 0.7f;
                        }
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
    public void set_motor_modes(DcMotor.RunMode mode) {
        rightRear.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        leftFront.setMode(mode);
    }
    public void set_motor_powers(double power) {
        rightRear.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
    }
    public void set_motor_zero_power_behaviour(DcMotor.ZeroPowerBehavior behaviour) {
        rightRear.setZeroPowerBehavior(behaviour);
        leftRear.setZeroPowerBehavior(behaviour);
        rightFront.setZeroPowerBehavior(behaviour);
        leftFront.setZeroPowerBehavior(behaviour);
    }

    public float angle_distance(float angle,float target) {
        return (target - angle + 180) % 360 - 180;
    }
    public float map(float min_out, float max_out,float min_in,float max_in,float val_in) {
        return min_out + ((max_out-min_out) / (max_in - min_in)) * (val_in - min_in);
    }

    public float apriltag_distance() {
        double limelight_angle_deg = 7.675;

        double apriltag_height_in = 29.5f;
        double limelight_height_in = 14.75f;
        double height_total_in = apriltag_height_in - limelight_height_in;

        double angle_total_deg = limelight_angle_deg + current_ty;

        double distance = height_total_in * (Math.cos(Math.toRadians(angle_total_deg)))/Math.sin(Math.toRadians(angle_total_deg));
        return (float) distance;
    }
    public float correction_angle() {
        if (!latest_result_valid) {
            return 0;
        }
        float distance = apriltag_distance;
        float tx_rad = (float) Math.toRadians(current_tx);
        float yaw_rad = (float) Math.toRadians(target_yaw);
        float tag_to_goal_in = 15;
        float x1 = (float) distance * (float) Math.sin(tx_rad);
        float y1 = (float) distance * (float) Math.cos(tx_rad);
        float x2 = tag_to_goal_in*(float) Math.sin(yaw_rad) + x1;
        float y2 = tag_to_goal_in*(float) Math.cos(yaw_rad) + y1;
        float angle = (float) Math.toDegrees(Math.atan((float) x2/y2));
        target_distance = (float) Math.sqrt(Math.pow(x2,2)+Math.pow(y2,2));
        return angle;
    }
    public void update_imu() {

        robotOrientation = imu.getRobotYawPitchRollAngles();
        robot_yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        robot_pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        robot_roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        true_yaw = (float) (robot_yaw + robot_starting_yaw);
        LLResult result = limelight.getLatestResult();
        latest_result_valid = result.isValid();
        if (result != null && !result.getFiducialResults().isEmpty()) {

            if (latest_result_valid) {
                Pose3D botpose = result.getBotpose();
                true_yaw = (float) botpose.getOrientation().getYaw();
                current_ty = (float) result.getTy();
                current_tx = (float) result.getTx();
                current_tag = result.getFiducialResults().get(0).getFiducialId();
                latest_fiducial = result.getFiducialResults().get(0);
                target_yaw = (float) result.getFiducialResults().get(0).getTargetPoseRobotSpace().getOrientation().getPitch();
            }
        }
        apriltag_distance = apriltag_distance();

        midtake_power = (float) (1.135f - 0.0019f * apriltag_distance);
        if (voltageSensor.getVoltage() < 12.9f) { midtake_power += (float) ((13-voltageSensor.getVoltage()) * 0.055); }
        if (midtake_power > 1) { midtake_power = 1; }

        if (true_yaw > 180) {
            true_yaw = true_yaw - 360;
        }
        if (true_yaw < -180) {
            true_yaw = true_yaw + 360;
        }

        currentVoltage = voltageSensor.getVoltage();

        robot_starting_yaw = (float) (true_yaw-robot_yaw);
        telemetry.addData("midtake power",midtake_power);
        correction_angle = correction_angle();
    }

    public void update_outtake_power() {
        if (!latest_result_valid) {
            outtake_power = 0.45f;
            return;
        }
        float voltage_threshold = 12f;
        float voltage_variation= 0.018f;

        outtake_power = 0.003f * apriltag_distance + 0.315f; // perfect at -- 12.2 --
        outtake_power += (float) ((voltage_threshold-currentVoltage)*voltage_variation);
    }

    public void update_outtake_velocity() {
        //DISTANCE : VELOCITY
        //82 : 1225
        //200 : 1525
        if (manual) {
            outtake_velocity = manual_velocity;
            return;
        }
        if (!latest_result_valid) {
            return;
        }

        double a = 4;
        double b = 925;
        outtake_velocity = (int) (a * target_distance + b );

    }

    public void update_outtake_pidf() {
        float P = 275; //275
        //if (apriltag_distance > 140) {
        //    P = 275;
        //}
        telemetry.addData("P",P);

        float F =  (float) (-449.47321*Math.pow(outtake_velocity,-0.619854f)+48.96472-(2.35*currentVoltage));
        telemetry.addData("F",F);
        PIDFCoefficients pidfCoef = new PIDFCoefficients(P, 0.0 ,0.005,F);
        outtake_pidf = pidfCoef;
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoef);
    }
    public void run_updates() {
        update_imu();
        update_outtake_velocity();
        update_outtake_power();
        update_outtake_pidf();
    }
    public void new_limelight_target_auto() {
        ElapsedTime time_since_start = new ElapsedTime();
        while (new_limelight_target() && opModeIsActive()) {
            if (time_since_start.milliseconds() > 2000) {
                break;
            }
        }
        set_motor_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        set_motor_zero_power_behaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        set_motor_powers(0);

    }
    public void limelight_read_auto() {
        while (limelight_read() && opModeIsActive()) {
        }
    }
    public void start_outtake_velocity_auto() {
        run_updates();
        startOuttakeVelocity(outtake_velocity);
    }

    public void set_tag_team() {
        tag_team = current_tag;
        if (tag_team == 20) {
            side_modifier = -1;
        } else if (tag_team == 24) {
            side_modifier = 1;
        }
    }

    public void limit_power() {
        if (rightFront.getCurrent(CurrentUnit.MILLIAMPS) < 500) {
            rightFront.setPower(0);
        }
        if (leftFront.getCurrent(CurrentUnit.MILLIAMPS) < 500) {
            leftFront.setPower(0);
        }
        if (rightRear.getCurrent(CurrentUnit.MILLIAMPS) < 500) {
            rightRear.setPower(0);
        }
        if (leftRear.getCurrent(CurrentUnit.MILLIAMPS) < 500) {
            leftRear.setPower(0);
        }
    }

    public boolean motor_at_velocity(DcMotorEx motor,int target_velocity,float tolerance) {
        boolean result = false;
        if (Math.abs(motor.getVelocity() - target_velocity) <= tolerance) {
            result = true;
        }
        return result;

    }

}

