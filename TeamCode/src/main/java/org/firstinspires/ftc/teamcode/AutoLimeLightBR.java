package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

@Autonomous( name = "AutoLimeLightBR")

public class AutoLimeLightBR extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

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
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
        //set drive speed at 0.5 initially
        speed = 0.5;
        //initialise bumpers as "not pressed"
    }

    @Override

    public void loop() {

    }
    


}

