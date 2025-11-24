package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

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

public class AutoLimeLightBR extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    @Override
    public void init() {
        zkely_init();
        vel = 1200;
    }
    @Override

    public void start() {
        //START INTAKE FACING GOAL

        //move 1 diagonal tile back
        posStraight((float) Math.sqrt(2.0d), vel, -1, true);

        //NEEDED:
        //move things through the body

        //shoot
        while (limelight_target(true,180)) {
            update_imu();
        }
        while (limelight_target(true,180)) {
            update_imu();
        }
        outtake.setPower(-max_outtake_power);
        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        outtake.setPower(0);

        //look at obelisk
        posTurn(1,vel,-1,true);
        while (!limelight_read()) {
            update_imu();
        }


        //NEEDED:
        //AprilTag code

        //turn towards the patterns, start intake
        posTurn(1.5f,vel,-1,true);
        intake.setPower(-1);

        //NEEDED:
        //if tag is other than 23 (PPG), move straight for 1 (22/PGP) or 2 (21/GPP) tiles
        posStraight(23-current_tag,vel,1,true);

        //turn into pieces and intake them
        posJoystick(1,vel,0,-1,-1,true);
        posStraight(0.5f,vel,1,true);
        intake.setPower(0);

        //set up for scoring
        if (current_tag == 23) {
            posTurn(0.5f, vel, -1, true);
            posStrafe(1, vel, -1, true);
            posStraight(0.5f, vel, -1, true);
        } else if (current_tag == 22) {
            posTurn(0.5f, vel, -1, true);
            posStrafe(1.5f, vel, -1, true);
        } else if (current_tag == 21) {
            //NEED TO DO THIS WHEN BACK
            //DO NOT RUN AROUND THINGS WHICH SHOULDN'T BE HIT
            posTurn(0.5f, vel, -1, true);
            posStrafe(1, vel, -1, true);
            posJoystick(1.5f,vel,-0.7f,-0.7f,0,true);
        }
        //fine tune position
        while (limelight_target(true,180)) {
            update_imu();
        }
        while (limelight_target(true,180)) {
            update_imu();
        }

        //NEEDED:
        //move pieces forward
        outtake.setPower(-max_outtake_power);
        try {
            sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        outtake.setPower(0);

    }
    @Override
    public void loop() {
        update_imu();
        if (gamepad1.dpad_up) {
            posStraight(1,vel,1,true);
        }
        if (gamepad1.dpad_down) {
            posStraight(1,vel,-1,true);
        }
        if (gamepad1.dpad_right) {
            posStrafe(1,vel,1,true);
        }
        if (gamepad1.dpad_left) {
            posStrafe(1,vel,-1,true);
        }
        if (gamepad1.left_bumper) {
            posTurn(0.5f,vel,-1,true);
        }
        if (gamepad1.right_bumper) {
            posTurn(0.5f,vel,1,true);
        }
        if (gamepad1.left_trigger > 0.2) {
            posJoystick(1,vel, gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x,true);
        }
        telemetry.update();
    }



}

