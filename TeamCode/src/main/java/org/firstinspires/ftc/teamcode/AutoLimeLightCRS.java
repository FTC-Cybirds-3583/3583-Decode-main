package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Red-Shoot-CLOSE")

public class AutoLimeLightCRS extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    @Override
    public void runOpMode() {
        zkely_init();
        vel = 2000;
        waitForStart();
        robot_starting_yaw = 130;
        team = "R";

        //move away from goal
        outtake.setPower(outtake_dir * outtake_power);
        intake.setPower(intake_dir * 1);
        posStraight(1.75f,(int) Math.floor(vel*1.25f),-1,0.75f);
        while (limelight_target(true,true) && opModeIsActive()) {
            update_imu();
            telemetry.update();
        }
        sleepMS(500);
        shootAuto(true);
        stopShooting();

        //LOOKING AT OBELISK
        posTurn(0.65f,vel,-1,0.9f);
        while (!limelight_read() && opModeIsActive()) {
            update_imu();
            telemetry.addData("current_tag", current_tag);
            telemetry.update();
        }
        posTurn(0.65f,vel,1,0.9f);

        //MOVING TO NEXT ROW

        posJoystick(0.9f,vel,0.715f,0,0.475f,1);
        if (current_tag == 23) {
            //posStrafe(0.2f,vel,-1,0.9f);

            startIntake();
            posStraight((float) 1.2f,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            outtake.setPower(outtake_dir * outtake_power);
            posJoystick(0.9f,vel,-1,1.2f,-0.475f,1);
            midtake_2.setPower(0);
        } else if (current_tag == 22) {

            posStrafe(1.12f,vel,1,0.85f);

            startIntake();
            posStraight((float) 1.5f,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            outtake.setPower(outtake_dir * outtake_power);

            posStraight((float) 1.5f,vel,-1,0.85f);
            midtake_2.setPower(0);
            posStrafe(1.25f,vel,-1,0.8f);
            posTurn(0.5f,vel,-1,0.75f);


        } else if (current_tag == 21) {
            posStrafe(2.1f,vel,1,0.85f);

            startIntake();
            posStraight((float) 1.4f,Math.round(vel*0.4f),1,0.75f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            outtake.setPower(outtake_dir * outtake_power);

            posStraight((float) 1.4f,vel,-1,0.9f);
            midtake_2.setPower(0);
            posStrafe(2.1f,vel,-1,0.8f);
            posTurn(0.5f,vel,-1,0.9f);

        }
        //FINAL SHOOT
        while (limelight_target(true,true) && opModeIsActive()) {
            update_imu();
            telemetry.update();
        }
        shootAuto(true);
        posStrafe(1,(int) Math.floor(vel*1.5f),1,0);
    }



}

