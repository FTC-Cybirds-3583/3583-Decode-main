package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous( name = "ZZBlue-Shoot-CLOSE")

public class AutoLimeLightCBS extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;

    @Override

    public void runOpMode() {

        zkely_init();
        vel = 2000;

        waitForStart();

        robot_starting_yaw = -130;
        team = "B";

        //move away from goal
        startOuttakeVelocity(outtake_velocity);
        intake.setPower(intake_dir * 1);
        posStraight(1.75f,(int) Math.floor(vel*1.25f),-1,0.75f);
        while (limelight_target(true,true) && opModeIsActive()) {
            run_updates();
            telemetry.update();
        }
        sleepMS(500);
        shootAutoVelocity();
        stopShooting();

        //LOOKING AT OBELISK
        posTurn(0.7f,vel,1,0.9f);
        while (!limelight_read() && opModeIsActive()) {
            run_updates();
            telemetry.addData("current_tag", current_tag);
            telemetry.update();
        }
        posTurn(0.7f,vel,-1,0.9f);

        //MOVING TO NEXT ROW
        posJoystick(0.9f,vel,-0.81f,0,-0.46f,0.9f);
        if (current_tag == 23) {
            //posStrafe(0.2f,vel,-1,0.9f);

            startIntake();
            posStraight((float) 1f,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            startOuttakeVelocity(outtake_velocity);
            posJoystick(0.9f,vel,1,1.2f,0.475f,1);
            midtake_2.setPower(0 * midtake_power);
        } else if (current_tag == 22) {

            posStrafe(1.1f,vel,-1,0.85f);

            startIntake();
            posStraight((float) 1.3,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            startOuttakeVelocity(outtake_velocity);
            posStraight((float) 1.3,vel,-1,0.9f);
            midtake_2.setPower(0);
            posStrafe(1,vel,1,0.8f);
            posTurn(0.5f,vel,1,0.75f);


        } else if (current_tag == 21) {
            //posJoystick(0.5f,vel,-0.85f,0,-0.9f,0.9f);
            posStrafe(2.1f,vel,-1,0.85f);

            startIntake();
            posStraight((float) 1.3,Math.round(vel*0.4f),1,0.825f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            midtake_2.setPower(-1f * midtake_dir * midtake_power);
            midtake.setPower(-0.8f * midtake_dir * midtake_power);
            startOuttakeVelocity(outtake_velocity);

            posStraight((float) 1.3,vel,-1,0.8f);
            midtake_2.setPower(0);
            posStrafe(2.1f,vel,1,0.8f);
            posTurn(0.5f,vel,1,0.7f);

        }
        //FINAL SHOOT
        while (limelight_target(true,true) && opModeIsActive()) {
            run_updates();
            telemetry.update();
        }
        shootAutoVelocity();
        posStrafe(1,(int) Math.floor(vel*1.5f),-1,0);
    }



}

