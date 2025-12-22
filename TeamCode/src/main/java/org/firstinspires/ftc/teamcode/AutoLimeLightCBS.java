package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Blue-Shoot-CLOSE")

public class AutoLimeLightCBS extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    @Override
    public void init() {
        zkely_init();
        vel = 2000;
    }
    @Override

    public void start() {
        robot_starting_yaw = -130;
        team = "B";

        //move away from goal
        outtake.setPower(outtake_dir * close_max_outtake_power);
        intake.setPower(intake_dir * 1);
        posStraight(1.5f,(int) Math.floor(vel*1.5f),-1,0.75f);
        while (limelight_target(true,true)) {
            update_imu();
            telemetry.update();
        }
        sleepMS(500);
        shootAuto(true);
        stopShooting();

        //LOOKING AT OBELISK
        posTurn(0.65f,vel,1,0.9f);
        while (!limelight_read()) {
            update_imu();
            telemetry.addData("current_tag", current_tag);
            telemetry.update();
        }
        posTurn(0.65f,vel,-1,0.9f);

        //MOVING TO NEXT ROW
        if (current_tag == 23) {
            posJoystick(0.5f,vel,-0.85f,0,-0.9f,1);

            startIntake();
            posTurn((float) 0.0,vel,0,1);
            posStraight((float) 1.275,Math.round(vel*0.4f),1,1);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            outtake.setPower(outtake_dir * close_max_outtake_power);

            posJoystick(1.5f,vel,0.75f,0.75f,0.375f,1);
        } else if (current_tag == 22) {

            posJoystick(0.5f,vel,-0.85f,0,-0.9f,0.9f);
            posStrafe(1.05f,vel,-1,0.85f);

            startIntake();
            posTurn((float) 0.0,vel,0,1);
            posStraight((float) 1.5,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            outtake.setPower(outtake_dir * close_max_outtake_power);

            posStraight((float) 1.5,vel,-1,1);
            posStrafe(1.225f,vel,1,1);
            posTurn(0.5f,vel,1,1);


        } else if (current_tag == 21) {
            posJoystick(0.5f,vel,-0.85f,0,-0.9f,0.9f);
            posStrafe(2.15f,vel,-1,0.85f);

            startIntake();
            posTurn((float) 0.0,vel,0,1);
            posStraight((float) 1.5,Math.round(vel*0.4f),1,0.9f);
            stopIntake();
            intake.setPower(intake_dir * 1.0);
            outtake.setPower(outtake_dir * close_max_outtake_power);

            posStraight((float) 1.5,vel,-1,1);
            posStrafe(2.3f,vel,1,1);
            posTurn(0.5f,vel,1,1);

        }
        //FINAL SHOOT
        while (limelight_target(true,true)) {
            update_imu();
            telemetry.update();
        }
        shootAuto(true);
        posStrafe(1,vel,-1,0);
    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

