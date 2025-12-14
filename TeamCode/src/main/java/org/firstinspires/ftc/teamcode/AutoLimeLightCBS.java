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
        vel = 1500;
    }
    @Override

    public void start() {
        robot_starting_yaw = -130;
        team = "B";

        //move away from goal
        outtake.setPower(outtake_dir * close_max_outtake_power);
        intake.setPower(intake_dir * 1);
        posStraight(1.7f,vel,-1,0.75f);
        while (limelight_target(true,true)) {
            update_imu();
            telemetry.update();
        }
        try {
            startShooting(close_max_outtake_power);
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }
        try {
            stopShooting();
        } catch (InterruptedException e) {
            telemetry.addData("e",e);

        }

        //LOOKING AT OBELISK
        posTurn(0.5f,vel,1,0.9f);
        while (!limelight_read()) {
            update_imu();
            telemetry.addData("current_tag", current_tag);
            telemetry.update();
        }
        posTurn(0.485f,vel,-1,0.9f);

        //MOVING TO NEXT ROW
        if (current_tag == 23) {
            //PPG
            posJoystick(0.45f, vel, 0, 1, -1, 1);
            try {
                startIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e", e);

            }
            posStrafe(0.2f, vel, -1, 1);
            posStraight(0.75f,vel,1,0.85f);
            posStraight(1.3f,Math.round(vel*0.5f),1,0.9f);
            try {
                stopIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e",e);
            }
            //WORKS UP UNTIL COLLECTION
            intake.setPower(intake_dir);
            posTurn(0.5f,vel,1,1);
            outtake.setPower(outtake_dir * close_max_outtake_power);
            posJoystick(1,vel,1,1,0,0.85f);
        } else if (current_tag == 22) {
            //PGP
            posTurn(0.5f,vel,-1,0.8f);
            posStrafe(1.35f,vel,-1,1);
            //INTAKE
            try {
                startIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e", e);

            }
            posStraight(0.75f,vel,1,0.85f);
            posStraight(1.3f,Math.round(vel*0.5f),1,0.9f);
            try {
                stopIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e",e);
            }
            //WORKS UP UNTIL COLLECTION
            intake.setPower(intake_dir);
            posStraight(2.05f,vel,-1,0.8f);
            outtake.setPower(outtake_dir * close_max_outtake_power);
            posStrafe(1.35f,vel,1,1);
            posTurn(0.5f,vel,1,1);
        } else if (current_tag == 21) {
            //PGP
            posTurn(0.5f,vel,-1,0.8f);
            posStrafe(2.45f,vel,-1,0.65f);
            //INTAKE
            try {
                startIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e", e);

            }
            posStraight(0.75f,vel,1,0.85f);
            posStraight(1.3f,Math.round(vel*0.5f),1,0.9f);
            try {
                stopIntake();
            } catch (InterruptedException e) {
                telemetry.addData("e",e);
            }
            //WORKS UP UNTIL COLLECTION
            intake.setPower(intake_dir);
            posStraight(2.05f,vel,-1,0.75f);
            outtake.setPower(outtake_dir * close_max_outtake_power);
            posStrafe(2.35f,vel,1,0.9f);
            posTurn(0.5f,vel,1,0.8f);
        }
        //FINAL SHOOT
        while (limelight_target(true,true)) {
            update_imu();
            telemetry.update();
        }
        try {
            startShooting(close_max_outtake_power + 0.025f);
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }
        try {
            sleep(5000);
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }
        posStrafe(1,vel,-1,0);
    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

