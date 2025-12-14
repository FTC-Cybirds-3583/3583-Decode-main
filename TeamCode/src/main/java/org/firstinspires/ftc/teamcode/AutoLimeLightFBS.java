package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Blue-Shoot-FAR")

public class AutoLimeLightFBS extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    @Override
    public void init() {
        zkely_init();
        vel = 1500;
    }
    @Override

    public void start() {
        robot_starting_yaw = -180;
        team = "B";

        outtake.setPower(outtake_dir * far_max_outtake_power);
        //aim
        posStraight(0.3f,vel,1,1);
        posTurn((float) 1 /3,vel,-1,1.1f);

        //double check aiming
        while (yaw_target(-152)) {
            update_imu();
            telemetry.update();
        }

        try {
            sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        //shoot preloaded
        try {
            startShooting(far_max_outtake_power);
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }
        try {
            sleep(6000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        try {
            stopShooting();
        } catch (InterruptedException e) {
            telemetry.addData("e",e);
        }

        //move out of zone
        posTurn((float) 1 /3,vel,1,1.1f);
        posStraight(0.3f,vel,-1,1);
        posStrafe(1.15f,vel,-1,1);
        posTurn(1,vel,-1,1);
    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

