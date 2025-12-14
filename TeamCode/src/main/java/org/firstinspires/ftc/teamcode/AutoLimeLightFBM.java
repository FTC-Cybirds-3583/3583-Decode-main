package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Blue-Move-FAR")

public class AutoLimeLightFBM extends Zkely {
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

        posStrafe(1.15f,vel,-1,1);
        posTurn(1,vel,-1,1);
    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

