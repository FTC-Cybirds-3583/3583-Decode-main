package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Blue-Move-CLOSE")

public class AutoLimeLightCBM extends Zkely {
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

        posStraight(1.4f,vel,-1,1);
        posTurn(0.5f,vel,1,1);
        posStrafe(1,vel, -1,1);
    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

