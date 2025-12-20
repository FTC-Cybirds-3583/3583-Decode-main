package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Auto-GEN-Test")

public class AutoGENTest extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    @Override
    public void init() {
        zkely_init();
        vel = 1500;
    }
    @Override

    public void start() {
        //initial pos: (0.352941, 3.529412)
//initial rot: 90.0
        team = "B";
        robot_starting_yaw = 90;
//1 Move
        posTurn((float) 0.0,vel,0,1);
        posStraight((float) 4.65,vel,1,1);
//2 Move
        posTurn((float) 0.4,vel,1,1);
        posStraight((float) 2.48,vel,-1,1);

    }
    @Override
    public void loop() {
        update_imu();
        telemetry.update();
    }



}

