package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "Blue Pedro STOPS AFTER 6")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class BluePedroAutoNo22 extends MainPedroAuto {

    @Override
    public void runOpMode() {
        run(false,true,true);
    }
}

