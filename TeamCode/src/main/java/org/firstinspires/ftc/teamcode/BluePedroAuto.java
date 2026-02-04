package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous( name = "Blue Pedro")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class BluePedroAuto extends MainPedroAuto {

    @Override
    public void runOpMode() {
        run(false);
    }
}

