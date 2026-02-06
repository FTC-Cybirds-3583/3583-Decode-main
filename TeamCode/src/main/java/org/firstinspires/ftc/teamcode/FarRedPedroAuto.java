package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "FAR Red Pedro")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class FarRedPedroAuto extends MainPedroAuto {

    @Override
    public void runOpMode() {
        run(true,false);
    }
}

