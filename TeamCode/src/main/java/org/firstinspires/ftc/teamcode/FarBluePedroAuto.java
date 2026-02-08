package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous( name = "FAR Blue Pedro")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class FarBluePedroAuto extends MainPedroAuto {

    @Override
    public void runOpMode() {
        run(false,false,false);
    }
}

