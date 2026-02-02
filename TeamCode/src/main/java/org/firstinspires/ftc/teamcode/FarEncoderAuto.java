package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous( name = "Far-Encoder")

public class FarEncoderAuto extends MainEncoderAuto {
    @Override

    public void runOpMode() {
        farAuto();
    }

}

