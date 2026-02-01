package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous( name = "Close-Obelisk")

public class CloseAutoObelisk extends MainAuto {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    float mod_amount = 10;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01};
    int step_index = 1;
    float ticks_to_stop = 0;
    boolean check_obelisk;

    @Override

    public void runOpMode() {
        closeAuto(true);
    }

}

