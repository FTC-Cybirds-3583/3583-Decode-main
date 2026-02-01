package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous( name = "TestAuto")

public class TestAuto extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();
    int vel;
    float mod_amount = 10;
    double[] step_sizes = {10.0, 1.0, 0.1, 0.01};
    int step_index = 1;
    float ticks_to_stop = 0;
    int last_tag = 23;


    @Override
    public void runOpMode() throws InterruptedException {
        zkely_init();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadDownWasPressed()) {
                posJoystick(1,1500,0,1,0,0);
            }
            if (gamepad1.dpadUpWasPressed()) {
                posJoystick(1,1500,0,-1,0,0);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                posJoystick(1,1500,-1,0,0,0); //not enough by ~2in
            }
            if (gamepad1.dpadRightWasPressed()) {
                posJoystick(1,1500,1,0,0,0);
            }
            if (gamepad1.leftBumperWasPressed()) {
                posJoystick(1,1500,0,0,-1,0); //too much by a tinyyy bit
            }
            if (gamepad1.rightBumperWasPressed()) {
                posJoystick(1,1500,0,0,1,0);
            }
        }
    }
}

