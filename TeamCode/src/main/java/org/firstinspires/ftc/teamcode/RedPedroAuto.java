package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous( name = "Red Pedro")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class RedPedroAuto extends MainPedroAuto {

    @Override
    public void runOpMode() {
        run(true);
    }
}

