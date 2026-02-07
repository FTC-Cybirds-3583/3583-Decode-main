package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous( name = "Main Pedro")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class MainPedroAuto extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();

    //facing ramp. facing goal is 144 degrees
    Pose startingPose = new Pose(17.2722,121.4214,Math.toRadians(234));
    Pose sideAimingPose = new Pose(54,110,Math.toRadians(150));
    //144
    Pose aimingPose = new Pose(48,96,Math.toRadians(146));
    Pose intakePose23 = new Pose(47,84,Math.toRadians(175));
    Pose intakeDonePose23 = new Pose(20,84,Math.toRadians(180));
    Pose intakePose22 = new Pose(47,62,Math.toRadians(185));
    Pose intakeDonePose22 = new Pose(15,58,Math.toRadians(180));
    Pose leavePose = new Pose(36,84,Math.toRadians(144));
    Pose gatePose = new Pose(11, 60, Math.toRadians(150));
    Pose preGatePose = new Pose(22, 60, Math.toRadians(180));
    Pose telePreGatePose = new Pose(30, 72, Math.toRadians(180));

    Pose farStartingPose = new Pose(57,9,Math.toRadians(90));
    Pose farAimingPose = new Pose(60,18,Math.toRadians(114));
    Pose farLeavePose = new Pose(36,10,Math.toRadians(90));
    Pose intakePose21 = new Pose(47,34,Math.toRadians(185));
    Pose intakeDonePose21 = new Pose(15,36,Math.toRadians(180));
    //REMEMBER TO ADD REVERSE CLAUSE FOR NEW POSES
    Follower follower;
    int pathState = 0;
    Timer pathTimer, actionTimer, opmodeTimer;
    PathChain moveBack,
            moveIntake23,moveDoneIntake23,moveToGoal23,
            moveIntake22,moveDoneIntake22,moveToGoal22,
            moveIntake21,moveDoneIntake21,moveToGoal21,
            moveToPreGate,moveToTelePreGate,moveToGate,moveToGoalGate,
            leave;
    PathChain farMoveForwards,
    farMoveIntake21, farMoveDoneIntake21, farMoveAim21,
    farLeave;
    boolean updateFollower = true;
    boolean red_team = false;
    boolean close_auto = true;
    int close_velocity = 1175;
    int far_velocity = 1545;
    public Pose reversePose(Pose pose) {
        return new Pose(144-pose.getX(),pose.getY(),Math.toRadians(180) - pose.getHeading());
    }

    public void setRedPoints() {
        startingPose = reversePose(startingPose);
        aimingPose = reversePose(aimingPose);
        sideAimingPose = reversePose(sideAimingPose);
        intakePose23 = reversePose(intakePose23);
        intakeDonePose23 = reversePose(intakeDonePose23);
        intakePose22 = reversePose(intakePose22);
        intakeDonePose22 = reversePose(intakeDonePose22);
        intakePose21 = reversePose(intakePose21);
        intakeDonePose21 = reversePose(intakeDonePose21);
        leavePose = reversePose(leavePose);
        gatePose = reversePose(gatePose);
        preGatePose = reversePose(preGatePose);
        telePreGatePose = reversePose(telePreGatePose);

        farStartingPose = reversePose(farStartingPose);
        farAimingPose = reversePose(farAimingPose);
        farLeavePose = reversePose(farLeavePose);
    }

    public void buildFarPaths() {
        farMoveForwards = follower.pathBuilder()
                .addPath(new BezierLine(farStartingPose,farAimingPose))
                .setLinearHeadingInterpolation(farStartingPose.getHeading(),farAimingPose.getHeading())
                .build();
        farMoveIntake21 = follower.pathBuilder()
                .addPath(new BezierLine(farAimingPose,intakePose21))
                .setLinearHeadingInterpolation(farAimingPose.getHeading(),intakePose21.getHeading())
                .setBrakingStrength(0.85)
                .build();
        farMoveDoneIntake21 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose21,intakeDonePose21))
                .setLinearHeadingInterpolation(intakePose21.getHeading(),intakeDonePose21.getHeading())
                .build();
        farMoveAim21 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeDonePose21,intakePose21,farAimingPose))
                .setLinearHeadingInterpolation(intakeDonePose21.getHeading(),farAimingPose.getHeading())
                .build();
        farLeave = follower.pathBuilder()
                .addPath(new BezierLine(farAimingPose,farLeavePose))
                .setLinearHeadingInterpolation(farAimingPose.getHeading(),farLeavePose.getHeading())
                .build();

    }
    public void buildClosePaths() {
        moveBack = follower.pathBuilder()
                .addPath(new BezierLine(startingPose,aimingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),aimingPose.getHeading())
                .setBrakingStrength(0.5)
                .build();

        moveIntake23 = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,intakePose23))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),intakePose23.getHeading())
                .build();
        moveDoneIntake23 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose23,intakeDonePose23))
                .setLinearHeadingInterpolation(intakePose23.getHeading(),intakeDonePose23.getHeading())
                .build();
        moveToGoal23 = follower.pathBuilder()
                .addPath(new BezierLine(intakeDonePose23,aimingPose))
                .setLinearHeadingInterpolation(intakeDonePose23.getHeading(),aimingPose.getHeading())
                .build();

        moveIntake22 = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,intakePose22))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),intakePose22.getHeading())
                .build();
        moveDoneIntake22 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose22,intakeDonePose22))
                .setLinearHeadingInterpolation(intakePose22.getHeading(),intakeDonePose22.getHeading())
                .build();
        moveToGoal22 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeDonePose22,intakePose22,aimingPose))
                .setLinearHeadingInterpolation(intakeDonePose22.getHeading(),aimingPose.getHeading())
                .build();

        moveIntake21 = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,intakePose21))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),intakePose21.getHeading())
                .build();
        moveDoneIntake21 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose21,intakeDonePose21))
                .setLinearHeadingInterpolation(intakePose21.getHeading(),intakeDonePose21.getHeading())
                .build();
        moveToGoal21 = follower.pathBuilder()
                .addPath(new BezierCurve(intakeDonePose21,intakePose21,aimingPose))
                .setLinearHeadingInterpolation(intakePose21.getHeading(),aimingPose.getHeading())
                .build();

        moveToPreGate = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,preGatePose))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),preGatePose.getHeading())
                .build();
        moveToTelePreGate = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,telePreGatePose))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),telePreGatePose.getHeading())
                .build();
        moveToGate = follower.pathBuilder()
                .addPath(new BezierLine(preGatePose,gatePose))
                .setLinearHeadingInterpolation(preGatePose.getHeading(),gatePose.getHeading())
                .build();
        moveToGoalGate = follower.pathBuilder()
                .addPath(new BezierLine(gatePose,preGatePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(),preGatePose.getHeading())
                .addPath(new BezierLine(preGatePose,aimingPose))
                .setLinearHeadingInterpolation(preGatePose.getHeading(),aimingPose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(aimingPose,leavePose))
                .setLinearHeadingInterpolation(aimingPose.getHeading(),leavePose.getHeading())
                .build();
    }

    public void farAutoLoop() {
        switch (pathState) {
            case 0:
                startOuttakeVelocity(far_velocity);
                intake.setPower(intake_dir);
                follower.followPath(farMoveForwards);
                setPathState(pathState + 1);
                break;
            case 1:
                if (follower.isBusy()) {
                    break;
                }

                pedroShootLoop();

                break;
            case 2:
                if (follower.isBusy()) { break; }

                innertake.setPosition(innertake_up_pos);
                follower.followPath(farMoveIntake21,0.7,false);

                setPathState(pathState+1);
                break;
            case 3:
                if (follower.isBusy()) { break; }
                startIntake();

                follower.followPath(farMoveDoneIntake21,0.5,false);
                setPathState(pathState+1);
                break;
            case 4:
                if (follower.isBusy()) { break; }

                sleepMS(500);
                stopIntake();
                intake.setPower(intake_dir);
                startOuttakeVelocity(far_velocity);
                follower.followPath(farMoveAim21,0.7,false);
                setPathState(pathState+1);
                break;
            case 5:
                if (follower.isBusy()) { break; }

                pedroShootLoop();

                break;
            case 6:
                if (follower.isBusy()) { break; }

                follower.followPath(farLeave,1,true);

                setPathState(pathState+1);
                break;
        }
    }

    public void closeAutoLoop() {
        switch (pathState) {
            case 0:
                startOuttakeVelocity(close_velocity);
                intake.setPower(intake_dir);
                follower.followPath(moveBack);
                setPathState(pathState+1);
                break;
            case 1:
                if (follower.isBusy()) { break; }

                pedroShootLoop();

                break;
            case 2:
                if (follower.isBusy()) { break; }

                innertake.setPosition(innertake_up_pos);
                follower.followPath(moveIntake23);

                setPathState(pathState+1);
                break;
            case 3:
                if (follower.isBusy()) { break; }
                startIntake();

                follower.followPath(moveDoneIntake23,0.5,false);
                setPathState(pathState+1);
                break;
            case 4:
                if (follower.isBusy()) { break; }

                sleepMS(500);
                stopIntake();
                intake.setPower(intake_dir);
                startOuttakeVelocity(1130);
                follower.followPath(moveToGoal23);
                setPathState(pathState+1);
                break;
            case 5:
                if (follower.isBusy()) { break; }

                pedroShootLoop();

                break;
            case 6:
                innertake.setPosition(innertake_up_pos);
                if (follower.isBusy()) { break; }
                follower.followPath(moveIntake22);
                startIntake();

                setPathState(pathState+1);
                break;
            case 7:
                if (follower.isBusy()) { break; }

                follower.followPath(moveDoneIntake22,0.5,false);
                setPathState(pathState+1);
                break;
            case 8:
                if (follower.isBusy()) { break; }

                sleepMS(500);
                stopIntake();
                intake.setPower(intake_dir);
                startOuttakeVelocity(1130);

                follower.followPath(moveToGoal22,0.75,false);

                setPathState(pathState+1);
                break;
            case 9:
                if (follower.isBusy()) { break; }
                
                //idk

                setPathState(pathState+1);
                break;
            case 10:
                if (follower.isBusy()) { break; }

                pedroShootLoop();
                innertake.setPosition(innertake_down_pos);

                break;
            case 11:
                if (follower.isBusy()) { break; }
                follower.followPath(moveToTelePreGate,1,true);
                setPathState(pathState+1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        updateFollower = true;
    }

    public void cycle_tag(int tag) {

    }

    public void pedroInit() {
        follower = Constants.createFollower(hardwareMap);
        if (close_auto) {
            follower.setStartingPose(startingPose);
        } else {
            follower.setStartingPose(farStartingPose);
        }
        if (red_team) {
            setRedPoints();
        }
        if (close_auto) {
            buildClosePaths();
        } else {
            buildFarPaths();
        }
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    public void run(boolean red,boolean close) {
        red_team = red;
        close_auto = close;
        zkely_init();
        pedroInit();
        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);
        if (close) {
            follower.setPose(startingPose);
        } else {
            follower.setPose(farStartingPose);
        }
        while (opModeIsActive()) {
            follower.update();
            run_updates();
            telemetry.addData("path state",pathState);
            if (close_auto) {
                closeAutoLoop();
            } else {
                farAutoLoop();
            }
            telemetry.update();
        }
    }
    public void setHeadingFromBotpose() {
        update_imu();

        follower.setPose(new Pose(follower.getPose().getX(),follower.getPose().getY(),Math.toRadians(latest_fiducial.getRobotPoseFieldSpace().getOrientation().getYaw() - 90)));
    }

    public void pedroShootLoop() {
        float tolerance = 0.1f;
        if (!close_auto) {
            tolerance = 0.05f;
        }
        float return_val = limelight_teleop_circle(true,tolerance);
        telemetry.addData("aiming",return_val);
        if (return_val == 0) {
            set_motor_powers(0);
            setPathState(pathState + 1);
            if (close_auto) {
                startOuttakeVelocity(close_velocity);
            } else {
                startOuttakeVelocity(far_velocity);
            }
            while (!motor_at_velocity(outtake,close_velocity,20)) {
                follower.update();
            }
            set_motor_powers(0);
            if (close_auto) {
                startShootingVelocity(close_velocity);
            } else {
                startShootingVelocity(far_velocity);
            }
            sleep(3000);
            stopShooting();
        }
    }


    @Override
    public void runOpMode() {

    }
}

