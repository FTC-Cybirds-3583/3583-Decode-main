package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
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

@Autonomous( name = "MainPedroAuto")
//new Pose (0,0,0);//  new Pose (0,20,0);// // STRAFE TESTING POSES

public class MainPedroAuto extends Zkely {
    private ElapsedTime runtime = new ElapsedTime();

    int last_tag = 23;
    Pose startingPose = new Pose(19.160,123.354,Math.toRadians(144));
    Pose aimingPose = new Pose(48,96,Math.toRadians(130));
    Pose intakePose23 = new Pose(44,84,Math.toRadians(180));
    Pose intakeDonePose23 = new Pose(16,84,Math.toRadians(180));
    Pose intakePose22 = new Pose(42,60,Math.toRadians(180));
    Pose intakeDonePose22 = new Pose(12,60,Math.toRadians(180));
    Follower follower;
    int pathState = 0;
    Timer pathTimer, actionTimer, opmodeTimer;
    PathChain moveBack,moveIntake23,moveDoneIntake23,moveToGoal23,moveIntake22,moveDoneIntake22,moveToGoal22;
    boolean updateFollower = true;

    public void farAuto() {

    }
    public void buildClosePaths() throws InterruptedException {
        moveBack = follower.pathBuilder()
                .addPath(new BezierLine(startingPose,aimingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),aimingPose.getHeading())
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
                .addPath(new BezierLine(intakeDonePose22,aimingPose))
                .setLinearHeadingInterpolation(intakeDonePose22.getHeading(),aimingPose.getHeading())
                .build();
    }

    public void closeAutoLoop() {
        switch (pathState) {
            case 0:
                //intake.setPower(intake_dir);
                //startOuttakeVelocity(1200);
                follower.followPath(moveBack);
                setPathState(1);
                break;
            case 1:
                if (follower.isBusy()) { break; }

                pedroShoot();

                setPathState(2);
                break;
            case 2:
                if (follower.isBusy()) { break; }
                startIntake();
                follower.followPath(moveIntake23);

                while (pathTimer.getElapsedTime() < 1750) {
                    follower.update();
                }
                follower.followPath(moveDoneIntake23,0.5,true);

                setPathState(3);
                break;
            case 3:
                if (follower.isBusy()) { break; }

                stopIntake();
                intake.setPower(intake_dir);

                follower.followPath(moveToGoal23);
                while (pathTimer.getElapsedTime() < 2000) {
                    follower.update();
                }
                setPathState(4);
                break;
            case 4:
                if (follower.isBusy()) { break; }
                startIntake();
                follower.followPath(moveIntake22);
                while (pathTimer.getElapsedTime() < 4000) {
                    follower.update();
                }

                follower.followPath(moveDoneIntake22,0.5,true);
                setPathState(5);
                break;
            case 5:
                if (follower.isBusy()) { break; }

                stopIntake();
                intake.setPower(intake_dir);

                follower.followPath(moveToGoal22);
                while (pathTimer.getElapsedTime() < 4000) {
                    follower.update();
                }
                setPathState(6);
                break;
            case 6:
                if (follower.isBusy()) { break; }

                pedroShoot();

                setPathState(7);
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

    public void pedroInit() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildClosePaths();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        zkely_init();
        pedroInit();
        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);
        follower.setPose(startingPose);
        follower.followPath(moveBack);
        while (opModeIsActive()) {
            follower.update();
            closeAutoLoop();

            telemetry.addData("path state",pathState);
            telemetry.update();
        }
    }
    public void setHeadingFromBotpose() {
        update_imu();

        follower.setPose(new Pose(follower.getPose().getX(),follower.getPose().getY(),Math.toRadians(latest_fiducial.getRobotPoseFieldSpace().getOrientation().getYaw() - 90)));
    }

    public void pedroShoot() {
        run_updates();
        while (limelight_teleop_circle(true) > 0.1 && opModeIsActive()) {
            run_updates();
        }
        power_dual_joy_control(0,0,0,0,0);

        //SIMULATE SHOOTING
        sleep(3000);
    }
}

