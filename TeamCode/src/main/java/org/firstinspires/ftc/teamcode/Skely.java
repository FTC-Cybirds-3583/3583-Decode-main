package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class Skely extends LinearOpMode {
    public DcMotorEx rfMotor, lfMotor, rbMotor, lbMotor;
    public int tolerance = 10;
    public Limelight3A limelight;
    public int rfDir = 1,
            rbDir = 1,
            lfDir = -1,
            lbDir = -1;
    //Direction modifiers for the motors, if spinning the wrong direction, change to 1 or -1

    public double driveSpeed;

    public void initRobot() {
        driveSpeed = 0.5;
        rfMotor = hardwareMap.get(DcMotorEx.class, "rightFront");//GoBILDA 5202/3/4 series //CH-M1
        lfMotor = hardwareMap.get(DcMotorEx.class, "leftFront");//GoBILDA 5202/3/4 series //CH-M0
        rbMotor = hardwareMap.get(DcMotorEx.class, "rightRear");//GoBILDA 5202/3/4 series //CH-M2
        lbMotor = hardwareMap.get(DcMotorEx.class, "leftRear");//GoBILDA 5202/3/4 series //CH-M3

        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        stopRobot();
    }

    public void drive() {
        lfMotor.setPower((driveSpeed * (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)));
        lbMotor.setPower((driveSpeed * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)));
        rbMotor.setPower((driveSpeed * (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)));
        rfMotor.setPower((driveSpeed * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)));

        /*lfMotor.setPower((driveSpeed * (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)));
        lbMotor.setPower((driveSpeed * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)));
        rbMotor.setPower((driveSpeed * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)));
        rfMotor.setPower((driveSpeed * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)));*/

    }

    public void powerDrive(double power, int time) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setPower(power * rfDir);
        lfMotor.setPower(power * lfDir);
        rbMotor.setPower(power * rbDir);
        lbMotor.setPower(power * lbDir);
        sleep(time);
        stopRobot();
    }

    public void powerStrafe(double power, int time){
        rfMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setPower(-power * rfDir);
        lfMotor.setPower(power * lfDir);
        rbMotor.setPower(power * rbDir);
        lbMotor.setPower(-power * lbDir);
        sleep(time);
        stopRobot();
    }

    public void velocityDrive(int velocity, int time) {
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setVelocity(velocity * rfDir);
        lfMotor.setVelocity(velocity * lfDir);
        rbMotor.setVelocity(velocity * rbDir);
        lbMotor.setVelocity(velocity * lbDir);
        sleep(time);
        stopRobot();
    }

    public void velocityStrafe(int velocity, int time) {
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setVelocity(-velocity * rfDir);
        lfMotor.setVelocity(velocity * lfDir);
        rbMotor.setVelocity(velocity * rbDir);
        lbMotor.setVelocity(-velocity * lbDir);
        sleep(time);
        stopRobot();
    }

    public void posDrive(int position, int velocity) {
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rfMotor.setTargetPosition(position * rfDir);
        lfMotor.setTargetPosition(position * lfDir);
        rbMotor.setTargetPosition(position * rbDir);
        lbMotor.setTargetPosition(position * lbDir);

        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfMotor.setTargetPositionTolerance(tolerance);
        lfMotor.setTargetPositionTolerance(tolerance);
        rbMotor.setTargetPositionTolerance(tolerance);
        lbMotor.setTargetPositionTolerance(tolerance);

        rfMotor.setVelocity(velocity);
        lfMotor.setVelocity(velocity);
        rbMotor.setVelocity(velocity);
        lbMotor.setVelocity(velocity);

        while (opModeIsActive()) {
            if (posInPlace()) {
                break;
            }

            telemetry.addLine("LF: " + lfMotor.getCurrentPosition() + "/" + lfMotor.getTargetPosition());
            telemetry.addLine("LB: " + lbMotor.getCurrentPosition() + "/" + lbMotor.getTargetPosition());
            telemetry.addLine("RF: " + rfMotor.getCurrentPosition() + "/" + rfMotor.getTargetPosition());
            telemetry.addLine("RB: " + rbMotor.getCurrentPosition() + "/" + rbMotor.getTargetPosition());

            telemetry.addLine("LF Velocity: " + lfMotor.getVelocity());
            telemetry.addLine("LB Velocity: " + lbMotor.getVelocity());
            telemetry.addLine("RF Velocity: " + rfMotor.getVelocity());
            telemetry.addLine("RB Velocity: " + rbMotor.getVelocity());

            //telemetry.addLine("Arm Low:" + armLow.getCurrentPosition());
            //telemetry.addLine("Arm High: " + armHigh.getCurrentPosition());
            //telemetry.addLine("Claw:" + claw.getPosition());

            telemetry.update();
        }
    }

    /**
     * @param position Positive strafes right, negative strafes left
     * @param velocity
     */
    public void posStrafe(int position, int velocity) {
        rfMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rfMotor.setTargetPosition(-position * rfDir);
        lfMotor.setTargetPosition(position * lfDir);
        rbMotor.setTargetPosition(position * rbDir);
        lbMotor.setTargetPosition(-position * lbDir);

        rfMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lfMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        rfMotor.setTargetPositionTolerance(tolerance);
        lfMotor.setTargetPositionTolerance(tolerance);
        rbMotor.setTargetPositionTolerance(tolerance);
        lbMotor.setTargetPositionTolerance(tolerance);

        rfMotor.setVelocity(velocity);
        lfMotor.setVelocity(velocity);
        rbMotor.setVelocity(velocity);
        lbMotor.setVelocity(velocity);

        while (opModeIsActive()) {
            if (posInPlace()) {
                break;
            }

            telemetry.addLine("LF: " + lfMotor.getCurrentPosition() + "/" + lfMotor.getTargetPosition());
            telemetry.addLine("LB: " + lbMotor.getCurrentPosition() + "/" + lbMotor.getTargetPosition());
            telemetry.addLine("RF: " + rfMotor.getCurrentPosition() + "/" + rfMotor.getTargetPosition());
            telemetry.addLine("RB: " + rbMotor.getCurrentPosition() + "/" + rbMotor.getTargetPosition());

            telemetry.addLine("LF Velocity: " + lfMotor.getVelocity());
            telemetry.addLine("LB Velocity: " + lbMotor.getVelocity());
            telemetry.addLine("RF Velocity: " + rfMotor.getVelocity());
            telemetry.addLine("RB Velocity: " + rbMotor.getVelocity());

            //telemetry.addLine("Arm Low:" + armLow.getCurrentPosition());
            //telemetry.addLine("Claw:" + claw.getPosition());

            telemetry.update();
        }
    }

    public void posTurnRight(int position, int velocity) {
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rfMotor.setTargetPosition(-position * rfDir);
        lfMotor.setTargetPosition(position * lfDir);
        rbMotor.setTargetPosition(-position * rbDir);
        lbMotor.setTargetPosition(position * lbDir);

        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfMotor.setTargetPositionTolerance(tolerance);
        lfMotor.setTargetPositionTolerance(tolerance);
        rbMotor.setTargetPositionTolerance(tolerance);
        lbMotor.setTargetPositionTolerance(tolerance);

        rfMotor.setVelocity(velocity);
        lfMotor.setVelocity(velocity);
        rbMotor.setVelocity(velocity);
        lbMotor.setVelocity(velocity);

        while (opModeIsActive()) {
            if (posInPlace()) {
                break;
            }

            telemetry.addLine("LF: " + lfMotor.getCurrentPosition() + "/" + lfMotor.getTargetPosition());
            telemetry.addLine("LB: " + lbMotor.getCurrentPosition() + "/" + lbMotor.getTargetPosition());
            telemetry.addLine("RF: " + rfMotor.getCurrentPosition() + "/" + rfMotor.getTargetPosition());
            telemetry.addLine("RB: " + rbMotor.getCurrentPosition() + "/" + rbMotor.getTargetPosition());

            telemetry.addLine("LF Velocity: " + lfMotor.getVelocity());
            telemetry.addLine("LB Velocity: " + lbMotor.getVelocity());
            telemetry.addLine("RF Velocity: " + rfMotor.getVelocity());
            telemetry.addLine("RB Velocity: " + rbMotor.getVelocity());

            //telemetry.addLine("Arm Low:" + armLow.getCurrentPosition());
            //telemetry.addLine("Claw:" + claw.getPosition());

            telemetry.update();
        }
    }

    public void posDrive(int position){
        posDrive(position, 500);
    }

    public void posStrafe(int position){
        posStrafe(position, 500);
    }

    public void posTurnRight(int position){
        posTurnRight(position, 500);
    }

    public void runToPos(@NonNull DcMotorEx motor, int position, int velocity, int tolerance){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPositionTolerance(tolerance);
        motor.setVelocity(velocity);
    }
    public void runToPos(DcMotorEx motor, int position, int velocity){
        runToPos(motor,position,velocity,motor.getTargetPositionTolerance());
    }

    public void stopRobot() {
        rfMotor.setPower(0);
        lfMotor.setPower(0);
        rbMotor.setPower(0);
        lbMotor.setPower(0);
    }

    public boolean isInPlace(DcMotorEx motor,double sd){
        return Math.abs(motor.getCurrentPosition()-motor.getTargetPosition())<sd*motor.getTargetPositionTolerance();
    }
    public boolean isInPlace(DcMotorEx motor){
        return isInPlace(motor,1.0);
    }

    public void holdInPlace(DcMotorEx motor,int velocity, int tolerance){
        if(isInPlace(motor)){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(0);
            runToPos(motor,motor.getCurrentPosition(),velocity,tolerance);
        }
    }
    public void holdInPlace(DcMotorEx motor,int velocity){
        holdInPlace(motor,velocity,10);
    }

    public void runToPosImpv(DcMotorEx motor, int position, int velocity, int tolerance){
        runToPos(motor, position, velocity, tolerance);
        holdInPlace(motor, velocity, tolerance);
    }
    public void runToPosImpv(DcMotorEx motor, int position, int velocity){
        runToPosImpv(motor, position, velocity, motor.getTargetPositionTolerance());
    }

    /**
     * Moves to a position in xy plane, with a certain turn
     * @param y Position in front of the robot in encoder ticks
     * @param x Position to the right of the robot in encoder ticks
     * @param turn Amount robot turns right in encoder ticks
     * @param velocity Speed the robot moves
     * @param tolerance Tolerances for all movements
     */
    public void posSpline(int y, int x, int turn, int velocity, int tolerance){
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runToPos(rfMotor, (y-x-turn)*rfDir, velocity, tolerance);
        runToPos(lfMotor, (y+x+turn)*lfDir, velocity, tolerance);
        runToPos(rbMotor, (y+x-turn)*rbDir, velocity, tolerance);
        runToPos(lbMotor, (y-x+turn)*lbDir, velocity, tolerance);
    }


    /**
     * Moves to a position in xy plane, with a certain turn, then holds position (might stop faster)
     * @param y Position in front of the robot in encoder ticks
     * @param x Position to the right of the robot in encoder ticks
     * @param turn Amount robot turns right in encoder ticks
     * @param velocity Speed the robot moves
     * @param tolerance Tolerances for all movements
     */
    public void posSplineImpv(int y, int x, int turn, int velocity, int tolerance){
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runToPosImpv(rfMotor, (y-x-turn)*rfDir, velocity, tolerance);
        runToPosImpv(lfMotor, (y+x+turn)*lfDir, velocity, tolerance);
        runToPosImpv(rbMotor, (y+x-turn)*rbDir, velocity, tolerance);
        runToPosImpv(lbMotor, (y-x+turn)*lbDir, velocity, tolerance);
    }

    public void runMHAuto(int turn, int distance){
        //Int turn: 1 for left turn, -1 right turn
        //Int distance 1 if if you,re closest to the backdrop, 2 if you,re farther
        if(opModeIsActive()){
            sleep(500);
            posDrive(1300,1000);
            sleep(1000);
            

            /*posDrive(-1150, 1000);
            closeClaw();
            sleep(1000);
            posTurnRight(920 * turn, 1000);
            posDrive(2000 * distance);//Make position 1500//*/
            //posStrafe(1000, 500)//
            //raise linear Slides//
            //openClaw//
            //sleep(500)//
            //lower linear Slides//
            //sleep(500)//
            //posDrive(-100, 300)//
            //posStrafe (-1000, 500)//
            //posDrive(500, 500)//
        }
    }

    public boolean posInPlace(){
        return isInPlace(rfMotor) && isInPlace(lfMotor) && isInPlace(rbMotor) && isInPlace(lbMotor);
    }

    public void rotateBot(int location){
        posTurnRight(location);
    }

    /* Method Vision Prop locator(){
    use camera to locate prop in robots vision

    do calculations to determine where in robots visio prop is

    round answer to -1,0, or 1 to determine how to position bot
     */


    public int calculatePos(float visionResult){
        visionResult = Math.round(visionResult);
        return (int) visionResult;
    }

    /* Method vision
        activate camera
        look for the team prop (red or blue color)
        determine where it is in field of view
        convert that position into a float number
        return that as a float
     */
}
