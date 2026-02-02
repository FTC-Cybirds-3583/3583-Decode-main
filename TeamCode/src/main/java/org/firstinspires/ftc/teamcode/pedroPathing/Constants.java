package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-102.33197970911206)
            .lateralZeroPowerAcceleration(-67.67113148829944)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.01,0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06,0,0.01,0.6,0.05))
            .centripetalScaling(0.0004)
            .mass(13.4263);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(74.5)
            .yVelocity(58.32)
            .rightFrontMotorName("frontright")
            .rightRearMotorName("backright")
            .leftRearMotorName("backleft")
            .leftFrontMotorName("frontleft")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0019638011191475883)
            .strafeTicksToInches(.001986126330238159)
            .turnTicksToInches(0.001984)
            .leftPodY(3.25)
            .rightPodY(-3.25)
            .strafePodX(0)

            .leftEncoder_HardwareMapName("frontright")
            .rightEncoder_HardwareMapName("backleft")
            .strafeEncoder_HardwareMapName("frontleft")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    new Orientation(
                            AxesReference.INTRINSIC,
                            AxesOrder.ZYX,
                            AngleUnit.DEGREES,
                            90f,
                            0f,
                            0f,
                            0L
                    )
            ));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.75, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
