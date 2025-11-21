package org.firstinspires.ftc.teamcode.pedropathing;

import androidx.annotation.NonNull;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    @NonNull
    public static FollowerConstants followerConstants = new FollowerConstants();

    @NonNull
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100.0, 1.0, 1.0);

    @NonNull
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.CM)
            .forwardPodY(18.0)
            .strafePodX(0.0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    @NonNull
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .rightFrontMotorName("frontRight")
            .leftRearMotorName("backLeft")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    @NonNull
    public static Follower createFollower(HardwareMap map) {
        return new FollowerBuilder(followerConstants, map)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}
