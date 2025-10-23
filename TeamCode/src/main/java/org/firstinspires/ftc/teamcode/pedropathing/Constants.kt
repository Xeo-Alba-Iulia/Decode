package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

object Constants {
    val followerConstants: FollowerConstants = FollowerConstants()

    @JvmField
    val pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, 1.0, 1.0)

    @JvmField
    val pinpointConstants: PinpointConstants = PinpointConstants()
        .distanceUnit(DistanceUnit.CM)
        .forwardPodY(18.0)
        .strafePodX(0.0)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)

    @JvmField
//    @JvmStatic
    val mecanumConstants: MecanumConstants = MecanumConstants()
        .leftFrontMotorName("frontLeft")
        .rightFrontMotorName("frontRight")
        .leftRearMotorName("backLeft")
        .rightRearMotorName("backRight")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
}
