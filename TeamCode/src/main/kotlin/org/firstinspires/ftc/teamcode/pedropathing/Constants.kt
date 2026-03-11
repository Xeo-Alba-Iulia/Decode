@file:JvmName("Constants")
@file:Config
package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@JvmField
var followerConstants: FollowerConstants = FollowerConstants()
    .mass(12.3)
    .forwardZeroPowerAcceleration(-41.02643032142339)
    .lateralZeroPowerAcceleration(-73.149645)
    .translationalPIDFCoefficients(PIDFCoefficients(0.2, 0.0, 0.012, 0.01))
    .headingPIDFCoefficients(PIDFCoefficients(1.8, 0.0, 0.09, 0.02))
    .drivePIDFCoefficients(FilteredPIDFCoefficients(0.007, 0.0, 0.00008, 0.6, 0.04))

@JvmField
var pathConstraints: PathConstraints = PathConstraints(0.99, 100.0, 1.5, 0.9)

@JvmField
var pinpointConstants: PinpointConstants = PinpointConstants()
    .distanceUnit(DistanceUnit.CM)
    .forwardPodY(18.0)
    .strafePodX(0.0)
    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)

@JvmField
var mecanumConstants: MecanumConstants = MecanumConstants()
    .leftFrontMotorName("frontLeft")
    .rightFrontMotorName("frontRight")
    .leftRearMotorName("backLeft")
    .rightRearMotorName("backRight")
    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .xVelocity(76.91)
    .yVelocity(63.23)
