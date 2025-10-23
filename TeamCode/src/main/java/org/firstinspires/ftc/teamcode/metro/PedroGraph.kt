package org.firstinspires.ftc.teamcode.metro

import com.pedropathing.follower.Follower
import com.pedropathing.ftc.FollowerBuilder
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.pedropathing.Constants.followerConstants
import org.firstinspires.ftc.teamcode.pedropathing.Constants.mecanumConstants
import org.firstinspires.ftc.teamcode.pedropathing.Constants.pathConstraints
import org.firstinspires.ftc.teamcode.pedropathing.Constants.pinpointConstants

@ContributesTo(OpModeScope::class)
interface PedroGraph {
    val follower: Follower

    @SingleIn(OpModeScope::class)
    @Provides
    fun provideFollower(map: HardwareMap): Follower =
        FollowerBuilder(followerConstants, map)
            .mecanumDrivetrain(mecanumConstants)
            .pathConstraints(pathConstraints)
            .pinpointLocalizer(pinpointConstants)
            .build()
}