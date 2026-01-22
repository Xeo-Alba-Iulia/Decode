package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import kotlin.math.PI

@OptIn(PathLinearExperimental::class)
@Autonomous
class FarBlueAuto : CoroutineOpMode() {
    lateinit var follower: Follower
    lateinit var shooter: Shooter

    val startPose = Pose(60.0, 12.0, PI / 2)
    val firstBallPose = Pose(24.0, 36.0, PI)
    val secondBallPose = Pose(24.0, 60.0, PI)
    val scorePose = Pose(42.0, 102.0, 3 / 4 * PI)

    val firstBalls = pathChain(null) {
        path {
            +startPose
            +Pose(60.0, 36.0)
            +Pose(42.0, 36.0)
        }
        pathToPose(firstBallPose)
    }
    val scoreFirstBalls = pathChain(null) {
        pathLinearHeading {
            +firstBallPose
            Pose(78.0, 66.0)
            +scorePose
        }
    }
    val secondBalls = pathChain(null) {
        pathConstantHeading(PI) {
            +scorePose
            +Pose(scorePose.x, secondBallPose.y, PI)
        }
        pathToPose(secondBallPose)
    }
    val scoreSecondBalls = pathChain(null) {
        pathLinearHeading {
            +secondBallPose
            +Pose(54.0, 72.0)
            +scorePose
        }
    }

    override fun init() {
        follower = opModeGraph.follower
        telemetry = opModeGraph.telemetry
        follower.setStartingPose(startPose)
    }

    override fun start() {
        opModeScope.launch {
            follower.followSuspend(firstBalls)
            follower.followSuspend(scoreFirstBalls)
            delay(1500L)
            follower.followSuspend(secondBalls)
            follower.followSuspend(scoreSecondBalls)
        }
    }

    override fun loop() {
        telemetry.addData("Pose", follower.pose)
    }
}