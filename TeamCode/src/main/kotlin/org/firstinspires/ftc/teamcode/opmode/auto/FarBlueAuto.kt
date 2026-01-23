package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI

@OptIn(PathLinearExperimental::class)
@Autonomous
class FarBlueAuto : CoroutineOpMode() {
    lateinit var follower: Follower
    lateinit var intake: Intake
    lateinit var sorter: Sorter

    val startPose = Pose(60.0, 12.0, PI / 2)
    val firstBallPose = Pose(24.0, 36.0, PI)
    val firstBallPositionPose = Pose(firstBallPose.x + 20.0, firstBallPose.y, PI)
    val secondBallPose = Pose(24.0, 60.0, PI)
    val scorePose = Pose(42.0, 102.0, Math.toRadians(135.0))

    val firstBallsPosition = pathChain(null) {
        pathLinearHeading(endTime = 0.8) {
            +startPose
            +Pose(startPose.x, firstBallPose.y)
            +firstBallPositionPose
        }
    }
    val firstBalls = pathChain(null) {
        pathLinearHeading {
            +firstBallPositionPose
            +firstBallPose
        }
    }
    val scoreFirstBalls = pathChain(null) {
        pathLinearHeading(0.8) {
            +firstBallPose
            +Pose(80.0, 60.0)
            +scorePose
        }
    }
    val secondBallsPosition = pathChain(null) {
        pathConstantHeading(PI) {
            +scorePose
            +Pose(scorePose.x, secondBallPose.y)
        }
    }
    val secondBalls = pathChain(null) {
        pathConstantHeading(PI) {
            +Pose(scorePose.x, secondBallPose.y)
            +secondBallPose
        }
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
        sorter = opModeGraph.sorter
        intake = opModeGraph.intake
        follower.setStartingPose(startPose)
        observers += sorter
    }

    override fun start() {
        super.start()
        intake.isRunning = true
        opModeScope.launch {
            follower.followSuspend(firstBallsPosition)
            follower.followSuspend(firstBalls, maxPower = 0.3)
            delay(150L)
            sorter.intake(ArtefactType.GREEN)
            intake.isRunning = false
            follower.followSuspend(scoreFirstBalls)
            delay(500L)
            follower.followSuspend(secondBallsPosition)
            intake.isRunning = true
            follower.followSuspend(secondBalls, maxPower = 0.3)
            sorter.intake(ArtefactType.PURPLE)
            intake.isRunning = false
            follower.followSuspend(scoreSecondBalls)
        }
    }

    override fun loop() {
        telemetry.addData("Pose", follower.pose)
        drawDebug(follower)
    }

}