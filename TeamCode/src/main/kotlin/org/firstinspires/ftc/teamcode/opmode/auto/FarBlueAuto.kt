package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped
import kotlin.math.PI
import kotlin.math.atan2

@OptIn(PathLinearExperimental::class)
@Autonomous
class FarBlueAuto : CoroutineOpMode(isAuto = true) {
    lateinit var follower: Follower
    lateinit var intake: Intake
    lateinit var sorter: Sorter
    lateinit var shooter: Shooter
    lateinit var limelight: Limelight3A
    lateinit var patternJob: Job
    lateinit var shooterJob: Job

    val startPose = Pose(60.0 + 24.0, 7.0, PI / 2)
    val goalPose = Pose(12.0, 144.0 - 12.0)
    val scorePreloadPose = Pose(
        72.0 + 12.0, 23.0, atan2(
            goalPose.y - 23.0,
            goalPose.x - 72.0 - 12.0
        )
    )
    val firstBallPose = Pose(24.0, 36.0, PI)
    val firstBallPositionPose = Pose(firstBallPose.x + 20.0, firstBallPose.y, PI)
    val secondBallPose = Pose(24.0, 60.0, PI)
    val scorePose = Pose(42.0, 102.0, Math.toRadians(135.0))

    val pattern = Array(3) { ArtefactType.PURPLE }
    @Volatile
    var fiducialId = 0
    val scorePreload = pathChain(null) {
        pathLinearHeading {
            +startPose
            +scorePreloadPose
        }
    }
    val leavePathChain = pathChain(null) {
        path {
            +scorePreloadPose
            +Pose(72.0 + 12.0, 36.0)
        }
    }

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
        sorter = opModeGraph.sorter.also {
            it.position = SorterWrapped.OFFSET
        }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.also {
            it.angleDegrees = 0.0
        }
        limelight = opModeGraph.limelight
        observers += sorter
        follower.setStartingPose(startPose)
        limelight.pipelineSwitch(0)
        limelight.start()

        patternJob = opModeScope.launch {
            while (isActive) {
                limelight.latestResult.fiducialResults.singleOrNull()?.let {
                    fiducialId = it.fiducialId
                }
                delay(100L)
            }
        }
    }

    override fun init_loop() {
        telemetry.addData("FiducialId", fiducialId)
        telemetry.update()
        Thread.sleep(100L)
    }

    override fun start() {
        patternJob.cancel()
        super.start()
        pattern[fiducialId - 21] = ArtefactType.GREEN
//        shooter.hood = 0.85
        shooterJob = shooter.shoot { (goalPose.distanceFrom(scorePreloadPose) / 39.37) }
        opModeScope.launch {
            follower.setMaxPower(0.5)
            follower.followPath(scorePreload)
            while (follower.isBusy)
                ensureActive()
            launch { shootAll(shooter.stateFlow, sorter, shooterJob, *pattern) }
            val transferJob = launch {
                shooter.stateFlow
                    .map { it.canShoot }
                    .distinctUntilChanged()
                    .collect {
//                        delay(1000L)
                        sorter.isLifting = it
                    }
            }
            shooterJob.join()
            transferJob.cancelAndJoin()
            follower.followPath(leavePathChain)
        }
//        opModeScope.launch {
//            follower.followSuspend(firstBallsPosition)
//            follower.followSuspend(firstBalls, maxPower = 0.3)
//            delay(150L)
//            sorter.intake(ArtefactType.GREEN)
//            intake.isRunning = false
//            follower.followSuspend(scoreFirstBalls)
//            delay(500L)
//            follower.followSuspend(secondBallsPosition)
//            intake.isRunning = true
//            follower.followSuspend(secondBalls, maxPower = 0.3)
//            sorter.intake(ArtefactType.PURPLE)
//            intake.isRunning = false
//            follower.followSuspend(scoreSecondBalls)
//        }
    }

    override fun loop() {
        follower.update()
        telemetry.addData("Pose", follower.pose)
        if (BuildConfig.DEBUG)
            drawDebug(follower)
    }

    override fun stop() {
        super.stop()
        lastPose = follower.pose
    }
}