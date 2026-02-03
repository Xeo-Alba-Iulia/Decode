package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.take
import kotlinx.coroutines.selects.onTimeout
import kotlinx.coroutines.selects.select
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

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

    val startPose = Pose(60.0, 7.0, PI / 2)
    val goalPose = Pose(12.0, 144.0 - 12.0)
    val scorePose = Pose(
        60.0, 20.0, atan2(
            goalPose.y - 20.0,
            goalPose.x - 60.0
        )
    )
    val firstBallPose = Pose(13.0, 36.0, PI)
    val firstBallPositionPose = Pose(firstBallPose.x + 25.0, firstBallPose.y, PI)

    @Volatile
    var fiducialId = 21
    val scorePreload = pathChain {
        pathLinearHeading {
            +startPose
            +scorePose
        }
    }
    val firstBallsPosition = pathChain(null) {
        pathLinearHeading(endTime = 0.8) {
            +scorePose
            +Pose(scorePose.x, firstBallPose.y)
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
        pathLinearHeading(endTime = 0.8) {
            +firstBallPose
            +scorePose
            callbacks {
                temporalCallback(650.milliseconds) {
                    intake.isRunning = false
                    intake.isServoRunning = true
                }
            }
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
        drawDebug(follower)
        shooter.hood = 0.8
    }

    override fun init_loop() {
        telemetry.addData("FiducialId", fiducialId)
        telemetry.update()
        Thread.sleep(100L)
    }

    private fun distanceFun() = 3.5

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun followAndIntake(pathChain: PathChain, pickupPattern: List<ArtefactType>) = coroutineScope {
        select<Unit> {
            launch {
                val iter = pickupPattern.iterator()
                intake.distanceFlow
                    .filter { it }
                    .take(3)
                    .collect { sorter.intake(iter.next()) }
            }.onJoin { Log.d(TAG, "Stopped follow because intake finished") }
            launch { follower.followSuspend(pathChain, maxPower = 0.37) }
                .onJoin { Log.e(TAG, "Only picked up ${sorter.size} artefacts") }
            onTimeout(2.seconds) { throw RuntimeException("Path didn't finish") }
        }
    }

    override fun start() {
        patternJob.cancel()
        super.start()
        val pattern = when (fiducialId) {
            21 -> listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE)
            22 -> listOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE)
            23 -> listOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN)
            else -> error("Invalid fiducialId: $fiducialId")
        }
        shooterJob = shooter.shoot(::distanceFun)
        opModeScope.launch {
            follower.followSuspend(scorePreload, maxPower = 0.7)
            shootAll(shooter.stateFlow, sorter, shooterJob, pattern)
            intake.isRunning = true
            follower.followSuspend(firstBallsPosition)
            followAndIntake(firstBalls, listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE))
            shooterJob = shooter.shoot(::distanceFun)
            follower.followSuspend(scoreFirstBalls)
            shootAll(shooter.stateFlow, sorter, shooterJob, pattern)
            requestOpModeStop()
        }
    }

    override fun loop() {
        telemetry.addData("Pose", follower.pose)
        if (BuildConfig.DEBUG)
            drawDebug(follower)
    }

    override fun stop() {
        super.stop()
        RobotLog.dd(TAG, "Stop ran")
        lastPose = follower.pose
    }

    companion object {
        const val TAG = "FarBlueAuto"
    }
}