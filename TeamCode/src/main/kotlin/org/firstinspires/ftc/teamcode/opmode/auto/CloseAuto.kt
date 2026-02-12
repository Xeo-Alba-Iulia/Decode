package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.take
import kotlinx.coroutines.selects.onTimeout
import kotlinx.coroutines.selects.select
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class CloseAuto(alliance: Alliance) : CoroutineOpMode() {
    private lateinit var follower: Follower
    private lateinit var sorter: Sorter
    private lateinit var intake: Intake
    private lateinit var shooter: Shooter
    private var limelight: Limelight3A? = null

    private val isMirrored = alliance == Alliance.RED
    private fun mirrorAlliance(pose: Pose): Pose = if (isMirrored) pose.mirror() else pose

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun followAndIntake(
        pathChain: PathChain,
        timeout: Duration = 5.seconds
    ) = followAndIntake(timeout) { follower.followSuspend(pathChain) }

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun followAndIntake(
        timeout: Duration = 5.seconds,
        followFunction: suspend () -> Unit
    ): Unit = coroutineScope {
        intake.isRunning = true
        val intakeJob = launch { intake.artefactFlow.take(3 - sorter.size).collect { sorter.intake(it) } }
        val followerJob = launch { followFunction() }
        select {
            intakeJob.onJoin { Log.d(TAG, "Stopped follow because intake finished") }
            followerJob.onJoin { Log.e(TAG, "Only picked up ${sorter.size} artefacts") }
            onTimeout(timeout) { Log.e(TAG, "Path didn't finish, picked up ${sorter.size} artefacts") }
        }
        followerJob.cancel()
        opModeScope.launch {
            delay(300.milliseconds)
            intakeJob.cancel()
            intake.isServoRunning = true
        }
    }

    private val obeliskPose = Pose(72.0, 144.0)
    private val startPose = mirrorAlliance(Pose(19.0, 121.0, Math.toRadians(54.0)))
    private val rawGoalPose = Pose(141.5 - 13.0, 13.0)
    private val rawScorePose: Pose = Pose(51.0, 84.0).run {
        withHeading(
            atan2(
                rawGoalPose.x - startPose.x,
                rawGoalPose.y - startPose.y
            )
        )
    }
    private val scorePose = mirrorAlliance(rawScorePose)
    private val firstBallsCollectPose = mirrorAlliance(Pose(21.0, 84.0, PI))
    private val freeGoalPose = mirrorAlliance(Pose(15.0, 72.0, PI))

    private val scorePreload = pathChain {
        pathLinearHeading {
            +startPose
            +scorePose
        }
    }

    private val collectBalls = pathChain {
        path {
            +scorePose
            +firstBallsCollectPose
        }
    }
    private val freeGate = pathChain {
        pathConstantHeading(PI) {
            +firstBallsCollectPose
            +Pose(firstBallsCollectPose.x, freeGoalPose.y)
            +freeGoalPose
        }
    }
    private val scoreBalls = pathChain {
        pathConstantHeading(scorePose.heading) {
            +freeGoalPose
            +scorePose
        }
    }

    override fun init() {
        follower = opModeGraph.follower
        sorter = opModeGraph.sorter.apply { position = 0.5 }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = 0.0 }
        limelight = opModeGraph.limelight
        telemetry = opModeGraph.telemetry
        limelight?.pipelineSwitch(0)
        limelight?.start()
    }

    override fun start() {
        val distance = rawGoalPose.distanceFrom(rawScorePose) / 39.37
        opModeScope.launch {
            val patternId = async {
                var id = 0
                while (id == 0) {
                    limelight?.latestResult?.fiducialResults?.singleOrNull()?.let {
                        id = it.fiducialId
                    }
                    shooter.alignToPose(follower.pose, obeliskPose)
                    delay(50.milliseconds)
                }
                Log.d(TAG, "Detected pattern ID: $id")
                id
            }
            follower.followSuspend(scorePreload)
            val pattern = withTimeoutOrNull(1.seconds) { patternId.await() }
            val patternList = when (pattern) {
                21 -> listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE)
                22 -> listOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE)
                23 -> listOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN)
                else -> {
                    Log.e(TAG, "Failed to detect pattern, defaulting to empty")
                    emptyList()
                }
            }
            shooter.angleDegrees = 0.0
            shootAll(shooter.stateFlow, sorter, shooter.shoot { distance })
            followAndIntake(collectBalls)
            follower.followSuspend(freeGate)
            val shooterJob = shooter.shoot { distance }
            follower.followSuspend(scoreBalls)
            shootAll(shooter.stateFlow, sorter, shooterJob, patternList)
        }
    }

    override fun loop() {}
}