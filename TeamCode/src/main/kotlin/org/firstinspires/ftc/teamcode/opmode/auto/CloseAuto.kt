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
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class CloseAuto(alliance: Alliance) : CoroutineOpMode() {
    lateinit var follower: Follower
    lateinit var sorter: Sorter
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var limelight: Limelight3A

    lateinit var patternJob: Job

    private val isMirrored = alliance == Alliance.RED
    private fun mirrorAlliance(pose: Pose): Pose = if (isMirrored) pose.mirror() else pose

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun followAndIntake(
        pathChain: PathChain,
        timeout: Duration = 3.seconds
    ) = followAndIntake(timeout) { follower.followSuspend(pathChain) }

    @OptIn(ExperimentalCoroutinesApi::class)
    private suspend fun followAndIntake(
        timeout: Duration = 5.seconds,
        followFunction: suspend () -> Unit
    ) = coroutineScope {
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
        limelight.pipelineSwitch(0)
    }

    override fun loop() {}
}