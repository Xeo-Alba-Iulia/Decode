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
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
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
        val intakeJob = launch { intake.artefactFlow.take(3 - sorter.size).collect { sorter.intake(it); delay(150L) } }
        val followerJob = launch { followFunction() }
        select {
            intakeJob.onJoin { Log.d(TAG, "Stopped follow because intake finished") }
            followerJob.onJoin { Log.e(TAG, "Only picked up ${sorter.size} artefacts") }
            onTimeout(timeout) { Log.e(TAG, "Path didn't finish, picked up ${sorter.size} artefacts") }
        }
        followerJob.cancel()
        opModeScope.launch {
            delay(500.milliseconds)
            intakeJob.cancel()
            intake.isServoRunning = true
        }
    }

    private val startPose = mirrorAlliance(Pose(32.0, 135.0))
    private val goalPose = mirrorAlliance(Pose(13.0, 141.5 - 13.0))
    private val scorePose: Pose = mirrorAlliance(Pose(54.0, 86.0)).run {
        withHeading(atan2(goalPose.y - y, goalPose.x - x))
    }
    private val firstBallsCollectPose = mirrorAlliance(Pose(18.0, 84.0, PI))
    private val secondBallsCollectPose = mirrorAlliance(Pose(15.0, 60.0, PI))
    private val freeGoalPose = mirrorAlliance(Pose(16.0, 72.0, PI / 2))

    private val scorePreload = pathChain {
        pathLinearHeading(endTime = .8) {
            +startPose
            +scorePose
        }
    }

    private val collectBalls1 = pathChain {
        pathLinearHeading(endTime = 0.45) {
            +scorePose
            +Pose(scorePose.x, firstBallsCollectPose.y)
            +firstBallsCollectPose
        }
    }
    private val freeGate = pathChain {
        pathLinearHeading(endTime = 0.8) {
            +firstBallsCollectPose
            +Pose(firstBallsCollectPose.x + 12.0, freeGoalPose.y + 5.0)
            +freeGoalPose
        }
    }
    private val scoreBalls1 = pathChain {
        pathLinearHeading {
            +freeGoalPose
            +scorePose
        }
    }
    private lateinit var collectBalls2: PathChain
    private val scoreBalls2 = pathChain {
        pathLinearHeading(endTime = 0.8) {
            +secondBallsCollectPose
            +scorePose
        }
    }
    private val leavePath = pathChain {
        pathConstantHeading(scorePose.heading) {
            +scorePose
            +Pose(30.0, 60.0)
        }
    }

    override fun init() {
        follower = opModeGraph.follower.apply { setStartingPose(startPose) }
        sorter = opModeGraph.sorter.apply { position = 0.5 }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = -15.0 }
        limelight = opModeGraph.limelight
        telemetry = opModeGraph.telemetry
        limelight?.pipelineSwitch(0)
        limelight?.start()
        collectBalls2 = pathChain(follower) {
            pathLinearHeading(endTime = 0.6) {
                +scorePose
                +Pose(scorePose.x, secondBallsCollectPose.y)
                +Pose(scorePose.x - 4.0, secondBallsCollectPose.y)
                +secondBallsCollectPose
                callbacks {
                    parametricCallback(0.5) { intake.isRunning = true; follower.setMaxPower(0.4) }
                }
            }
        }
    }

    override fun start() {
        val distance = goalPose.distanceFrom(scorePose) / 39.37
        intake.isServoRunning = true
        opModeScope.launch {
            val job: Job
            val pattern = coroutineScope {
                val patternId = async {
                    var id = 0
                    while (id == 0) {
                        limelight?.latestResult?.fiducialResults?.singleOrNull()?.let { id = it.fiducialId }
                        delay(50.milliseconds)
                    }
                    Log.d(TAG, "Detected pattern ID: $id")
                    id
                }
                follower.followSuspend(scorePreload)
                job = shooter.shoot { distance }
                launch { delay(500.milliseconds) }
                val result = withTimeoutOrNull(1.seconds) { patternId.await() }
                shooter.angleDegrees = 0.0
                result
            }
            val patternList = when (pattern) {
                21 -> listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE)
                22 -> listOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE)
                23 -> listOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN)
                else -> {
                    Log.e(TAG, "Failed to detect pattern, defaulting to empty")
                    emptyList()
                }
            }
//            shooter.alignToPose(follower.pose, goalPose)
            shootAll(shooter.stateFlow, sorter, job)
            follower.setMaxPower(0.45)
            followAndIntake(collectBalls1)
            follower.setMaxPower(1.0)
            follower.followSuspend(freeGate, maxPower = .7)
            follower.followSuspend(scoreBalls1)
//            shooter.alignToPose(follower.pose, goalPose)
            shootAll(shooter.stateFlow, sorter, shooter.shoot { distance }, patternList)
            followAndIntake(collectBalls2)
            follower.setMaxPower(1.0)
            follower.followSuspend(scoreBalls2)
            shootAll(shooter.stateFlow, sorter, shooter.shoot { distance }, patternList)
            follower.followSuspend(leavePath)
        }
    }

    override fun loop() {
        drawDebug(follower)
    }
}