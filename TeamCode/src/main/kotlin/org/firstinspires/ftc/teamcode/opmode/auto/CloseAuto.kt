package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.take
import kotlinx.coroutines.selects.onTimeout
import kotlinx.coroutines.selects.select
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.opmode.lastPose
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
        val intakeJob = launch {
            intake.artefactFlow.take(3 - sorter.size).collect {
                sorter.intake(it)
                RobotLog.dd(TAG, "Intaked $it, sorter now has ${sorter.size} artefacts")
                delay(130L)
            }
        }
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

    private val rawStartPose = Pose(32.0, 135.0)
    private val startPose = mirrorAlliance(rawStartPose)
    private val rawGoalPose = Pose(13.0, 141.5 - 13.0)
    private val goalPose = mirrorAlliance(rawGoalPose)
    private val rawScorePose: Pose = Pose(54.0, 86.0).run {
        withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x))
    }
    private val scorePose = mirrorAlliance(rawScorePose)
    private val rawFirstBallsCollectPose = Pose(18.0, 84.0, PI)
    private val firstBallsCollectPose = mirrorAlliance(rawFirstBallsCollectPose)
    private val rawSecondBallsCollectPose = Pose(11.0, 60.0, PI)
    private val secondBallsCollectPose = mirrorAlliance(rawSecondBallsCollectPose)
    private val rawFreeGoalPose = Pose(16.0, 73.0, PI / 2)
    private val freeGoalPose = mirrorAlliance(rawFreeGoalPose)

    private val scorePreload = pathChain {
        pathLinearHeading(endTime = .8) {
            +startPose
            +scorePose
        }
    }

    private val collectBalls1 = pathChain {
        pathLinearHeading(endTime = 0.45) {
            +scorePose
            +mirrorAlliance(Pose(rawScorePose.x, rawFirstBallsCollectPose.y))
            +firstBallsCollectPose
        }
    }
    private val freeGate = pathChain {
        pathLinearHeading(endTime = 0.8) {
            +firstBallsCollectPose
            +mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 12.0, rawFreeGoalPose.y + 5.0))
            +freeGoalPose
        }
    }
    private val scoreBalls1 = pathChain {
        pathLinearHeading {
            +firstBallsCollectPose
            +scorePose
        }
    }
    private lateinit var collectBalls2: PathChain
    private val scoreBalls2 = pathChain {
        pathLinearHeading(endTime = 0.8) {
            +secondBallsCollectPose
            +mirrorAlliance(Pose(rawScorePose.x, rawSecondBallsCollectPose.y))
            +scorePose
        }
    }
    private val leavePath = pathChain {
        pathConstantHeading(scorePose.heading) {
            +scorePose
            +mirrorAlliance(Pose(rawScorePose.x - 15.0, rawScorePose.y))
        }
    }

    override fun init() {
        follower = opModeGraph.follower.apply { setStartingPose(startPose) }
        sorter = opModeGraph.sorter.apply { position = 0.5 }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = if (isMirrored) 15.0 else -15.0 }
        limelight = opModeGraph.limelight
        telemetry = opModeGraph.telemetry
        limelight?.pipelineSwitch(0)
        limelight?.start()
        collectBalls2 = pathChain(follower) {
            pathLinearHeading(endTime = 0.6) {
                +scorePose
                +mirrorAlliance(Pose(rawScorePose.x, rawSecondBallsCollectPose.y))
                +mirrorAlliance(Pose(rawScorePose.x - 4.0, rawSecondBallsCollectPose.y))
                +secondBallsCollectPose
                callbacks {
                    parametricCallback(0.35) { intake.isRunning = true; follower.setMaxPower(0.4) }
                }
            }
        }
    }

    override fun start() {
        val distance = goalPose.distanceFrom(scorePose) / 39.37
        intake.isServoRunning = true
        opModeScope.launch {
            var job: Job
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
                Log.d(TAG, "Detected $result")
                result
            }
            val patternList = when (pattern) {
                21 -> listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE)
                22 -> listOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE)
                23 -> listOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN)
                else -> {
                    Log.e(TAG, "Failed to detect pattern, defaulting to first pattern")
                    listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE)
                }
            }
            shootAll(shooter.stateFlow, sorter, job, patternList)
            follower.setMaxPower(0.5)
            followAndIntake(collectBalls1)
            follower.setMaxPower(1.0)
//            follower.followSuspend(freeGate, maxPower = .7)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls1)
            shootAll(shooter.stateFlow, sorter, job, patternList)
            followAndIntake(collectBalls2)
            follower.setMaxPower(1.0)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls2)
            shootAll(shooter.stateFlow, sorter, job, patternList)
            /*
            val count = sorter.size
            if (count != 0) {
                sorter.prepareShoot()
                sorter.isLifting = true
                delay(600L)
//                shootFlow.map { it.canShoot }.filter { it }.first()
                for (i in 1..<count) {
                    sorter.prepareShoot()
                    RobotLog.dd("Shooter", "Shot $i balls")
                    delay(600L)
                }
                delay(600L)
                sorter.prepareIntake()
                delay(250L)
                sorter.isLifting = false
                job.cancel()
            }
             */
            follower.followSuspend(leavePath)
        }
    }

    override fun loop() {
        drawDebug(follower)
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
        lastPose = follower.pose
    }
}