package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class FarAuto(alliance: Alliance) : CoroutineOpMode() {
    lateinit var follower: Follower
    lateinit var intake: Intake
    lateinit var sorter: Sorter
    lateinit var shooter: Shooter
    lateinit var limelight: Limelight3A
    lateinit var patternJob: Job
    lateinit var shooterJob: Job

    private val isMirrored = alliance == Alliance.RED

    // helper to mirror a pose only when needed
    private fun mirrorAlliance(p: Pose): Pose = if (isMirrored) p.mirror() else p

    // raw poses (defined for the blue alliance), then mirrored when needed
    val startPose: Pose = mirrorAlliance(Pose(60.0, 7.0, PI / 2))
    val rawGoalPose = Pose(12.0, 144.0 - 12.0)
    val goalPose = mirrorAlliance(rawGoalPose)
    val rawScorePose = Pose(
        60.0, 20.0, atan2(
            rawGoalPose.y - 20.0,
            rawGoalPose.x - 60.0
        )
    )
    val scorePose: Pose = mirrorAlliance(rawScorePose)
    val rawFirstBallPose = Pose(11.5, 36.0, PI)
    val firstBallPose: Pose = mirrorAlliance(rawFirstBallPose)
    val firstBallPositionPose: Pose = mirrorAlliance(Pose(rawFirstBallPose.x + 28.0, rawFirstBallPose.y, PI))

    val cornerBallPreposition: Pose = mirrorAlliance(Pose(13.0, 18.0, 7 * PI / 6))
    val cornerBallPose: Pose = mirrorAlliance(Pose(12.0, 10.0, 5 * PI / 6))

    lateinit var cornerPath: PathChain

    val scoreFromCornerPath = pathChain {
        pathLinearHeading {
            +cornerBallPose
            +scorePose
        }
    }

    val lastBallPositionPath = pathChain {
        pathLinearHeading {
            +cornerBallPose
            +cornerBallPose.withX(cornerBallPose.x + 7.0)
            callbacks {
                temporalCallback(Duration.ZERO) { intake.isRunning = false }
            }
        }
    }

    val lastBallCollectPath = pathChain {
        pathLinearHeading {
            +cornerBallPose.withY(cornerBallPose.y + 7.0)
            +Pose(cornerBallPose.x + 7.0, cornerBallPose.y + 7.0)
            +cornerBallPose.withHeading(PI)
            callbacks {
                temporalCallback(200.milliseconds) { intake.isRunning = true }
            }
        }
    }

    @Volatile
    var fiducialId = 21
    val scorePreload = pathChain {
        pathLinearHeading {
            +startPose
            +scorePose
        }
    }
    lateinit var firstBalls: PathChain
    val scoreFirstBalls = pathChain(null) {
        pathLinearHeading(endTime = 0.8) {
            +firstBallPose
            +scorePose
            callbacks { addCallback { follower.setMaxPower(1.0) } }
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
        intake.artefactFlow
            .onEach { Log.d("Intake", "Detected artefact: $it") }
            .onEach { sorter.intake(it) }
            .launchIn(opModeScope + Dispatchers.IO)

        intake.stateFlow
            .onEach {
                val packet = TelemetryPacket().apply { put("Intake State", it) }
                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }
            .launchIn(opModeScope + Dispatchers.IO)

        cornerPath = pathChain(follower, decelerationType = PathChain.DecelerationType.NONE) {
            pathLinearHeading(0.9) {
                +scorePose
                +cornerBallPreposition
                callbacks { parametricCallback(0.6) { follower.setMaxPower(0.5) } }
            }
            path(
                interpolator = HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode(
                        0.0,
                        0.6,
                        HeadingInterpolator.constant(cornerBallPreposition.heading)
                    ),
                    HeadingInterpolator.PiecewiseNode(
                        0.6, 1.0, HeadingInterpolator.linear(
                            cornerBallPreposition.heading,
                            cornerBallPose.heading
                        )
                    )
                )
            ) {
                +cornerBallPreposition
                +cornerBallPose
                callbacks { addCallback { follower.setMaxPower(0.5) } }
            }
        }
        firstBalls = pathChain(follower) {
            pathLinearHeading(endTime = 0.8) {
                +scorePose
                +Pose(scorePose.x, firstBallPose.y)
                +firstBallPositionPose
                callbacks { parametricCallback(0.5) { follower.setMaxPower(0.5) } }
            }
            pathLinearHeading {
                +firstBallPositionPose
                +firstBallPose
            }
        }
    }

    override fun init_loop() {
        telemetry.addData("FiducialId", fiducialId)
        telemetry.update()
        Thread.sleep(100L)
    }

    private fun distanceFun() = 3.5

    private suspend fun followAndIntake(
        pathChain: PathChain,
        pickupPattern: List<ArtefactType>,
        timeout: Duration = 3.seconds
    ) {
        merge(
            flow {
                intake.artefactFlow
                    .take(3)
                    .collect { sorter.intake(it) }
                Log.d(TAG, "Got all 3 balls")
                emit(Unit)
            },
            flow {
                follower.followSuspend(pathChain)
                delay(250.milliseconds)
                Log.e(TAG, "Path finished prematurely")
                emit(Unit)
            },
            flow {
                delay(timeout)
                Log.e(TAG, "Path didn't finish, picked up ${sorter.size} artefacts")
                emit(Unit)
            }
        ).first()
    }

    private suspend fun outputAll(pattern: List<ArtefactType?>) {
        val iter = pattern.iterator()
        while (!sorter.isEmpty && iter.hasNext()) {
            sorter.prepareShoot(iter.next())
            delay(1.25.seconds)
        }
        sorter.prepareIntake()
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
        opModeScope.launch {
            follower.followSuspend(scorePreload)
            shooterJob = shooter.shoot(::distanceFun)
            shootAll(shooter.stateFlow, sorter, shooterJob, pattern)
            intake.isRunning = true
            followAndIntake(
                firstBalls,
                listOf(ArtefactType.GREEN, ArtefactType.PURPLE, ArtefactType.PURPLE),
                timeout = 8.seconds
            )
            follower.followSuspend(scoreFirstBalls)
            delay(200.milliseconds)
            shooter.alignToPose(follower.pose, goalPose, 0.0)
            shooterJob = shooter.shoot(::distanceFun)
            shootAll(shooter.stateFlow, sorter, shooterJob, pattern)
            val intakeJob = launch {
                intake.artefactFlow
                    .take(3)
                    .collect { sorter.intake(it) }
            }
            follower.followSuspend(cornerPath)
            follower.followSuspend(lastBallPositionPath, maxPower = 0.5)
            follower.followSuspend(lastBallCollectPath, maxPower = 0.5)
            follower.setMaxPower(1.0)
            follower.followSuspend(scoreFromCornerPath)
            intake.isRunning = false
            intakeJob.cancel()
            delay(200.milliseconds)
            shooter.alignToPose(follower.pose, goalPose, 0.0)
            shooterJob = shooter.shoot(::distanceFun)
            shootAll(shooter.stateFlow, sorter, shooterJob, pattern)
        }
    }

    override fun loop() {
//        telemetry.addData("Pose", follower.pose)
//        if (BuildConfig.DEBUG)
//            drawDebug(follower)
    }

    override fun stop() {
        super.stop()
        RobotLog.dd(TAG, "Stop ran")
        lastPose = follower.pose
    }

    companion object {
        const val TAG = "Auto"
    }
}