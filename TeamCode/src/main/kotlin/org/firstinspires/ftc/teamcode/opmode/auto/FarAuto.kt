package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followAndIntake
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.shootPattern
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.toArtefactList
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class FarAuto(alliance: Alliance) : CoroutineOpMode() {
    private lateinit var follower: Follower
    private lateinit var intake: Intake
    private lateinit var sorter: Sorter
    private lateinit var shooter: Shooter
    private lateinit var limelight: Limelight3A
    private lateinit var patternJob: Job

    private val isMirrored = alliance == Alliance.RED

    // helper to mirror a pose only when needed
    private fun mirrorAlliance(p: Pose): Pose = if (isMirrored) p.mirror() else p

    // raw poses (defined for the blue alliance), then mirrored when needed
    private val startPose: Pose = mirrorAlliance(Pose(60.0, 7.0, PI / 2))
    private val rawGoalPose = Pose(12.0, 144.0 - 12.0)
    private val goalPose = mirrorAlliance(rawGoalPose)
    private val rawScorePose: Pose = Pose(60.0, 20.0).run { withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x)) }
    private val scorePose: Pose = mirrorAlliance(rawScorePose)
    private val rawFirstBallPose = Pose(10.0, 36.0, PI)
    private val firstBallPose: Pose = mirrorAlliance(rawFirstBallPose)
    private val firstBallPositionPose: Pose = mirrorAlliance(Pose(rawFirstBallPose.x + 28.0, rawFirstBallPose.y, PI))

    private val rawCornerBallPreposition = Pose(13.0, 18.0, 7 * PI / 6)
    private val cornerBallPreposition: Pose = mirrorAlliance(rawCornerBallPreposition)
    private val rawCornerBallPose = Pose(12.3, 9.4, 5 * PI / 6)
    private val cornerBallPose: Pose = mirrorAlliance(rawCornerBallPose)

    private lateinit var cornerPath: PathChain

    private val scoreFromCornerPath = pathChain {
        pathLinearHeading {
            +cornerBallPose
            +scorePose
//            callbacks {
//                temporalCallback(700.milliseconds) {intake.isRunning = false}
//            }
        }
    }

    private val lastBallPositionPath = pathChain {
        pathLinearHeading {
            +cornerBallPose
            +cornerBallPose.withX(cornerBallPose.x + 5.0)
            callbacks {
                addCallback { intake.isRunning = false }
            }
        }
    }

    private val lastBallCollectPath = pathChain {
        pathLinearHeading {
            +cornerBallPose.withY(cornerBallPose.y + 5.0)
            +mirrorAlliance(Pose(rawCornerBallPose.x + 5.0, rawCornerBallPose.y + if (isMirrored) -7 else 7))
            +mirrorAlliance(Pose(rawCornerBallPose.x + 10.0, rawCornerBallPose.y))
            +mirrorAlliance(Pose(rawCornerBallPose.x, rawCornerBallPose.y, PI))
            callbacks {
                temporalCallback(200.milliseconds) { intake.isRunning = true }
            }
        }
    }

    @Volatile
    private var fiducialId = 21
    private val scorePreload = pathChain {
        pathLinearHeading {
            +startPose
            +scorePose
        }
    }
    private lateinit var firstBalls: PathChain
    private val scoreFirstBalls = pathChain {
        pathLinearHeading(endTime = 0.8) {
            +firstBallPose
            +scorePose
        }
    }

    private val leavePathChain = pathChain {
        pathConstantHeading(PI) {
            +scorePose
            +mirrorAlliance(Pose(rawScorePose.x - 10.0, rawScorePose.y))
        }
    }

    override fun init() {
        follower = opModeGraph.follower
        telemetry = opModeGraph.telemetry
        sorter = opModeGraph.sorter.apply { position = 0.5 }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = 0.0 }
        limelight = opModeGraph.limelight
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
                callbacks { parametricCallback(0.6) { follower.setMaxPower(0.6) } }
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
                callbacks { addCallback { follower.setMaxPower(0.7) } }
            }
        }
        firstBalls = pathChain(follower) {
            pathLinearHeading(endTime = 0.8) {
                +scorePose
                +mirrorAlliance(Pose(rawScorePose.x, rawFirstBallPose.y))
                +firstBallPositionPose
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

    override fun start() {
        patternJob.cancel()
        super.start()
        val pattern = fiducialId.toArtefactList()
        val followerDispatcher = Dispatchers.Default.limitedParallelism(1)
        val distanceFlow = flowOf(goalPose.distanceFrom(scorePose) / 39.37)
        if (BuildConfig.DEBUG)
            opModeScope.launch(followerDispatcher) {
                telemetry.addData("Pose", follower.pose)
                drawDebug(follower)
            }
        opModeScope.launch(followerDispatcher) {
            var shooterJob = shooter.shoot(distanceFlow)
            follower.followSuspend(scorePreload)
            delay(2.seconds)
            shootPattern(sorter, shooterJob, emptyList())
            intake.isRunning = true
            follower.setMaxPower(0.7)
            follower.followAndIntake(intake, sorter, firstBalls, colorList = listOf(GREEN, PURPLE, PURPLE))
            intake.isOuttake = true
            follower.setMaxPower(1.0)
            shooterJob = shooter.shoot(distanceFlow)
            sorter.position = 0.5
            follower.followSuspend(scoreFirstBalls)
            delay(200.milliseconds)
            shooter.alignToPose(follower.pose, goalPose)
            shootPattern(sorter, shooterJob, pattern)
            repeat(3) {
                follower.setMaxPower(0.7)
                follower.followAndIntake(
                    intake,
                    sorter,
                    cornerPath,
                    lastBallPositionPath,
                    lastBallCollectPath,
                    colorList = listOf(PURPLE, GREEN, PURPLE),
                    timeout = 10.seconds
                )
                follower.setMaxPower(1.0)
                shooterJob = shooter.shoot(distanceFlow)
                follower.followSuspend(scoreFromCornerPath)
                shooter.alignToPose(follower.pose, goalPose, 0.0)
                shootPattern(sorter, shooterJob, pattern)
            }
            shooterJob.cancel()
            follower.followSuspend(leavePathChain)
        }
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