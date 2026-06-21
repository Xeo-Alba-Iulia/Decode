package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilderKt
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followAndIntake
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.*
import org.firstinspires.ftc.teamcode.toArtefactList
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class FarAuto(alliance: Alliance) : CoroutineOpMode() {
    val follower by onInit { opModeGraph.follower.apply { setStartingPose(startPose) } }
    val intake by onInit { opModeGraph.intake }
    val sorter by onInit { opModeGraph.sorter.apply { position = 0.5 } }
    val shooter: Shooter by onInit { opModeGraph.shooter.apply { angleDegrees = 0.0 } }
    val limelight by onInit { opModeGraph.limelight.apply { pipelineSwitch(0); start() } }
    private lateinit var patternJob: Job

    private val isMirrored = alliance == Alliance.RED

    // helper to mirror a pose only when needed
    private fun mirrorAlliance(p: Pose): Pose = if (isMirrored) p.mirror() else p

    // raw poses (defined for the blue alliance), then mirrored when needed
    private val startPose: Pose = mirrorAlliance(Pose(60.0, 7.0, PI / 2))
    private val rawGoalPose = Pose(10.0, 144.0 - 10.0)
    private val goalPose = mirrorAlliance(rawGoalPose)
    private val rawScorePose: Pose = Pose(60.0, 20.0).run { withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x)) }
    private val scorePose: Pose = mirrorAlliance(rawScorePose)
    private val rawFirstBallPose = Pose(10.0, 36.0, PI)
    private val firstBallPose: Pose = mirrorAlliance(rawFirstBallPose)
    private val firstBallPositionPose: Pose = mirrorAlliance(Pose(rawFirstBallPose.x + 28.0, rawFirstBallPose.y, PI))

//    private val rawCornerBallPreposition = Pose(13.0, 18.0, 7 * PI / 6)
    private val rawCornerBallPreposition = Pose(13.0, 22.0, 7 * PI / 6)
    private val cornerBallPreposition: Pose = mirrorAlliance(rawCornerBallPreposition)
    private val rawCornerBallPose = Pose(12.3, 14.0, 7 * PI / 6)
    private val cornerBallPose: Pose = mirrorAlliance(rawCornerBallPose)

    private fun initPathChain(block: PathBuilderKt.() -> Unit) = onInit { follower.pathChain { block() } }

    private val scoreFromCornerPath by initPathChain { pathLinearHeading(cornerBallPose, scorePose) }
    private val lastBallPositionPath by initPathChain {
        pathLinearHeading(cornerBallPose, cornerBallPose.withX(cornerBallPose.x + 5.0))
    }

    private val lastBallCollectPath by initPathChain {
        pathLinearHeading(
            cornerBallPose.withY(cornerBallPose.y + 5.0),
            mirrorAlliance(Pose(rawCornerBallPose.x + if (isMirrored) -5.0 else 5.0, rawCornerBallPose.y)),
            mirrorAlliance(Pose(rawCornerBallPose.x + if (isMirrored) -10.0 else 10.0, rawCornerBallPose.y)),
            mirrorAlliance(Pose(rawCornerBallPose.x, rawCornerBallPose.y, PI)),
        ) {
            temporalCallback(200.milliseconds) { intake.isRunning = true }
        }
    }

    @Volatile
    private var fiducialId = 21
    private val scorePreload by initPathChain { pathLinearHeading(startPose, scorePose) }
    private val cornerPath by onInit {
        follower.pathChain(decelerationType = PathChain.DecelerationType.NONE) {
            pathLinearHeading(scorePose, cornerBallPreposition) {
                parametricCallback(0.6) { this@FarAuto.follower.setMaxPower(0.6) }
            }
        }
    }
    private val firstBalls by initPathChain {
        pathLinearHeading(scorePose, mirrorAlliance(Pose(rawScorePose.x, rawFirstBallPose.y)), firstBallPositionPose)
        pathToPose(firstBallPose)
    }
    private val scoreFirstBalls by initPathChain { pathLinearHeading(firstBallPose, scorePose, endTime = 0.8) }

    private val leavePathChain by initPathChain {
        val leavePose = mirrorAlliance(Pose(rawScorePose.x , rawScorePose.y + 10.0))
        pathConstantHeading(if (isMirrored) PI else -PI, scorePose, leavePose)
    }

    override fun init() {
        super.init()

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
            sorter.prepareFastShoot()
            delay(2.seconds)
            sorter.fastShoot()
            intake.isRunning = true
            follower.setMaxPower(0.9)
            follower.followAndIntake(intake, sorter, firstBalls, isDetectingColor = true)
            intake.isOuttake = true
            follower.setMaxPower(1.0)
            shooterJob = shooter.shoot(distanceFlow)
            sorter.position = 0.5
            follower.followSuspend(scoreFirstBalls)
            sorter.prepareFastShoot()
            delay(200.milliseconds)
            shooter.alignToPose(follower.pose, goalPose)
            sorter.fastShoot()
            val n = 3
            repeat(n) { index ->
                follower.setMaxPower(0.9)
                follower.followAndIntake(
                    intake,
                    sorter,
                    cornerPath,
                    lastBallPositionPath,
//                    lastBallCollectPath,
                    isDetectingColor = true,
                    timeout = 10.seconds
                )
                delay(500.milliseconds)
                if (sorter.isFull) {
                    intake.isOuttake = true
                }
                follower.setMaxPower(1.0)
                shooterJob = shooter.shoot(distanceFlow)
                follower.followSuspend(scoreFromCornerPath)
                if (index >= n - 2) {
                    sorter.prepareFastShoot()
                    shooter.alignToPose(follower.pose, goalPose, 0.0)
                    sorter.fastShoot()
                }
                else {
                    shooter.alignToPose(follower.pose, goalPose, 0.0)
                    shootPattern(sorter, shooterJob, pattern)
                }
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