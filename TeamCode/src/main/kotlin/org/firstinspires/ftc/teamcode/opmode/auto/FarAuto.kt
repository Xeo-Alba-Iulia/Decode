package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.*
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.*
import org.firstinspires.ftc.teamcode.shooter.*
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class FarAuto(alliance: Alliance) : CoroutineOpMode() {
    val follower by onInit { opModeGraph.follower.apply { setStartingPose(startPose) } }
    val intake by onInit { opModeGraph.intake }
    val sorter by onInit { opModeGraph.sorter.apply { prepareFastShoot() } }
    val shooter: Shooter by onInit { opModeGraph.shooter.apply { angleDegrees = 0.0; hood = 1.0 } }
    private lateinit var launchJob: Job
    private lateinit var shooterJob: Job
    private val shooterDistanceFlow = MutableStateFlow(0.0)

    private val isMirrored = alliance == Alliance.RED

    // helper to mirror a pose only when needed
    private fun mirrorAlliance(p: Pose): Pose = if (isMirrored) p.mirror() else p

    // raw poses (defined for the blue alliance), then mirrored when needed
    private val startPose: Pose = mirrorAlliance(Pose(60.0, 7.0, PI / 2))
    private val rawGoalPose = Pose(0.0, 144.0)
    private val goalPose = mirrorAlliance(rawGoalPose)
    private val rawScorePose: Pose = Pose(60.0, 20.0, PI)
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

    private val scoreFromCornerPath by initPathChain {
        pathLinearHeading(cornerBallPose, scorePose) {
            launchFromCallback(0.85)
        }
    }
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

    private val scorePreload by initPathChain {
        pathLinearHeading(startPose, scorePose) {
            launchFromCallback(2.5)
        }
    }
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
    private val scoreFirstBalls by initPathChain {
        pathLinearHeading(firstBallPose, scorePose, endTime = 0.8) {
            launchFromCallback(0.85)
        }
    }

    private val leavePathChain by initPathChain {
        val leavePose = mirrorAlliance(Pose(rawScorePose.x , rawScorePose.y + 10.0))
        pathConstantHeading(if (isMirrored) PI else -PI, scorePose, leavePose)
    }

    private fun romanianSpeedLUT(distance: Double): Double {
        return 0.19856 * distance + 126.62389
    }

    private fun updateShooterUtils(robotPose: Pose) {
        val distanceMeters = shooterDistanceFlow.value
        val flightTime = distanceMeters / romanianSpeedLUT(distanceMeters)
        val currentGoalPose = Pose(
            goalPose.x - follower.velocity.xComponent * flightTime,
            goalPose.y - follower.velocity.yComponent * flightTime,
        )
        val shooterPose = getShooterPose(robotPose)
        shooter.alignToPose(robotPose, currentGoalPose)
        shooterDistanceFlow.value = hypot(
            currentGoalPose.x - shooterPose.x,
            currentGoalPose.y - shooterPose.y,
        ) / 39.37
        telemetry.addData("Shooter Distance", shooterDistanceFlow.value)
    }

    private fun Flow<Pose>.updateShooterFollowing() =
        onEach { pose ->
            updateShooterUtils(pose)
            drawRobot(pose)
            sendPacket()
        }

    private fun CallbackBuilderKt.launchFromCallback(parametricValue: Double) {
        addCallback { sorter.prepareFastShoot() }
        parametricCallback(parametricValue) {
            intake.isServoRunning = true
            launchJob = opModeScope.launch {
                sorter.fastShoot()
            }
        }
    }

    override fun init() {
        super.init()

        intake.stateFlow
            .onEach {
                val packet = TelemetryPacket().apply { put("Intake State", it) }
                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }
            .launchIn(opModeScope + Dispatchers.IO)
    }

    override fun init_loop() {
        telemetry.update()
        Thread.sleep(100L)
    }

    override fun start() {
        super.start()
        val followerDispatcher = Dispatchers.Default.limitedParallelism(1)
        opModeScope.launch(followerDispatcher) {
            shooterJob = shooter.shoot(shooterDistanceFlow)
            try {
                follower.followSuspendFlow(scorePreload).updateShooterFollowing().collect()
                delay(2.seconds)
                launchJob.join()
                intake.isRunning = true
                follower.setMaxPower(0.9)
                follower.followAndIntake(intake, sorter, firstBalls, isDetectingColor = false)
                intake.isOuttake = true
                follower.setMaxPower(1.0)
                follower.followSuspendFlow(scoreFirstBalls).updateShooterFollowing().collect()
                launchJob.join()
                repeat(3) {
                    follower.setMaxPower(0.9)
                    follower.followAndIntake(
                        intake,
                        sorter,
                        cornerPath,
                        lastBallPositionPath,
                        lastBallCollectPath,
                        isDetectingColor = false,
                        timeout = 10.seconds
                    )
                    delay(500.milliseconds)
                    intake.isOuttake = true
                    follower.setMaxPower(1.0)
                    follower.followSuspendFlow(scoreFromCornerPath).updateShooterFollowing().collect()
                    scoreFromCornerPath.resetCallbacks()
                    launchJob.join()
                }
                follower.followSuspend(leavePathChain)
                requestOpModeStop()
            } finally {
                shooterJob.cancel()
            }
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