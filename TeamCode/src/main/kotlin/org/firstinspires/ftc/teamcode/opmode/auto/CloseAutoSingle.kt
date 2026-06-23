package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.*
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.opmode.pattern
import org.firstinspires.ftc.teamcode.pedropathing.*
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.getShooterPose
import org.firstinspires.ftc.teamcode.shooter.prepareFastShoot
import org.firstinspires.ftc.teamcode.shooter.shootPattern
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.toArtefactList
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class CloseAutoSingle(alliance: Alliance) : CoroutineOpMode() {
    private lateinit var follower: Follower
    private lateinit var sorter: Sorter
    private lateinit var intake: Intake
    private lateinit var shooter: Shooter
    private lateinit var limelight: Limelight3A

    /**
     * Alias to create a non-mirrored [com.pedropathing.geometry.Pose] to allow declaring the [Pose] function
     */
    private fun rawPose(x: Double, y: Double, headingRad: Double = 0.0) =
        com.pedropathing.geometry.Pose(x, y, headingRad)

    /**
     * Alias to `mirrorAlliance(Pose(x, y, headingRad))` to allow declaring mirrored [Pose]s more easily
     */
    private fun Pose(x: Double, y: Double, headingRad: Double = 0.0) =
        mirrorAlliance(rawPose(x, y, headingRad))

    private val isMirrored = alliance == Alliance.RED
    private fun mirrorAlliance(pose: Pose): Pose = if (isMirrored) pose.mirror() else pose

    private val goalPose = Pose(12.0, 141.5 - 12.0)

    private val startPose = Pose(19.5, 122.0, Math.toRadians(144.0))
    private val scorePose = Pose(62.0, 78.0, Math.toRadians(-140.0))
    private val scoreLastBallsPose = Pose(57.0, 111.0)

    private val collectBalls1Pose = Pose(12.0, 57.0)
    private val collectBalls2Pose = Pose(18.0, 84.0)

    private val collectBalls3Pose = Pose(12.0, 36.0)

    private val gatePose = Pose(12.0, 60.3, Math.toRadians(145.0))

    private inner class Paths {
        private fun pathChain(block: PathBuilderKt.() -> Unit) = follower.pathChain(block = block)

        val collectBalls1 = pathChain { path(scorePose, Pose(39.0, 57.0), collectBalls1Pose) }
        val collectBalls2 = pathChain { path(scorePose, Pose(39.0, 84.0), collectBalls2Pose) }

        val collectBalls3 = pathChain {
            val collectBalls3ControlPoint = Pose(50.0, 40.5,PI)
            path(scorePose, Pose(34.0, 36.0, PI), collectBalls3Pose)
        }
        val collectGateBalls = pathChain {
            val hitGatePose = Pose(24.0, 61.5,Math.toRadians(145.0) )
            path(scorePose, Pose(40.0, 59.0), hitGatePose)
            pathToPose(gatePose)
        }

        val scorePreload = pathChain {
            val preloadScorePose = Pose(60.0, 78.0, Math.toRadians(-140.0))
            pathLinearHeading(startPose, preloadScorePose, endTime = .75) {
                launchFromCallback(0.85)
            }
        }
        val scoreBalls1 = pathChain {
            path(collectBalls1Pose, scorePose, interpolator = HeadingInterpolator.tangent.reverse()) {
                launchFromCallback(0.85)
            }
        }
        val scoreBalls2 = pathChain {
            path(collectBalls2Pose, scoreLastBallsPose, interpolator = HeadingInterpolator.tangent.reverse()) {
                launchFromCallback(0.75, shootPatternOnLaunch = true)
            }
        }
        val scoreBalls3 = pathChain {
            path(collectBalls3Pose, scorePose, interpolator = HeadingInterpolator.tangent.reverse()) {
                launchFromCallback(0.85, shootPatternOnLaunch = true)
            }
        }
        val scoreGateBalls = pathChain {
            path(gatePose, scorePose, interpolator = HeadingInterpolator.tangent.reverse()) {
                launchFromCallback(0.85)
            }
        }
    }

    private lateinit var paths: Paths

    private lateinit var launchJob: Job
    private lateinit var shooterJob: Job
    private val shooterDistanceFlow = MutableStateFlow(0.0)

    @Volatile
    private var fiducialId = 21
    private var patternList: List<ArtefactType> = emptyList()

    private fun romanianSpeedLUT(distance: Double): Double {
        return 0.19856 * distance + 126.62389
    }

    private fun updateShooterUtils(robotPose: Pose) {
        val distanceMeters = shooterDistanceFlow.value
        val flightTime = distanceMeters / romanianSpeedLUT(distanceMeters)
        val currentGoalPose = rawPose(
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

    private fun CallbackBuilderKt.launchFromCallback(
        parametricValue: Double,
        shootPatternOnLaunch: Boolean = false
    ) {
        addCallback { sorter.prepareFastShoot() }
        parametricCallback(parametricValue) {
            intake.isServoRunning = true
            launchJob = opModeScope.launch {
                if (shootPatternOnLaunch && patternList.isNotEmpty()) {
                    shootPattern(sorter, shooterJob, patternList)
                } else {
                    sorter.fastShoot()
                }
            }
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    private fun updatePatternFromJob(patternJob: Deferred<Int>) {
        patternList = runCatching { patternJob.getCompleted() }
            .getOrNull()
            ?.also { fiducialId = it }
            ?.toArtefactList()
            ?: emptyList()
        Log.d("Auto", "Fiducial id: $fiducialId")
    }

    override fun init() {
        follower = opModeGraph.follower.apply { setStartingPose(startPose) }
        sorter = opModeGraph.sorter.apply { prepareFastShoot() }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = 0.0; hood = 1.0 }
        limelight = opModeGraph.limelight.apply { pipelineSwitch(0); start() }
        telemetry = opModeGraph.telemetry
        paths = Paths()
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    override fun start() {
        val autoStartTimeNanos = System.nanoTime()
        val patternJob = opModeScope.getFiducialId(limelight)
        opModeScope.launch {
            shooterJob = shooter.shoot(shooterDistanceFlow)
            try {
                follower.followSuspendFlow(paths.scorePreload).updateShooterFollowing().collect()
                launchJob.join()
                follower.followAndIntake(intake, sorter) {
                    followSuspend(paths.collectBalls1)
                    delay(500.milliseconds)
                }
                intake.isOuttake = true
                follower.followSuspendFlow(paths.scoreBalls1).updateShooterFollowing().collect()
                launchJob.join()
                shooter.angleDegrees = if (isMirrored) 90.0 else -90.0
                repeat(2) {
                    follower.followAndIntake(intake, sorter, isDetectingColor = false, timeout = 4.seconds) {
                        followSuspend(paths.collectGateBalls)
                        holdSuspend(gatePose, 3.seconds)
                    }
                    intake.isOuttake = true
                    follower.followSuspendFlow(paths.scoreGateBalls).updateShooterFollowing().collect()
                    paths.scoreGateBalls.resetCallbacks()
                    launchJob.join()
                }

                follower.followAndIntake(intake, sorter, isDetectingColor = false) {
                    follower.followSuspend(paths.collectBalls3)
                }
                updatePatternFromJob(patternJob)
                intake.isOuttake = true
                follower.followSuspendFlow(paths.scoreBalls3).updateShooterFollowing().collect()
                launchJob.join()
                if (shooterJob.isCancelled) {
                    shooterJob = shooter.shoot(shooterDistanceFlow)
                }

                follower.followAndIntake(intake, sorter, isDetectingColor = false) {
                    follower.followSuspend(paths.collectBalls2)
                }
                updatePatternFromJob(patternJob)
                intake.isOuttake = true
                if (System.nanoTime() - autoStartTimeNanos >= 28.seconds.inWholeNanoseconds) {
                    requestOpModeStop()
                    return@launch
                }
                follower.followSuspendFlow(paths.scoreBalls2).updateShooterFollowing().collect()
                launchJob.join()
                requestOpModeStop()
            } finally {
                shooterJob.cancel()
            }
        }
    }

    override fun stop() {
        super.stop()
        limelight.stop()
        lastPose = follower.pose
        pattern = patternList
    }
}
