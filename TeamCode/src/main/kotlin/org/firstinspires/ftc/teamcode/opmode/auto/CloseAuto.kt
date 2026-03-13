package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.*
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.opmode.pattern
import org.firstinspires.ftc.teamcode.pedropathing.*
import org.firstinspires.ftc.teamcode.shooter.*
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.time.Duration.Companion.seconds

@OptIn(PathLinearExperimental::class)
abstract class CloseAuto(alliance: Alliance) : CoroutineOpMode() {
    private lateinit var follower: Follower
    private lateinit var sorter: Sorter
    private lateinit var intake: Intake
    private lateinit var shooter: Shooter
    private var limelight: Limelight3A? = null

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

    private val goalPose = Pose(5.0, 141.5 - 5.0)

    private val startPose = Pose(19.5, 122.0, Math.toRadians(144.0))
    private val scorePose = Pose(60.0, 78.0, Math.toRadians(-140.0))
    private val scoreLastBallsPose = Pose(57.0, 105.0)

    private val collectBalls1Pose = Pose(10.0, 57.0)
    private val collectBalls2Pose = Pose(19.0, 84.0)

    private val gatePose = Pose(13.3, 59.6, Math.toRadians(150.0))

    private val collectBalls1 = pathChain {
        path {
            +scorePose
            +Pose(39.0, 57.0)
            +collectBalls1Pose
        }
    }

    private val collectGateBalls = pathChain {
        val hitGatePose = Pose(24.0, 64.0, PI)
        path {
            +scorePose
            +Pose(40.0, 62.0)
            +hitGatePose
        }
        pathLinearHeading {
            +hitGatePose
//            +Pose(18.0, 57.0)
            +gatePose
        }
    }

    private val collectBalls2 = pathChain {
        path {
            +scorePose
            +Pose(39.0, 84.0)
            +collectBalls2Pose
        }
    }

    private lateinit var scorePreload: PathChain
    private lateinit var scoreGateBalls: PathChain
    private lateinit var scoreBalls1: PathChain
    private lateinit var scoreBalls2: PathChain

    private lateinit var launchJob: Job
    private var patternList: List<ArtefactType> = emptyList()

    private fun Flow<Pose>.alignShooterFollowing(offset: Double = 0.0) =
        onEach { pose -> shooter.alignToPose(pose, goalPose, if (isMirrored) -offset else offset); drawRobot(pose); sendPacket() }

    private fun CallbackBuilder.launchFromCallback(parametricValue: Double) {
        parametricCallback(parametricValue) {
            intake.isServoRunning = true
            launchJob = opModeScope.launch {
                sorter.fastShoot()
            }
        }
    }

    override fun init() {
        follower = opModeGraph.follower.apply { setStartingPose(startPose) }
        sorter = opModeGraph.sorter.apply { prepareFastShoot() }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = 0.0; hood = 1.0 }
        limelight = opModeGraph.limelight.apply { pipelineSwitch(0); start() }
        telemetry = opModeGraph.telemetry
        scorePreload = pathChain(follower) {
            pathLinearHeading(endTime = .8) {
                +startPose
                +Pose(60.0, 78.0, Math.toRadians(-140.0))
                callbacks {
                    addCallback { sorter.prepareFastShoot() }
                    launchFromCallback(0.85)
                }
            }
        }
        scoreBalls1 = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +collectBalls1Pose
                +scorePose
                callbacks {
                    addCallback { sorter.prepareFastShoot() }
                    launchFromCallback(0.85)
                }
            }
        }
        scoreBalls2 = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +collectBalls2Pose
                +scoreLastBallsPose
                callbacks {
                    addCallback { sorter.position = 0.5 }
                    parametricCallback(0.75) {
                        opModeScope.launch { shootPattern(sorter, launchJob, patternList) }
                    }
                }
            }
        }
        scoreGateBalls = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +gatePose
                +scorePose
                callbacks {
                    addCallback { sorter.prepareFastShoot() }
                    launchFromCallback(0.85)
                }
            }
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    override fun start() {
        opModeScope.launch {
            val distanceFlow = flow {
                emit(goalPose.distanceFrom(scorePose) / 39.37)
                follower.followSuspendFlow(scorePreload)
                    .alignShooterFollowing()
                    .collect()
                launchJob.join()
                shooter.angleDegrees = if (isMirrored) 90.0 else -90.0
                val asyncList = async { getPatternList(limelight) }
                follower.followAndIntake(intake, sorter) {
                    followSuspend(collectBalls1)
                    delay(500L)
                }
                intake.isOuttake = true
                follower.followSuspendFlow(scoreBalls1).alignShooterFollowing(9.0).collect()
                launchJob.join()
                repeat(3) {
                    follower.followAndIntake(intake, sorter, timeout = 4.seconds) {
                        followSuspend(collectGateBalls)
                        holdSuspend(gatePose, 2.seconds)
                    }
                    intake.isOuttake = true
                    follower.followSuspendFlow(scoreGateBalls).alignShooterFollowing(8.0).collect()
                    scoreGateBalls.resetCallbacks()
                    launchJob.join()
                }
                emit(goalPose.distanceFrom(scoreLastBallsPose) / 39.37)
                follower.followAndIntake(intake, sorter) {
                    follower.followSuspend(collectBalls2)
                    delay(500L)
                }
                patternList = try {
                    asyncList.getCompleted()
                } catch (_: Exception) {
                    emptyList()
                }
                intake.isOuttake = true
                follower.followSuspendFlow(scoreBalls2).alignShooterFollowing(12.0).collect()
                requestOpModeStop()
            }
            launchJob = shooter.shoot(distanceFlow)
        }
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
        lastPose = follower.pose
        pattern = patternList
    }
}