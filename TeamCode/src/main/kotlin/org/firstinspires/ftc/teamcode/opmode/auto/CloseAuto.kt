package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
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
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.prepareFastShoot
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.toArtefactList
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
    private val scorePose = Pose(62.0, 78.0, Math.toRadians(-140.0))
    private val scoreLastBallsPose = Pose(57.0, 111.0)

    private val collectBalls1Pose = Pose(12.0, 57.0)
    private val collectBalls2Pose = Pose(21.0, 84.0)

    private val gatePose = Pose(14.2, 61.8, Math.toRadians(145.0))

    private val collectBalls1 = pathChain {
        path {
            +scorePose
            +Pose(39.0, 57.0)
            +collectBalls1Pose
        }
    }

    private val collectGateBalls = pathChain {
        val hitGatePose = Pose(24.0, 66.0, PI)
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

    @Volatile
    private var fiducialId = 21
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
            pathLinearHeading(endTime = .75) {
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
                    addCallback { sorter.prepareFastShoot() }
                    launchFromCallback(0.75)
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
        val patternJob = opModeScope.launch {
            while (isActive) {
                limelight?.latestResult?.fiducialResults?.singleOrNull()?.let {
                    fiducialId = it.fiducialId
                }
                delay(100L)
            }
        }
        opModeScope.launch {
            val distanceFlow = flow {
                emit(goalPose.distanceFrom(scorePose) / 39.37 - 0.1)
                follower.followSuspendFlow(scorePreload).alignShooterFollowing().collect()
                launchJob.join()
                follower.followAndIntake(intake, sorter) {
                    followSuspend(collectBalls1)
                    delay(500L)
                }
                emit(goalPose.distanceFrom(scorePose) / 39.37)
                intake.isOuttake = true
                follower.followSuspendFlow(scoreBalls1).alignShooterFollowing(5.0).collect()
                launchJob.join()
                shooter.angleDegrees = if (isMirrored) 90.0 else -90.0
                repeat(3) {
                    follower.followAndIntake(intake, sorter, isDetectingColor = false, timeout = 4.seconds) {
                        followSuspend(collectGateBalls)
                        holdSuspend(gatePose, 2.seconds)
                    }
                    intake.isOuttake = true
                    follower.followSuspendFlow(scoreGateBalls).alignShooterFollowing(4.0).collect()
                    scoreGateBalls.resetCallbacks()
                    launchJob.join()
                }
                emit(goalPose.distanceFrom(scoreLastBallsPose) / 39.37)
                follower.followAndIntake(intake, sorter, isDetectingColor = false) {
                    follower.followSuspend(collectBalls2)
                }
                patternJob.cancel()
                patternList = fiducialId.toArtefactList()
                Log.d("Auto", "Fiducial id: $fiducialId")
                intake.isOuttake = true
                follower.followSuspendFlow(scoreBalls2).alignShooterFollowing(10.0).collect()
                launchJob.join()
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