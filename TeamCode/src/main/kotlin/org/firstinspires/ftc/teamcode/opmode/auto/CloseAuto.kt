package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.flowOf
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
import kotlin.math.atan2
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

    private val rawStartPose = Pose(32.0, 135.0)
    private val startPose = mirrorAlliance(rawStartPose)

    private val rawGoalPose = Pose(13.0, 141.5 - 13.0)
    private val goalPose = mirrorAlliance(rawGoalPose)

    private val rawScorePose = Pose(58.0, 90.0).run { withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x)) }
    private val scorePose = mirrorAlliance(rawScorePose)

    private val rawFirstBallsCollectPose = Pose(17.5, 88.0, PI)
    private val firstBallsCollectPose = mirrorAlliance(rawFirstBallsCollectPose)

    private val rawSecondBallsCollectPose = Pose(11.0, 60.0, PI)
    private val secondBallsCollectPose = mirrorAlliance(rawSecondBallsCollectPose)

    private val rawCollectAndFreeGoalPose = Pose(13.8, 61.0, Math.toRadians(150.0))
    private val collectAndFreeGoalPose = mirrorAlliance(rawCollectAndFreeGoalPose)

    private val rawFreeGatePose = Pose(19.4, 79.8, Math.toRadians(180.0))
    private val freeGatePose = mirrorAlliance(rawFreeGatePose)

    private lateinit var scorePreload: PathChain

    private val collectBalls1 = pathChain {
        path {
            +scorePose
            +firstBallsCollectPose
        }
    }
    private val freeGate = pathChain {
        pathConstantHeading(scorePose.heading) {
            +scorePose
            +mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 5.0, rawFirstBallsCollectPose.y, PI))
        }
        pathToPose(mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 5.0, rawFreeGatePose.y, PI)))
        pathToPose(freeGatePose)
    }
    private val freeGateAndCollect = pathChain {
        path(
            pathConstraints = pathConstraints.copy().apply {
                brakingStart = 0.5
                brakingStrength = 1.0
            },
            interpolator = HeadingInterpolator.piecewise(
                HeadingInterpolator.PiecewiseNode(0.0, 0.65, HeadingInterpolator.tangent),
                HeadingInterpolator.PiecewiseNode(
                    0.65,
                    1.0,
                    HeadingInterpolator.constant(collectAndFreeGoalPose.heading)
                )
            )
        ) {
            +scorePose
            +collectAndFreeGoalPose
        }
    }

    private lateinit var scoreBalls1: PathChain
    private lateinit var collectBalls2: PathChain
    private lateinit var scoreBalls2: PathChain
    private lateinit var scoreBallsGate: PathChain
    private val leavePath = pathChain {
        path {
            +scorePose
            +mirrorAlliance(Pose(rawScorePose.x - 25.0, rawScorePose.y - 20.0))
        }
    }

    lateinit var launchJob: Job
    var patternList: List<ArtefactType> = emptyList()

    override fun init() {
        follower = opModeGraph.follower.apply { setStartingPose(startPose) }
        sorter = opModeGraph.sorter.apply { prepareFastShoot() }
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter.apply { angleDegrees = -3.0 * if (isMirrored) -1 else 1 }
        limelight = opModeGraph.limelight
        telemetry = opModeGraph.telemetry
        limelight?.pipelineSwitch(0)
        limelight?.start()
        scorePreload = pathChain(follower) {
            pathLinearHeading(endTime = .55) {
                +startPose
                +scorePose
                callbacks {
                    parametricCallback(0.7) {
                        launchJob.start()
                    }
                }
            }
        }
        scoreBalls1 = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +firstBallsCollectPose
                +scorePose
                callbacks {
                    parametricCallback(0.8) {
                        launchJob.start()
                    }
                }
            }
        }
        collectBalls2 = pathChain(follower) {
            pathLinearHeading(endTime = 0.6) {
                +scorePose
                +mirrorAlliance(Pose(rawScorePose.x, rawSecondBallsCollectPose.y))
                +mirrorAlliance(Pose(rawScorePose.x - 4.0, rawSecondBallsCollectPose.y))
                +secondBallsCollectPose
            }
        }
        scoreBalls2 = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +secondBallsCollectPose
                +scorePose
                callbacks {
                    parametricCallback(0.78) {
                        launchJob.start()
                    }
                }
            }
        }
        scoreBallsGate = pathChain(follower) {
            path(
                interpolator = HeadingInterpolator.piecewise(
                    HeadingInterpolator.PiecewiseNode(
                        0.0,
                        0.12,
                        HeadingInterpolator.constant(collectAndFreeGoalPose.heading)
                    ),
                    HeadingInterpolator.PiecewiseNode(0.12, 1.0, HeadingInterpolator.tangent.reverse())
                )
            ) {
                +collectAndFreeGoalPose
                +scorePose
                callbacks {
                    parametricCallback(0.78) {
                        launchJob.start()
                    }
                }
            }
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    override fun start() {
        val distanceFlow = flowOf(goalPose.distanceFrom(scorePose) / 39.37)
        val followerDispatcher = Dispatchers.Default.limitedParallelism(1)
        intake.isServoRunning = true
        if (BuildConfig.DEBUG)
            opModeScope.launch(followerDispatcher) {
                while (true) {
                    drawDebug(follower)
                    yield()
                }
            }
        opModeScope.launch(followerDispatcher) {
            follower.setMaxPower(0.8)
            val pattern = async(Dispatchers.IO) { getPatternList(limelight) }
            var job = shooter.shoot(distanceFlow)
            launchJob = launch(start = CoroutineStart.LAZY) { fastShoot(sorter, job) }
            follower.followSuspend(scorePreload)
            launchJob.join()
            follower.followAndIntake(intake, sorter, collectBalls2)
            shooter.angleDegrees = -75.0 * if (isMirrored) -1 else 1
            job = shooter.shoot(distanceFlow)
            sorter.prepareFastShoot()
            intake.isOuttake = true
            launchJob = launch(start = CoroutineStart.LAZY) { fastShoot(sorter, job) }
            follower.followSuspend(scoreBalls2)
            intake.isServoRunning = true
            launchJob.join()
            follower.followAndIntake(intake, sorter) {
                follower.followSuspend(freeGateAndCollect)
                holdSuspend(collectAndFreeGoalPose, 3.seconds)
            }
            job = shooter.shoot(distanceFlow)
            sorter.prepareFastShoot()
            intake.isOuttake = true
            launchJob = launch(start = CoroutineStart.LAZY) { fastShoot(sorter, job) }
            follower.followSuspend(scoreBallsGate)
            intake.isServoRunning = true
            launchJob.join()
            follower.followAndIntake(intake, sorter) {
                follower.followSuspend(freeGateAndCollect)
                holdSuspend(collectAndFreeGoalPose, 3.seconds)
            }
            job = shooter.shoot(distanceFlow)
            patternList = pattern.await()
            sorter.prepareFastShoot()
            launchJob = launch(start = CoroutineStart.LAZY) { fastShoot(sorter, job) }
            intake.isOuttake = true
            scoreBallsGate.resetCallbacks()
            follower.followSuspend(scoreBallsGate)
            intake.isServoRunning = true
            launchJob.join()
            follower.setMaxPower(0.7)
            follower.followAndIntake(intake, sorter, collectBalls1, colorList = listOf(PURPLE, PURPLE, GREEN))
            shooter.alignToPose(mirrorAlliance(rawScorePose.withHeading(PI)), goalPose, if (isMirrored) 1.0 else -1.0)
            job = shooter.shoot(distanceFlow)
            sorter.position = 0.5
            launchJob = launch(start = CoroutineStart.LAZY) { shootPattern(sorter, job, patternList) }
            follower.followSuspend(scoreBalls1)
            launchJob.join()
            follower.followSuspend(leavePath)
        }
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
        lastPose = follower.pose
        pattern = patternList
    }
}