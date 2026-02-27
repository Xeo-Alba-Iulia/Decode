package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.flowOf
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followAndIntake
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.pedropathing.pathConstraints
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootPattern
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2
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

    private val rawStartPose = Pose(32.0, 135.0)
    private val startPose = mirrorAlliance(rawStartPose)
    private val rawGoalPose = Pose(13.0, 141.5 - 13.0)
    private val goalPose = mirrorAlliance(rawGoalPose)
    private val rawScorePose = Pose(58.0, 86.0).run { withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x)) }
    private val scorePose = mirrorAlliance(rawScorePose)
    private val rawFirstBallsCollectPose = Pose(18.0, 84.0, PI)
    private val firstBallsCollectPose = mirrorAlliance(rawFirstBallsCollectPose)
    private val rawSecondBallsCollectPose = Pose(12.0, 60.0, PI)
    private val secondBallsCollectPose = mirrorAlliance(rawSecondBallsCollectPose)
    private val rawFreeGoalPose = Pose(10.0, 76.0, PI / 2)
    private val freeGoalPose = mirrorAlliance(rawFreeGoalPose)

    private val rawCollectAndFreeGoalPose = Pose(16.6, 62.5, Math.toRadians(150.0))

    private val collectAndFreeGoalPose = mirrorAlliance(rawCollectAndFreeGoalPose)

    private val rawCollectAndFreeGoalPose2 = Pose(9.0, 51.0, Math.toRadians(120.0))

    private val collectAndFreeGoalPose2 = mirrorAlliance(rawCollectAndFreeGoalPose2)

    private val rawFreeGatePose = Pose(19.0, 81.0, Math.toRadians(180.0))

    private val freeGatePose = mirrorAlliance(rawFreeGatePose)

    private val scorePreload = pathChain {
        pathLinearHeading(endTime = .8) {
            +startPose
            +scorePose
        }
    }

    private val collectBalls1 = pathChain {
        pathLinearHeading(endTime = 0.45) {
            +scorePose
            +firstBallsCollectPose
        }
    }
    private val freeGate = pathChain {
        pathConstantHeading(scorePose.heading) {
            +scorePose
            +mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 10.0, rawFirstBallsCollectPose.y, PI))
        }
        pathToPose(mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 10.0, rawFreeGatePose.y, PI)))
        pathToPose(freeGatePose)
    }
    private val freeGateAndCollect = pathChain {
        pathLinearHeading(pathConstraints = pathConstraints.copy().apply {
            brakingStart = 0.5
            brakingStrength = 0.85
        }) {
            +scorePose
            +mirrorAlliance(Pose(scorePose.x, collectAndFreeGoalPose.y - 4))
            +collectAndFreeGoalPose
        }
    }
    private val freeGateAndCollect2 = pathChain {
        pathLinearHeading {
            +scorePose
            +mirrorAlliance(Pose(scorePose.x, collectAndFreeGoalPose2.y))
            +collectAndFreeGoalPose2
        }
    }

    private val scoreBalls1 = pathChain {
        pathLinearHeading {
            +freeGatePose
            +scorePose
        }
    }
    private lateinit var collectBalls2: PathChain
    private lateinit var scoreBalls2: PathChain
    private val leavePath = pathChain {
        pathConstantHeading(scorePose.heading) {
            +scorePose
            +mirrorAlliance(Pose(rawScorePose.x - 15.0, rawScorePose.y))
        }
    }

    lateinit var launchJob: Job

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
                    parametricCallback(0.35) { intake.isRunning = true; follower.setMaxPower(0.7) }
                }
            }
        }
        scoreBalls2 = pathChain(follower) {
            path(interpolator = HeadingInterpolator.tangent.reverse()) {
                +secondBallsCollectPose
                +scorePose
                callbacks { parametricCallback(0.60) { launchJob.start() } }
            }
        }
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    override fun start() {
        val distance = goalPose.distanceFrom(scorePose) / 39.37
        val followerDispatcher = Dispatchers.Default.limitedParallelism(1)
        intake.isServoRunning = true
        opModeScope.launch(followerDispatcher) {
            while (true) {
                drawDebug(follower)
                yield()
            }
        }
        opModeScope.launch(followerDispatcher) {
            val patternList = async(Dispatchers.IO) { getPatternList(limelight) }
            var job = shooter.shoot(flowOf(distance))
            follower.followSuspend(scorePreload)
            shooter.alignToPose(follower.pose, goalPose)
            outtakeBall(intake)
            shootPattern(sorter, job, emptyList())
            follower.setMaxPower(0.8)
            follower.followAndIntake(intake, sorter, collectBalls1)
            follower.setMaxPower(1.0)
            follower.followSuspend(freeGate)
            job = shooter.shoot(flowOf(distance))
            follower.followSuspend(scoreBalls1)
            shooter.alignToPose(follower.pose, goalPose)
            outtakeBall(intake)
            shootPattern(sorter, job, patternList.await())
            follower.setMaxPower(1.0)
            follower.followAndIntake(intake, sorter, collectBalls2)
            follower.setMaxPower(1.0)
            job = shooter.shoot(flowOf(distance))
            shooter.angleDegrees = -75.0 * if (isMirrored) -1 else 1
            launchJob =
                launch(start = CoroutineStart.LAZY) { shootPattern(sorter, job, emptyList()) }
            follower.followSuspend(scoreBalls2)
            intake.isOuttake = true
            delay(300.milliseconds)
            intake.isOuttake = false
            launchJob.join()
            follower.followAndIntake(intake, sorter) {
                follower.followSuspend(freeGateAndCollect)
                delay(5.seconds)
            }
            follower.setMaxPower(1.0)
            job = shooter.shoot(flowOf(distance))
            launchJob =
                launch(start = CoroutineStart.LAZY) { shootPattern(sorter, job, emptyList()) }
            follower.followAndIntake(intake, sorter, scoreBalls2)
            launchJob.join()
            follower.followAndIntake(intake, sorter) {
                follower.followSuspend(freeGateAndCollect)
                delay(3.seconds)
            }
            follower.setMaxPower(1.0)
            job = shooter.shoot(flowOf(distance))
            launchJob =
                launch(start = CoroutineStart.LAZY) { shootPattern(sorter, job, patternList.await()) }
            follower.followAndIntake(intake, sorter, scoreBalls2)
            delay(1000L)
            follower.followSuspend(leavePath)
        }
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
        lastPose = follower.pose
    }
}