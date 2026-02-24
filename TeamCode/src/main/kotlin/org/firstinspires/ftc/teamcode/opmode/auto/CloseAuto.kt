package org.firstinspires.ftc.teamcode.opmode.auto

import android.util.Log
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.android.awaitFrame
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.auto.FarAuto.Companion.TAG
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.followAndIntake
import org.firstinspires.ftc.teamcode.pedropathing.followSuspend
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootPattern
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.toArtefactList
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
    private val rawScorePose = Pose(54.0, 86.0).run { withHeading(atan2(rawGoalPose.y - y, rawGoalPose.x - x)) }
    private val scorePose = mirrorAlliance(rawScorePose)
    private val rawFirstBallsCollectPose = Pose(18.0, 84.0, PI)
    private val firstBallsCollectPose = mirrorAlliance(rawFirstBallsCollectPose)
    private val rawSecondBallsCollectPose = Pose(10.0, 60.0, PI)
    private val secondBallsCollectPose = mirrorAlliance(rawSecondBallsCollectPose)
    private val rawFreeGoalPose = Pose(16.0, 75.0, PI / 2)
    private val freeGoalPose = mirrorAlliance(rawFreeGoalPose)

    private val rawCollectAndFreeGoalPose = Pose(15.0,62.5,(5 * PI) / 6)

    private val collectAndFreeGoalPose = mirrorAlliance(rawCollectAndFreeGoalPose)

    private val rawCollectAndFreeGoalPose2 = Pose(9.0,51.0,Math.toRadians(100.0))

    private val collectAndFreeGoalPose2 = mirrorAlliance(rawCollectAndFreeGoalPose2)

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
            +mirrorAlliance(Pose(rawFirstBallsCollectPose.x + 12.0, rawFreeGoalPose.y + 17.0))
            +freeGoalPose
        }
    }
    private val freeGateAndCollect = pathChain {
        pathLinearHeading {
            +scorePose
            +mirrorAlliance(Pose(scorePose.x, collectAndFreeGoalPose.y-4))
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
            +freeGoalPose
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
        sorter = opModeGraph.sorter.apply { position = 0.5; isLifting = false }
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
                    parametricCallback(0.35) { intake.isRunning = true; follower.setMaxPower(1.0) }
                }
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
            var job = shooter.shoot { distance }
            val pattern: Int?
            withContext(Dispatchers.Default) {
                pattern = coroutineScope {
                    val patternId = async {
                        var id = 0
                        while (id == 0) {
                            delay(50.milliseconds)
                            limelight?.latestResult?.fiducialResults?.singleOrNull()
                                ?.let { id = it.fiducialId }
                        }
                        id
                    }
                    withContext(followerDispatcher) { follower.followSuspend(scorePreload) }
                    launch { delay(0.milliseconds) }
                    val result = withTimeoutOrNull(0.seconds) { patternId.await() }
                    patternId.cancel()
                    shooter.angleDegrees = 0.0
                    Log.d(TAG, "Detected $result")
                    result
                }
            }
            val patternList = pattern?.toArtefactList() ?: emptyList()
            shooter.alignToPose(follower.pose, goalPose)
            intake.isOuttake = true
            delay(300L)
            intake.isServoRunning = true
            shootPattern(shooter.stateFlow, sorter, job, patternList)
            follower.setMaxPower(1.0)
            follower.followAndIntake(intake, sorter, collectBalls1)
            follower.setMaxPower(1.0)
            follower.followSuspend(freeGate, maxPower = 1.0)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls1)
            shooter.alignToPose(follower.pose, goalPose)
            intake.isOuttake = true
            delay(300L)
            intake.isServoRunning = true
            shootPattern(shooter.stateFlow, sorter, job, patternList)
            follower.followAndIntake(intake, sorter, collectBalls2)
            follower.setMaxPower(1.0)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls2)
            shooter.alignToPose(follower.pose, goalPose)
            intake.isOuttake = true
            delay(300L)
            intake.isServoRunning = true
            shootPattern(shooter.stateFlow, sorter, job, patternList)
            follower.followAndIntake(intake, sorter) {
                    follower.followSuspend(freeGateAndCollect)
                    delay(2.seconds)
            }
            follower.setMaxPower(1.0)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls2)
            shooter.alignToPose(follower.pose, goalPose)
            intake.isOuttake = true
            delay(300L)
            intake.isServoRunning = true
           shootPattern(shooter.stateFlow, sorter, job, patternList    )
            follower.followAndIntake(intake, sorter) {
                follower.followSuspend(freeGateAndCollect)
                delay(2.seconds)
            }
            follower.setMaxPower(1.0)
            job = shooter.shoot { distance }
            follower.followSuspend(scoreBalls2)
            shooter.alignToPose(follower.pose, goalPose)
            shootPattern(shooter.stateFlow, sorter, job, patternList)
            follower.followSuspend(leavePath)
        }
    }

    override fun stop() {
        super.stop()
        limelight?.stop()
        lastPose = follower.pose
    }
}