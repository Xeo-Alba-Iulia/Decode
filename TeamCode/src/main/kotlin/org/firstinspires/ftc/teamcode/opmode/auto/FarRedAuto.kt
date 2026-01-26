package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathLinearExperimental
import com.pedropathing.paths.pathChain
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.CoroutineOpMode
import org.firstinspires.ftc.teamcode.opmode.lastPose
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped
import kotlin.math.PI
import kotlin.math.atan2

@OptIn(PathLinearExperimental::class)
@Autonomous
class FarRedAuto : CoroutineOpMode(isAuto = true) {
    lateinit var follower: Follower
    lateinit var intake: Intake
    lateinit var sorter: Sorter
    lateinit var shooter: Shooter
    lateinit var limelight: Limelight3A
    lateinit var patternJob: Job
    lateinit var shooterJob: Job

    val startPose = Pose(84.0, 7.0, PI / 2)
    val goalPose = Pose(132.0, 132.0)
    val scorePreloadPose = Pose(
        startPose.x, 20.0, atan2(
            goalPose.y - 20.0,
            goalPose.x - startPose.x
        )
    )

    val pattern = Array(3) { ArtefactType.PURPLE }

    @Volatile
    var fiducialId = 21

    val scorePreload = pathChain(null) {
        pathLinearHeading {
            +startPose
            +scorePreloadPose
        }
    }

    val leavePathChain = pathChain(null) {
        path {
            +scorePreloadPose
            +Pose(startPose.x, 36.0)
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
    }

    override fun init_loop() {
        telemetry.addData("FiducialId", fiducialId)
        telemetry.update()
        Thread.sleep(100L)
    }

    override fun start() {
        patternJob.cancel()
        super.start()
        pattern[fiducialId - 21] = ArtefactType.GREEN
        shooterJob = shooter.shoot { goalPose.distanceFrom(scorePreloadPose) / 39.37 }
        opModeScope.launch {
//            delay(10.seconds)
            follower.setMaxPower(0.5)
            follower.followPath(scorePreload)
            while (follower.isBusy)
                ensureActive()
            launch { shootAll(shooter.stateFlow, sorter, shooterJob, *pattern) }
            val transferJob = launch {
                shooter.stateFlow
                    .map { it.canShoot }
                    .distinctUntilChanged()
                    .collect {
                        sorter.isLifting = it
                    }
            }
            shooterJob.join()
            transferJob.cancelAndJoin()
            sorter.isLifting = false
            follower.followPath(leavePathChain)
            while (follower.isBusy)
                ensureActive()
        }
    }

    override fun loop() {
        follower.update()
        telemetry.addData("Pose", follower.pose)
        if (BuildConfig.DEBUG)
            drawDebug(follower)
    }

    override fun stop() {
        super.stop()
        lastPose = follower.pose
    }
}
