package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.coroutines.cancellation.CancellationException

/**Control Scheme:
 * GAMEPAD 1 (Driver):
 * - Left Stick: Strafe/Forward movement
 * - Right Stick X: Rotation
 * - A: Start intake
 * - B: Stop intake
 * - Y: Toggle field-centric mode
 * - Left Bumper: Slow mode (hold)
 *
 * GAMEPAD 2 (Sisteme):
 * - A: Start shooting sequence
 * - B: Stop shooting sequence
 * - X: Decrease shooter velocity
 * - Y: Increase shooter velocity
 * - Dpad Up/Down: Adjust hood position
 * - Dpad Left/Right: Adjust shooter angle
 * - Left Bumper: Prepare shoot (purple artefact)
 * - Right Bumper: Prepare shoot (green artefact)
 * - Back: Prepare shoot (any artefact)
 * - Start: Prepare intake
 * - Left Trigger: Decrease sorter position
 * - Right Trigger: Increase sorter position
 * - Left Stick Button: Intake purple artefact
 * - Right Stick Button: Intake green artefact
 */
@Config
abstract class FullTeleOp : CoroutineOpMode() {
    // Subsystems
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var sorter: Sorter
    lateinit var follower: Follower
    lateinit var limelight: Limelight3A

    // Drive state
    private var isRobotCentric = false

    // Shooter state
    private var currentShooterJob: Job? = null
    private var turretOffset = 0.0

    abstract val goalPose: Pose
    abstract val startPose: Pose
    abstract val limelightPipeline: Int

    // Speed control
    companion object {
        const val HOOD_ADJUSTMENT_STEP = 0.01
        const val ANGLE_ADJUSTMENT_STEP = -0.5
        const val VELOCITY_ADJUSTMENT_STEP = 10.0
        const val SORTER_POSITION_MULTIPLIER = 0.005
        const val TURET_OFFSET_ADJUSTMENT_STEP = 1
    }

    override fun init() {
        telemetry = opModeGraph.telemetry
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter
        sorter = opModeGraph.sorter
        follower = opModeGraph.follower
        limelight = opModeGraph.limelight
        observers += sorter
        limelight.pipelineSwitch(limelightPipeline)
    }

    override fun start() {
        super.start()
        follower.setStartingPose(lastPose ?: startPose)
        follower.startTeleopDrive(true)
        opModeGraph.tickFlow
            .onEach {
                handleSorter()
            }
            .launchIn(opModeScope)

        shooter.stateFlow
            .map { (_, canShoot) -> canShoot }
            .filter { it }
            .onEach {
                gamepad2.rumble(200)
            }
            .launchIn(opModeScope)

//        intake.artefactFlow
//            .onEach {
//                telemetry.addData("ArtefactType", it)
//            }
//            .zipWithNext()
//            .buffer(capacity = 2)
//            .onEach { (previous, _) ->
//                if (previous != null) {
//                    delay(160L)
//                    sorter.intake(previous)
//                }
//            }
//            .launchIn(opModeScope)

        limelight.start()
    }

    override fun loop() {
        follower.setTeleOpDrive(
            /* forward = */ -gamepad1.left_stick_y.toDouble() * if (!isRobotCentric) -1 else 1,
            /* strafe = */ -gamepad1.left_stick_x.toDouble() * if (!isRobotCentric) -1 else 1,
            /* turn = */ -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ isRobotCentric
        )
        follower.update()
        drawDebug(follower)
        when {
            gamepad1.rightBumperWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }
        handleShooter()
        telemetry.addData("Pose", follower.pose)

        if (gamepad2.dpad_right) {
            turretOffset -= TURET_OFFSET_ADJUSTMENT_STEP
        }
        if (gamepad2.dpad_left) {
            turretOffset += TURET_OFFSET_ADJUSTMENT_STEP
        }

        telemetry.addData("Turret Offset", turretOffset)
        telemetry.addData("shooter Angle", shooter.angleDegrees)

        shooter.alignToPose(follower.pose, goalPose, turretOffset)

        val distanceToGoal = goalPose.distanceFrom(follower.pose) / 40.0
        limelight.latestResult.fiducialResults.singleOrNull()?.let {
            if (gamepad1.crossWasPressed())
                turretOffset -= it.targetXDegrees
            val position = it.targetPoseCameraSpace.position
//            distanceToGoal = sqrt(position.x.pow(2) + position.y.pow(2) + position.z.pow(2))
        }
        telemetry.addData("AprilTag Distance", distanceToGoal)
        shooter.hood = hoodLiftByDistance(distanceToGoal)

        if (gamepad1.triangleWasPressed())
            isRobotCentric = !isRobotCentric

        telemetry.addData("Shooter speed", shooter.stateFlow.value.velocity)
        telemetry.addData("Shooter hood", shooter.hood)
        telemetry.addData("Sorter size", sorter.size)
        telemetry.addData("Field Centric", !isRobotCentric)
        telemetry.update()
    }

    private fun handleShooter() {
        val autoShoot = gamepad2.triangleWasPressed()

        // Start/stop shooting sequence
        if ((gamepad2.aWasPressed() || autoShoot) && currentShooterJob?.isCancelled != false) {
            currentShooterJob = shooter.shoot { goalPose.distanceFrom(follower.pose) / 39.37 }
        }

        if (autoShoot)
            opModeScope.launch {
                try {
                    shootAll(shooter.stateFlow, sorter, currentShooterJob!!)
                } catch (e: Exception) {
                    if (e is CancellationException) return@launch
                    RobotLog.dd("FullTeleOp", e, "Shooter problem")
                }
            }

        if (gamepad2.bWasPressed()) {
            currentShooterJob?.cancel()
        }

        // Adjust shooter velocity
        if (gamepad2.y) shooter.velocity += VELOCITY_ADJUSTMENT_STEP
        if (gamepad2.x) shooter.velocity -= VELOCITY_ADJUSTMENT_STEP

        // Adjust hood position
        if (gamepad2.dpad_up) shooter.hood += HOOD_ADJUSTMENT_STEP
        if (gamepad2.dpad_down) shooter.hood -= HOOD_ADJUSTMENT_STEP

        // Adjust shooter angle
        when {
            gamepad2.dpad_right -> shooter.angleDegrees += ANGLE_ADJUSTMENT_STEP
            gamepad2.dpad_left -> shooter.angleDegrees -= ANGLE_ADJUSTMENT_STEP
        }
    }

    private suspend fun handleSorter() {
        // Intake artefacts (using stick buttons to avoid conflicts)
        when {
            gamepad2.rightBumperWasPressed() -> sorter.intake(ArtefactType.PURPLE)
            gamepad2.leftBumperWasPressed() -> sorter.intake(ArtefactType.GREEN)
        }

        // Prepare shoot
        when {
            gamepad2.rightStickButtonWasPressed() -> sorter.prepareShoot(ArtefactType.PURPLE)
            gamepad2.leftStickButtonWasPressed() -> sorter.prepareShoot(ArtefactType.GREEN)
            gamepad2.backWasPressed() -> sorter.prepareShoot()
        }

        // Prepare intake
        if (gamepad2.startWasPressed()) {
            sorter.prepareIntake()
        }

        // Using left trigger as boolean for lifting
        when {
            gamepad2.squareWasPressed() -> sorter.isLifting = true
            gamepad2.squareWasReleased() -> sorter.isLifting = false
        }

        // Adjust sorter position with triggers
        sorter.position += (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() * SORTER_POSITION_MULTIPLIER
    }
    private fun hoodLiftByDistance(distance: Double) = when (distance) {
        in 2.8..5.0 -> 0.55
        in 1.0..2.8 -> 0.2
        in 0.0..1.0 -> 0.1
        else -> error("Distance not in interval")
    }
}
