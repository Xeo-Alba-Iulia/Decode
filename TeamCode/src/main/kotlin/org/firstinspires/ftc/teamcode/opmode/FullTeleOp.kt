package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.config.Config
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.shootAll
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.coroutines.cancellation.CancellationException
import kotlin.math.absoluteValue
import kotlin.math.sign

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

    fun deadZone(value: Double, deadZone: Double): Double {
        val valueAbs = value.absoluteValue
        val valueSgn = value.sign
        return if (valueAbs < deadZone) 0.0 else (valueAbs - deadZone) * valueSgn * (1.0 / (1.0 - deadZone))
    }

    // Speed control
    companion object {
        @JvmField
        var HOOD_ADJUSTMENT_STEP = 0.01
        @JvmField
        var ANGLE_ADJUSTMENT_STEP = -0.5
        @JvmField
        var VELOCITY_ADJUSTMENT_STEP = 10.0
        @JvmField
        var SORTER_POSITION_MULTIPLIER = -0.005
        @JvmField
        var TURET_OFFSET_ADJUSTMENT_STEP = 1
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
        lastPose = null
        follower.startTeleopDrive(true)
        opModeGraph.tickFlow
            .onEach {
                handleSorter()
            }
            .launchIn(opModeScope)

        shooter.stateFlow
            .map { (_, canShoot) -> canShoot }
            .distinctUntilChanged()
            .filter { it }
            .onEach {
                gamepad2.rumble(100)
            }
            .launchIn(opModeScope)

//        intake.artefactFlow
//            .onEach {
//                telemetry.addData("ArtefactType", it)
//            }
//            .filterNotNull()
//            .onEach {
//                delay(50L)
//                sorter.intake(it)
//            }
//            .launchIn(opModeScope)

        limelight.start()
    }

    override fun loop() {
        telemetry.addData("canShoot", shooter.stateFlow.value.canShoot)
        follower.setTeleOpDrive(
            /* forward = */ -gamepad1.left_stick_y.toDouble() * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* strafe = */ deadZone(-gamepad1.left_stick_x.toDouble(), 0.15) * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* turn = */ -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ isRobotCentric
        )
        follower.update()
//        drawDebug(follower)
        when {
            gamepad1.rightBumperWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }

        // Manual intake servo control (only works if intake is not running/outtaking)
        if (!intake.isRunning && !intake.isOuttake) {
            when {
                gamepad1.squareWasPressed() -> intake.isServoRunning = true
                gamepad1.squareWasReleased() -> intake.isServoRunning = false
            }
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
        }
        telemetry.addData("AprilTag Distance", distanceToGoal)

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

//        if (gamepad2.y) shooter.velocity += VELOCITY_ADJUSTMENT_STEP
//        if (gamepad2.x) shooter.velocity -= VELOCITY_ADJUSTMENT_STEP

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
}
