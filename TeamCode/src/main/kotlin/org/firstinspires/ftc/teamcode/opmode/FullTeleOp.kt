package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

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
 * - X: Decrease shooter velocityOffset
 * - Y: Increase shooter velocityOffset
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

    var heightIterator: Iterator<Double> = HEIGHT_LIST.iterator()

    val distanceFlow = MutableStateFlow(0.0)

    // Speed control
    companion object {
        @JvmField
        var ANGLE_ADJUSTMENT_STEP = -0.5
        @JvmField
        var SORTER_POSITION_MULTIPLIER = -0.02
        @JvmField
        var TURET_OFFSET_ADJUSTMENT_STEP = 1

        @JvmField
        var HEIGHT_LIST = mutableListOf(1200.0, 800.0, 0.0)

        const val TAG = "TeleOp"
    }

    override fun init() {
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
        shooter.stateFlow
            .map { (_, canShoot) -> canShoot }
            .distinctUntilChanged()
            .filter { it }
            .onEach { gamepad2.rumble(100) }
            .launchIn(opModeScope + Dispatchers.IO)

        intake.artefactFlow
            .onEach { Log.d("Intake", "Detected artefact: $it") }
            .onEach { sorter.intake(it) }
            .onEach { delay(150.milliseconds) }
            .launchIn(opModeScope + Dispatchers.IO)

        intake.stateFlow
            .onEach {
                val packet = TelemetryPacket().apply { put("Intake State", it) }
                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }
            .launchIn(opModeScope + Dispatchers.IO)

        opModeScope.launch {
            var lastIsFull = false
            var lastIsEmpty = true
            while (true) {
                val isFull = sorter.isFull
                val isEmpty = sorter.isEmpty
                when {
                    isFull && !lastIsFull -> {
                        intake.isServoRunning = true
                        gamepad1.rumble(500)
                        gamepad2.rumble(500)
                    }
                    isEmpty && !lastIsEmpty -> intake.isRunning = true
                }
                lastIsFull = isFull
                lastIsEmpty = isEmpty
                delay(150L)
            }
        }

        limelight.start()
    }

    override fun loop() {
        handleSorter()
        follower.setTeleOpDrive(
            /* forward = */ -gamepad1.left_stick_y.toDouble() * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* strafe = */ -gamepad1.left_stick_x.toDouble() * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* turn = */ -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ isRobotCentric
        )
        follower.update()
        drawDebug(follower)
        distanceFlow.value = goalPose.distanceFrom(follower.pose)
        when {
            gamepad1.rightBumperWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }

        handleShooter()

        if (gamepad2.dpad_right) {
            turretOffset -= TURET_OFFSET_ADJUSTMENT_STEP
        }
        if (gamepad2.dpad_left) {
            turretOffset += TURET_OFFSET_ADJUSTMENT_STEP
        }

        shooter.alignToPose(follower.pose, goalPose, turretOffset)

        limelight.latestResult.fiducialResults.singleOrNull()?.let {
            if (gamepad1.crossWasPressed())
                turretOffset -= it.targetXDegrees
        }

        if (gamepad1.triangleWasPressed())
            isRobotCentric = !isRobotCentric
    }

    private fun handleShooter() {
        val autoShoot = gamepad2.triangleWasPressed()

        // Start/stop shooting sequence
        if ((gamepad2.aWasPressed()) && currentShooterJob?.isCancelled != false) {
            currentShooterJob = shooter.shoot(
                distanceFlow.map { it / 39.37 }.distinctUntilChanged { vel1, vel2 -> abs(vel1 - vel2) < 0.1 }
            )
            sorter.position = 0.0
        }

        if (autoShoot) {
            if (sorter.position != 0.0)
                RobotLog.ww(TAG, "Auto shoot triggered while in position: ${sorter.position}")
            sorter.position = 1.0
            sorter.isLifting = true
            opModeScope.launch {
                delay(1.25.seconds)
                sorter.isLifting = false
                sorter.artefacts.indices.forEach { sorter.artefacts[it] = null }
                sorter.prepareIntake()
            }
        }

        if (gamepad2.bWasPressed()) {
            currentShooterJob?.cancel()
        }

        // Adjust shooter angle
        when {
            gamepad2.dpad_right -> shooter.angleDegrees += ANGLE_ADJUSTMENT_STEP
            gamepad2.dpad_left -> shooter.angleDegrees -= ANGLE_ADJUSTMENT_STEP
        }
    }

    private fun handleSorter() {
        // Intake artefacts (using stick buttons to avoid conflicts)
        when {
            gamepad2.rightBumperWasPressed() -> sorter.intake(ArtefactType.PURPLE)
            gamepad2.leftBumperWasPressed() -> sorter.intake(ArtefactType.GREEN)
        }

        if (!sorter.isEmpty)
        // Prepare shoot
            when {
                gamepad2.rightStickButtonWasPressed() -> sorter.prepareShoot(ArtefactType.PURPLE)
                gamepad2.leftStickButtonWasPressed() -> sorter.prepareShoot(ArtefactType.GREEN)
                gamepad2.backWasPressed() -> sorter.prepareShoot()
            }
        else if (gamepad2.rightStickButtonWasPressed() ||
            gamepad2.leftStickButtonWasPressed() ||
            gamepad2.backWasPressed()
        ) {
            sorter.prepareIntake()
            sorter.isLifting = false
        }

        // Prepare intake
        if (gamepad2.startWasPressed())
            sorter.prepareIntake()

        if (gamepad2.squareWasPressed())
            sorter.isLifting = !sorter.isLifting

        // Adjust sorter position with triggers
        sorter.position += (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() * SORTER_POSITION_MULTIPLIER
    }
}
