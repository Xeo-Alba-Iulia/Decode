package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Drive
import org.firstinspires.ftc.teamcode.SensorOdometry
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.sorter.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.cos
import kotlin.math.sin

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
@TeleOp(name = "Full TeleOp", group = "Competition")
class FullTeleOp : CoroutineOpMode() {

    // Subsystems
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var drive: Drive
    lateinit var odometry: SensorOdometry
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var sorter: Sorter

    // Drive state
    private var isFieldCentric = true
    private var wasYPressed = false

    // Shooter state
    private var currentShooterJob: Job? = null
    private var currentShooterFlow: Flow<Shooter.State>? = null
    private var lastShooterState: Shooter.State? = null

    // Speed control
    companion object {
        const val SLOW_MODE_MULTIPLIER = 0.3
        const val NORMAL_MODE_MULTIPLIER = 1.0
        const val HOOD_ADJUSTMENT_STEP = 0.01
        const val ANGLE_ADJUSTMENT_STEP = 0.01
        const val VELOCITY_ADJUSTMENT_STEP = 10.0
        const val SORTER_POSITION_MULTIPLIER = 0.001
    }

    override fun init() {
        telemetry = opModeGraph.telemetry
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter
        sorter = opModeGraph.sorter

        // Initialize drive and odometry directly (not in DI graph)
        drive = Drive(hardwareMap)
        odometry = SensorOdometry(hardwareMap)

        // Register sorter as observer for lifecycle management
        observers += sorter

        // Initialize shooter hood to center position
        shooter.hood = 0.5

        telemetry.addLine("Full TeleOp Initialized")
        telemetry.addLine("Press START to begin")
        telemetry.update()
    }

    override fun start() {
        super.start()
        // Prepare sorter for intake on start
        opModeScope.launch {
            sorter.prepareIntake()
        }
    }

    override fun loop() {
        // Update odometry
        odometry.update()

        // Handle driving
        handleDrive()

        // Handle intake controls (Gamepad 1)
        handleIntake()

        // Handle shooter controls (Gamepad 2)
        handleShooter()

        // Handle sorter controls (Gamepad 2)
        handleSorter()

        // Update telemetry
        updateTelemetry()
    }

    private fun handleDrive() {
        // Get raw input
        var axial = -gamepad1.left_stick_y.toDouble()
        var lateral = gamepad1.left_stick_x.toDouble()
        val yaw = gamepad1.right_stick_x.toDouble()

        // Toggle field-centric mode with Y button
        if (gamepad1.y && !wasYPressed) {
            isFieldCentric = !isFieldCentric
        }
        wasYPressed = gamepad1.y

        // Apply field-centric transformation
        if (isFieldCentric) {
            val heading = odometry.pose.getHeading(AngleUnit.RADIANS)
            val temp = axial * cos(heading) - lateral * sin(heading)
            lateral = axial * sin(heading) + lateral * cos(heading)
            axial = temp
        }

        // Apply speed multiplier (slow mode when left bumper held)
        val speedMultiplier = if (gamepad1.left_bumper) SLOW_MODE_MULTIPLIER else NORMAL_MODE_MULTIPLIER
        axial *= speedMultiplier
        lateral *= speedMultiplier
        val adjustedYaw = yaw * speedMultiplier

        drive.drive(axial, lateral, adjustedYaw)
    }

    private fun handleIntake() {
        when {
            gamepad1.aWasPressed() -> intake.isRunning = true
            gamepad1.bWasPressed() -> intake.isRunning = false
        }
    }

    private fun handleShooter() {
        // Start/stop shooting sequence
        if (gamepad2.aWasPressed()) {
            if (currentShooterFlow == null) {
                currentShooterFlow = shooter.shoot()
            }
            currentShooterJob = opModeScope.launch {
                currentShooterFlow!!.collect { state ->
                    lastShooterState = state
                }
            }
        }

        if (gamepad2.bWasPressed()) {
            currentShooterJob?.cancel()
            currentShooterJob = null
            currentShooterFlow = null
            lastShooterState = null
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

    private fun handleSorter() {
        runBlocking {
            // Intake artefacts (using stick buttons to avoid conflicts)
            when {
                gamepad2.left_stick_button -> sorter.intake(ArtefactType.PURPLE)
                gamepad2.right_stick_button -> sorter.intake(ArtefactType.GREEN)
            }

            // Prepare shoot
            when {
                gamepad2.leftBumperWasPressed() -> sorter.prepareShoot(ArtefactType.PURPLE)
                gamepad2.rightBumperWasPressed() -> sorter.prepareShoot(ArtefactType.GREEN)
                gamepad2.backWasPressed() -> sorter.prepareShoot()
            }

            // Prepare intake
            if (gamepad2.startWasPressed()) {
                sorter.prepareIntake()
            }

            // Lifting control (hold A on gamepad2 - but A is used for shooter, use different button)
            // Using left trigger as boolean for lifting
            sorter.isLifting = gamepad2.left_trigger > 0.5

            // Adjust sorter position with triggers
            sorter.position += (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() * SORTER_POSITION_MULTIPLIER
        }
    }

    private fun updateTelemetry() {
        telemetry.addData("Mode", if (isFieldCentric) "Field-Centric" else "Robot-Centric")
        telemetry.addData("Slow Mode", gamepad1.left_bumper)
        telemetry.addData("Pose", odometry.pose)
        telemetry.addData("Heading", "%.2f°".format(odometry.heading))

        telemetry.addData("Running", intake.isRunning)
        telemetry.addData("Power", Intake.INTAKE_POWER)

        telemetry.addData("Target Velocity", shooter.velocity)
        telemetry.addData("Hood Position", "%.3f".format(shooter.hood))
        telemetry.addData("Angle", "%.3f°".format(shooter.angleDegrees))
        lastShooterState?.let { state ->
            telemetry.addData("Current Velocity", "%.1f".format(state.velocity))
            telemetry.addData("Can Shoot", state.canShoot)
        } ?: telemetry.addData("Status", "Idle")

        telemetry.addData("Size", "${sorter.size}/3")
        telemetry.addData("Is Lifting", sorter.isLifting)
        telemetry.addData("Position", "%.4f".format(sorter.position))
        telemetry.addData("Empty", sorter.isEmpty)
        telemetry.addData("Full", sorter.isFull)

        telemetry.addLine()
        telemetry.addData("Runtime", "%.2f s".format(runtime))
        telemetry.update()
    }

    override fun stop() {
        // Cancel any ongoing shooter job
        currentShooterJob?.cancel()
        currentShooterJob = null
        currentShooterFlow = null

        // Stop intake
        intake.isRunning = false

        // Stop drive
        drive.drive(0.0, 0.0, 0.0)

        super.stop()
    }
}

