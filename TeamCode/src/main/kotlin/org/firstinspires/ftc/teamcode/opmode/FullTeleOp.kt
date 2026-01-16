package org.firstinspires.ftc.teamcode.opmode

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Drive
import org.firstinspires.ftc.teamcode.SensorOdometry
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.shootCount
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
@TeleOp(group = "Meet")
class FullTeleOp : CoroutineOpMode() {
    // Subsystems
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var drive: Drive
    lateinit var odometry: SensorOdometry
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var sorter: Sorter
    lateinit var follower: Follower

    // Drive state
    private var isRobotCentric = true
    private var wasYPressed = false

    // Shooter state
    private var currentShooterJob: Job? = null
    private var currentShooterFlow: Flow<Shooter.State>? = null
    private var lastShooterState: Shooter.State? = null

    private var speedMultiplier = NORMAL_MODE_MULTIPLIER

    private var isDrawingMutex = Mutex()

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
        follower = opModeGraph.follower

        // Initialize drive and odometry directly (not in DI graph)
//        drive = Drive(hardwareMap)
//        odometry = SensorOdometry(hardwareMap)

        runBlocking { sorter.prepareIntake() }
    }

    override fun start() {
        super.start()
        follower.startTeleopDrive(true)
        opModeGraph.tickFlow
            .onEach {
                handleSorter()
            }
            .launchIn(opModeScope)

        shooter.stateFlow
            .onEach {
                RobotLog.dd("FullTeleOp", it.toString())
                telemetry.addData("Shooter", it)
                telemetry.update()
            }
            .launchIn(opModeScope)
    }

    override fun loop() {
        follower.setTeleOpDrive(
            /* forward = */ -gamepad1.left_stick_y.toDouble() * speedMultiplier,
            /* strafe = */ gamepad1.left_stick_x.toDouble() * speedMultiplier,
            /* turn = */ -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ isRobotCentric
        )
        follower.update()
        if (!isDrawingMutex.isLocked) {
            opModeScope.launch(Dispatchers.IO) {
                isDrawingMutex.withLock {
                    drawDebug(follower)
                }
            }
        }

        // Update odometry
//        odometry.update()

        // Handle driving
//        handleDrive()

        when {
            gamepad1.crossWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }

        if (gamepad1.yWasPressed())
            isRobotCentric = !isRobotCentric

        // Handle shooter controls (Gamepad 2)
        handleShooter()

        // Handle sorter controls (Gamepad 2)
//        handleSorter()

        // Update telemetry
//        updateTelemetry()
    }

    private fun handleDrive() {
        // Get raw input
        var axial = -gamepad1.left_stick_y.toDouble()
        var lateral = -gamepad1.left_stick_x.toDouble()
        val yaw = gamepad1.right_stick_x.toDouble()

        // Toggle field-centric mode with Y button
        // Apply field-centric transformation
        if (!isRobotCentric) {
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

    private fun handleShooter() {
        // Start/stop shooting sequence
        if (gamepad2.aWasPressed() && currentShooterJob == null) {
            currentShooterJob = shooter.shoot()
        }

        if (currentShooterJob != null) {
            if (gamepad1.rightBumperWasPressed())
                opModeScope.launch {
                    shootCount(shooter.stateFlow, sorter)
                }

            if (gamepad2.bWasPressed()) {
                currentShooterJob?.cancel()
                currentShooterJob = null
            }
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
                gamepad2.leftStickButtonWasReleased() -> sorter.intake(ArtefactType.PURPLE)
                gamepad2.rightStickButtonWasPressed() -> sorter.intake(ArtefactType.GREEN)
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

            // Using left trigger as boolean for lifting
            sorter.isLifting = gamepad2.left_trigger > 0.5

//            // Adjust sorter position with triggers
//            sorter.position += (gamepad2.right_trigger - gamepad2.left_trigger).toDouble() * SORTER_POSITION_MULTIPLIER
        }
    }

    private fun updateTelemetry() {
        telemetry.addData("Power", Intake.INTAKE_POWER)

        telemetry.addData("Target Velocity", shooter.velocity)
        telemetry.addData("Hood Position", "%.3f".format(shooter.hood))
        telemetry.addData("Angle", "%.3f°".format(shooter.angleDegrees))

        telemetry.addData("Size", "${sorter.size}/3")
        telemetry.addData("Is Lifting", sorter.isLifting)
        telemetry.addData("Position", "%.4f".format(sorter.position))
        telemetry.addData("Empty", sorter.isEmpty)
        telemetry.addData("Full", sorter.isFull)

        telemetry.addLine()
        telemetry.addData("Runtime", "%.2f s".format(runtime))
        telemetry.update()
    }
}
