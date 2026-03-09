package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.prepareFastShoot
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.concurrent.atomics.AtomicBoolean
import kotlin.concurrent.atomics.ExperimentalAtomicApi
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Config
abstract class FullTeleOp : CoroutineOpMode() {
    // Subsystems
    lateinit var intake: Intake
    lateinit var shooter: ShooterImpl
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

    val distanceFlow = MutableStateFlow(0.0)

    @OptIn(ExperimentalAtomicApi::class)
    val updatedByCam = AtomicBoolean(false)

    lateinit var patternList: List<ArtefactType>

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

        patternList = pattern ?: emptyList()
    }

    override fun start() {
        super.start()
        follower.setStartingPose(lastPose ?: startPose)
        lastPose = null
        patternList = pattern ?: emptyList()
        pattern = null
        follower.startTeleopDrive(true)
        shooter.stateFlow
            .map { (_, canShoot) -> canShoot }
            .distinctUntilChanged()
            .filter { it }
            .onEach { gamepad1.rumble(100); gamepad2.rumble(100) }
            .launchIn(opModeScope + Dispatchers.IO)

//        intake.artefactFlow
//            .onEach { Log.d("Intake", "Detected artefact $it") }
//            .onEach { sorter.intake(it) }
//            .onEach { delay(250.milliseconds) }
//            .launchIn(opModeScope + Dispatchers.IO)

        intake.distanceFlow
            .filter { it }
            .onEach { Log.d("Intake", "Detected artefact") }
            .onEach { sorter.intake(PURPLE) }
            .onEach { delay(250.milliseconds) }
            .launchIn(opModeScope + Dispatchers.IO)

        intake.stateFlow
            .onEach {
                val packet = TelemetryPacket().apply { put("Intake State", it) }
                FtcDashboard.getInstance().sendTelemetryPacket(packet)
            }
            .launchIn(opModeScope + Dispatchers.IO)

        opModeScope.launch(Dispatchers.IO) {
            var lastIsFull = false
            var lastIsEmpty = true
            while (true) {
                val isFull = sorter.isFull
                val isEmpty = sorter.isEmpty
                when {
                    isFull && !lastIsFull -> {
                        gamepad1.rumble(500)
                        gamepad2.rumble(500)
                        if (currentShooterJob?.isCancelled != false)
                            currentShooterJob = shooter.shoot(distanceFlow)
                        intake.isOuttake = true
                        delay(1.seconds)
                        intake.isServoRunning = true
                        sorter.prepareFastShoot()
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

    @OptIn(ExperimentalAtomicApi::class)
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
        if (!updatedByCam.load())
            distanceFlow.value = (hypot(12.0 - follower.pose.x, (141.5 - 12.0) - follower.pose.y)) / 39.37
        telemetry.addData("Distance", distanceFlow.value)
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

        val result = limelight.latestResult
        if (result.isValid() && gamepad1.crossWasPressed()) {
            turretOffset -= result.tx
            if (updatedByCam.compareAndSet(expectedValue = false, newValue = true)) {
                opModeScope.launch(Dispatchers.IO) {
                    val pos = result.fiducialResults.single().targetPoseCameraSpace.position
                    distanceFlow.value = sqrt(pos.z * pos.z + pos.x * pos.x) + 0.15
                    delay(3.seconds)
                    val _ = updatedByCam.exchange(false)
                }
            }
        }

        if (gamepad1.triangleWasPressed())
            isRobotCentric = !isRobotCentric
    }

    var velocity = 1800.0
    var hood = 0.5

    private fun handleShooter() {
        val autoShoot = gamepad2.triangleWasPressed() || gamepad1.rightTriggerWasPressed()

        velocity += gamepad2.right_stick_y.toDouble()
        hood += gamepad2.left_stick_y.toDouble() * (-0.001)

        telemetry.addData("Velocity", velocity)
        telemetry.addData("Hood", hood)

        // Start/stop shooting sequence
        if ((gamepad2.aWasPressed() || gamepad1.leftTriggerWasPressed()) && currentShooterJob?.isCancelled != false) {
            currentShooterJob = shooter.shoot(::velocity, ::hood)
        }

        if (autoShoot) {
            opModeScope.launch(Dispatchers.Unconfined) {
                intake.isServoRunning = true
                sorter.fastShoot()
//                currentShooterJob?.cancel()
            }
        }

        if (gamepad2.bWasPressed() || gamepad1.leftBumperWasPressed()) {
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
