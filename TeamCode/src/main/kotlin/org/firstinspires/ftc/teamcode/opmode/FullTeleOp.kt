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
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.elevator.Elevator
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.auto.mirrorAlliance
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.prepareFastShoot
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeSource

@Config
abstract class FullTeleOp(isMirrored: Boolean, private val limelightPipeline: Int) : CoroutineOpMode() {

    constructor(alliance: Alliance, limelightPipeline: Int) : this(alliance == Alliance.RED, limelightPipeline)

    // Subsystems
    lateinit var intake: Intake
    lateinit var shooter: ShooterImpl
    lateinit var sorter: Sorter
    lateinit var follower: Follower
    lateinit var limelight: Limelight3A
    lateinit var elevator: Elevator

    // Drive state
    private var isRobotCentric = false

    // Shooter state
    private var currentShooterJob: Job? = null
    private var turretOffset = 0.0

    protected open val startPose = Pose(60.0, 7.0, PI / 2).mirrorAlliance(isMirrored)
    protected open val goalPose = Pose(5.0, 141.5 - 5.0).mirrorAlliance(isMirrored)

    val distanceFlow = MutableStateFlow(0.0)

    private lateinit var patternList: List<ArtefactType>
    private var startedLifting = false
    private var distanceTimeMark = TimeSource.Monotonic.markNow()
    private var isShootingFar = false

    companion object {
        @JvmField
        var SORTER_POSITION_MULTIPLIER = -0.02
        @JvmField
        var TURET_OFFSET_ADJUSTMENT_STEP = 1

        const val TAG = "TeleOp"
    }

    override fun init() {
        intake = opModeGraph.intake
        shooter = opModeGraph.shooter
        sorter = opModeGraph.sorter
        follower = opModeGraph.follower
        limelight = opModeGraph.limelight
        elevator = opModeGraph.elevator
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
        sorter.prepareIntake()
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
                        delay(0.5.seconds)
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

    override fun loop() {
        follower.setTeleOpDrive(
            /* forward = */ -gamepad1.left_stick_y.toDouble() * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* strafe = */ -gamepad1.left_stick_x.toDouble() * if (!isRobotCentric && this is BlueTeleOp) -1 else 1,
            /* turn = */ -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ isRobotCentric
        )
        follower.update()
        drawDebug(follower)

        handleIntake()
        handleSorter()
        handleShooter()
        handleElevator()

        limelight.latestResult.takeIf { it.isValid() && gamepad1.crossWasPressed() }?.let { result ->
            turretOffset -= result.tx
            val pos = result.fiducialResults.single().targetPoseCameraSpace.position
            distanceFlow.value = sqrt(pos.z * pos.z + pos.x * pos.x) + 0.15
            distanceTimeMark = TimeSource.Monotonic.markNow() + 3.seconds
        }

        if (distanceTimeMark.hasPassedNow()) {
            val distance = (hypot(12.0 - follower.pose.x, (141.5 - 12.0) - follower.pose.y)) / 39.37
            distanceFlow.value = distance.takeUnless { isShootingFar } ?: min(distance, 3.0)
            if (!startedLifting)
                shooter.alignToPose(follower.pose, goalPose, turretOffset)
        }

        if (gamepad1.triangleWasPressed())
            isRobotCentric = !isRobotCentric

        telemetry.addData("Distance", distanceFlow.value)
    }

    private fun handleIntake() {
        when {
            gamepad1.rightBumperWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }
    }

    private fun handleElevator() {
        if (gamepad1.squareWasPressed())
            if (!startedLifting) {
                startedLifting = true
                currentShooterJob?.cancel()
                intake.isRunning = false
                sorter.isLifting = false
                shooter.angleDegrees = -90.0
                elevator.goUp()
                opModeScope.launch(Dispatchers.IO) {
                    elevator.positionFlow
                        .filter { it >= 500.0 }
                        .first()
                    shooter.angleDegrees = 0.0
                }
            } else
                elevator.goPark()
    }

    private fun handleShooter() {
        val autoShoot = gamepad2.triangleWasPressed() || gamepad1.rightTriggerWasPressed()

        // Start/stop shooting sequence
        if ((gamepad2.aWasPressed() || gamepad1.leftTriggerWasPressed()) && currentShooterJob?.isCancelled != false) {
            currentShooterJob = shooter.shoot(distanceFlow)
        }

        if (autoShoot) {
            if (!sorter.isFull)
                sorter.prepareFastShoot()
            else
                opModeScope.launch(Dispatchers.Unconfined) {
                    intake.isServoRunning = true
                    sorter.fastShoot()
                }
        }

        if (gamepad2.bWasPressed() || gamepad1.leftBumperWasPressed()) {
            currentShooterJob?.cancel()
        }

        // Adjust shooter angle
        when {
            gamepad2.dpad_right -> turretOffset -= TURET_OFFSET_ADJUSTMENT_STEP
            gamepad2.dpad_left -> turretOffset += TURET_OFFSET_ADJUSTMENT_STEP
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
        else if (
            gamepad2.rightStickButtonWasPressed() ||
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
