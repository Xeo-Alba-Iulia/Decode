package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.zacsweers.metro.DependencyGraph
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.teamcode.Alliance
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.elevator.Elevator
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.opmode.auto.mirrorAlliance
import org.firstinspires.ftc.teamcode.pedropathing.drawDebug
import org.firstinspires.ftc.teamcode.pedropathing.pinpointConstants
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.shooter.ShooterConfig
import org.firstinspires.ftc.teamcode.shooter.alignToPose
import org.firstinspires.ftc.teamcode.shooter.fastShoot
import org.firstinspires.ftc.teamcode.shooter.getShooterPose
import org.firstinspires.ftc.teamcode.shooter.limelightGroundDistanceMeters
import org.firstinspires.ftc.teamcode.shooter.prepareFastShoot
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.max
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

    protected open val startPose = Pose(60.0, 8.0, PI / 2).mirrorAlliance(isMirrored)

    protected open val resetPose = Pose(15.0, 80.0, PI)
    protected open val actualGoalPose = Pose(0.0, 144.0).mirrorAlliance(isMirrored)
    protected open var goalPose = actualGoalPose
    val distanceFlow = MutableStateFlow(0.0)

    private lateinit var patternList: List<ArtefactType>
    private var startedLifting = false
    private var isOdometryDisabled = false
    private var distanceTimeMark = TimeSource.Monotonic.markNow()
    private var odometrySwitchTimeMark: TimeSource.Monotonic.ValueTimeMark? = null
    private var squareHoldTimeMark: TimeSource.Monotonic.ValueTimeMark? = null
    private var hasHandledSquareHold = false
    private var isShootingFar = false
    private var wasPrepared = false

    private val restartCameraMutex = Mutex()

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
                        intake.isOuttake = true
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
        handlePoseResetAndCameraMode()
        //handleElevator()

        val limelightResult = limelight
            .takeIf { it.isConnected }
            ?.latestResult
            ?.takeIf { it.isValid() }

        limelightResult?.let { result ->
            when {
                isOdometryDisabled -> {
                    shooter.angleDegrees -= result.tx
                    val pos = result.fiducialResults.firstOrNull()?.targetPoseCameraSpace?.position ?: return@let
                    distanceFlow.value =
                        limelightGroundDistanceMeters(pos.x, pos.z) + ShooterConfig.SHOOTER_BACK_OFFSET_INCHES / 39.37
                    distanceTimeMark = TimeSource.Monotonic.markNow() + 3.seconds
                }
                gamepad1.crossWasPressed() -> {
                    turretOffset -= result.tx
                }
            }
        }

        if (!isOdometryDisabled) {
            val shooterPose = getShooterPose(follower.pose)
            shooter.alignToPose(follower.pose, goalPose, turretOffset)
            if (distanceTimeMark.hasPassedNow()) {
                val distance = (hypot(goalPose.x - shooterPose.x, goalPose.y - shooterPose.y)) / 39.37
                distanceFlow.value = distance.takeUnless { isShootingFar } ?: max(distance, 3.0)
            }
        }

        // goalPose = if (distanceFlow.value <= 2.8) Pose(actualGoalPose.x, actualGoalPose.y - 5.0) else Pose(actualGoalPose.x + 5.0, actualGoalPose.y)

        val dist = distanceFlow.value
        val flightTime = (dist / romanianSpeedLUT(dist))
        goalPose = Pose(actualGoalPose.x - follower.velocity.xComponent * flightTime, actualGoalPose.y - follower.velocity.yComponent * flightTime)

        with(gamepad2) {
            when {
                crossWasPressed() -> odometrySwitchTimeMark = TimeSource.Monotonic.markNow() + 2.seconds
                crossWasReleased() -> odometrySwitchTimeMark = null
            }
            if (odometrySwitchTimeMark?.hasPassedNow() == true && cross) {
                odometrySwitchTimeMark = null
//                isOdometryDisabled = !isOdometryDisabled
            }

//            if (bWasPressed()) {
//                opModeScope.launch {
//                    if (restartCameraMutex.isLocked) return@launch
//                    restartCameraMutex.withLock {
//                        limelight.stop()
//                        limelight.pipelineSwitch(limelightPipeline)
//                        limelight.start()
//                        Log.d("Limelight", limelight.status.toString())
//                        rumble(0.0, 0.5, 1500)
//                    }
//                }
//            }
        }

        if (gamepad1.triangleWasPressed())
            isRobotCentric = !isRobotCentric

        if (gamepad2.triangleWasPressed()) {
            isShootingFar = !isShootingFar
            gamepad2.rumble(if (isShootingFar) 1000 else 200)
        }

        telemetry.addData("Distance", distanceFlow.value)
    }

    private fun romanianSpeedLUT(distance: Double): Double {
        return 0.19856 * distance + 126.62389
    }

    private fun handlePoseResetAndCameraMode() {
        when {
            gamepad1.squareWasPressed() -> {
                squareHoldTimeMark = TimeSource.Monotonic.markNow() + 3.seconds
                hasHandledSquareHold = false
            }
            gamepad1.squareWasReleased() -> {
                if (!hasHandledSquareHold) {
                    val pose = follower.pose
                    val closestResetPose =
                        if (pose.distanceFrom(startPose) <= pose.distanceFrom(resetPose)) startPose else resetPose
                    follower.setPose(closestResetPose)
                    gamepad1.rumble(200)
                }
                squareHoldTimeMark = null
                hasHandledSquareHold = false
            }
        }

        if (!hasHandledSquareHold && squareHoldTimeMark?.hasPassedNow() == true && gamepad1.square) {
            isOdometryDisabled = true
            isRobotCentric = true
            hasHandledSquareHold = true
            gamepad1.rumble(750)
        }
    }

    private fun handleIntake() {
        when {
            gamepad1.rightBumperWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }
    }

    @Deprecated("Parking no longer exists or robot")
    private fun handleElevator() {
        if (gamepad1.squareWasPressed())
            if (!startedLifting) {
                startedLifting = true
                isOdometryDisabled = true
                currentShooterJob?.cancel()
                intake.isRunning = false
                sorter.isLifting = false
                shooter.angleDegrees = -90.0
                elevator.goUp()
                opModeScope.launch(Dispatchers.IO) {
                    elevator.positionFlow
                        .filter { it >= 800.0 }
                        .first()
                    shooter.angleDegrees = 0.0
                }
            } else
                elevator.goPark()
    }

    private fun handleShooter() {
        val autoShoot = gamepad1.rightTriggerWasPressed()

        // Start/stop shooting sequence
        if (gamepad1.leftTriggerWasPressed() && currentShooterJob?.isCancelled != false) {
            currentShooterJob = shooter.shoot(distanceFlow)
        }

        if (autoShoot) {
            if (!sorter.isFull && !wasPrepared) {
                sorter.prepareFastShoot()
                intake.isServoRunning = true
                wasPrepared = true
            }
            else
                opModeScope.launch(Dispatchers.Unconfined) {
                    intake.isServoRunning = true
                    wasPrepared = false
                    sorter.fastShoot()
                }
        }

        if (gamepad1.leftBumperWasPressed()) {
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
