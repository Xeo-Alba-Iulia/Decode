package org.firstinspires.ftc.teamcode.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.FlowPreview
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ArtefactType

@Config
@Inject
class Intake(
    @Named("intake") private val motor: DcMotor,
    @Named("intake") private val servo: CRServo,
    private val sensor: RevColorSensorV3,
    opModeScope: CoroutineScope,
) {
    data class State(
        val alpha: Int,
        val red: Int,
        val green: Int,
        val blue: Int,
        val distanceCm: Double,
    ) {
        companion object {
            val ZERO = State(0, 0, 0, 0, 0.0)
        }
    }

    var isDebug = false

    private var _isRunning = false
    var isRunning
        get() = _isRunning
        set(value) {
            _isOuttake = false
            _isServoRunning = false
            _isRunning = value
            motor.power = if (value) INTAKE_POWER else 0.0
            servo.power = if (value) SERVO_POWER else 0.0
        }

    private var _isOuttake = false
    var isOuttake
        get() = _isOuttake
        set(value) {
            _isOuttake = value
            _isRunning = false
            _isServoRunning = false
            motor.power = if (value) -INTAKE_POWER else 0.0
            servo.power = if (value) -SERVO_POWER else 0.0
        }

    private var _isServoRunning = false
    var isServoRunning
        get() = _isServoRunning
        set(value) {
            _isServoRunning = value
            _isRunning = false
            _isOuttake = false
            motor.power = 0.0
            servo.power = if (value) SERVO_POWER else 0.0
        }

    init {
        sensor.gain = GAIN
    }

    val stateFlow =
        flow {
            while (true) {
                if (isRunning || isDebug)
                    emit(with(sensor) {
                        State(
                            alpha(),
                            red(),
                            green(),
                            blue(),
                            getDistance(DistanceUnit.CM)
                        )
                    })
                delay(10L)
            }
        }.stateIn(opModeScope, SharingStarted.WhileSubscribed(replayExpirationMillis = 100L), State.ZERO)

    @OptIn(FlowPreview::class)
    val distanceFlow
        get() =
            stateFlow
                .map { (alpha) -> alpha >= ALPHA_THRESHOLD }
                .distinctUntilChanged()
                .debounce(40L)

    @OptIn(FlowPreview::class)
    val artefactFlow
        get() =
            stateFlow
                .map { (alpha, red, green, blue) ->
                    when {
                        alpha < ALPHA_THRESHOLD -> null
                        red > 150.0 && blue >= 300.0 -> ArtefactType.PURPLE
                        red <= 150.0 && green >= 100.0 -> ArtefactType.GREEN
                        else -> {
                            RobotLog.dd("Intake", "Unknown artefact color: A=$alpha R=$red, G=$green, B=$blue")
                            null
                        }
                    }
                }.distinctUntilChanged()
                .debounce(40L)
                .filterNotNull()

    companion object {
        @JvmField
        var INTAKE_POWER = 0.7
        @JvmField
        var SERVO_POWER = 1.0
        @JvmField
        var ALPHA_THRESHOLD = 50.0
        @JvmField
        var GAIN = 15f
    }
}