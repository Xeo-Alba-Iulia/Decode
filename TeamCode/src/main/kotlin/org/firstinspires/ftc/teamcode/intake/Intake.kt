package org.firstinspires.ftc.teamcode.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.FlowPreview
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ArtefactType

@Config
@Inject
class Intake(
    @Named("intake") private val motor: DcMotorEx,
    private val sensor: ColorRangeSensor,
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
            val ZERO = State(0, 0, 0, 0, 17.0)
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
        }

    private var _isOuttake = false
    var isOuttake
        get() = _isOuttake
        set(value) {
            _isOuttake = value
            _isRunning = false
            _isServoRunning = false
            motor.power = if (value) -INTAKE_POWER else 0.0
        }

    private var _isServoRunning = false
    var isServoRunning
        get() = _isServoRunning
        set(value) {
            _isServoRunning = value
            _isRunning = false
            _isOuttake = false
            motor.power = if (value) SLOW_POWER else 0.0
        }

    val current get() = motor.getCurrent(CurrentUnit.AMPS)

    init {
        sensor.gain = GAIN
    }

    val stateFlow =
        flow {
            with(sensor) {
                while (true) {
                    if (isRunning || isDebug)
                        emit(State(alpha(), red(), green(), blue(), getDistance(DistanceUnit.CM)))
                    delay(10L)
                }
            }
        }.stateIn(opModeScope, SharingStarted.WhileSubscribed(replayExpirationMillis = 100L), State.ZERO)

    val distanceFlow
        get() =
            stateFlow
                .map { it.distanceCm <= MAX_DISTANCE }
                .distinctUntilChanged()

    @OptIn(FlowPreview::class)
    val artefactFlow
        get() =
            stateFlow
                .map { (alpha, red, green, blue, dist) ->
                    when {
                        dist > MAX_DISTANCE -> null
                        alpha > ALPHA_THRESHOLD && green > blue && blue > red -> ArtefactType.GREEN
                        alpha > ALPHA_THRESHOLD && blue > red && red > green -> ArtefactType.PURPLE
                        else -> {
                            RobotLog.dd("Intake", "Unknown artefact color: A=$alpha R=$red, G=$green, B=$blue")
                            null
                        }
                    }
                }.distinctUntilChanged()
                .debounce(30L)
                .filterNotNull()

    companion object {
        @JvmField
        var MAX_DISTANCE = 8.0
        @JvmField
        var INTAKE_POWER = 0.7
        @JvmField
        var SLOW_POWER = 0.3
        @JvmField
        var ALPHA_THRESHOLD = 350.0
        @JvmField
        var GAIN = 2f
    }
}