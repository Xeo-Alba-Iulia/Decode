package org.firstinspires.ftc.teamcode.intake

import android.graphics.Color
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
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
    private val sensorList: List<ColorRangeSensor>,
    opModeScope: CoroutineScope,
) {
    data class State(
        val hue: Float,
        val saturation: Float,
        val value: Float,
        val distanceCm: Double,
    ) {
        companion object {
            val ZERO = State(0f, 0f, 0f, 25.0)
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
        sensorList.forEach { it.gain = GAIN }
    }

    val stateFlow =
        flow {
            val hsvArr = FloatArray(3)
            with(sensorList) {
                while (true) {
                    if (isRunning || isDebug) {
                        val red = sensorList.maxOf { it.red() }
                        val green = sensorList.maxOf { it.green() }
                        val blue = sensorList.maxOf { it.blue() }
                        Color.RGBToHSV(red, green, blue, hsvArr)
                        val (hue, saturation, value) = hsvArr
                        emit(
                            State(
                                hue,
                                saturation,
                                value,
                                minOf { sensor ->
                                    sensor.getDistance(DistanceUnit.CM).takeUnless { it.isNaN() } ?: 25.0
                                }
                            )
                        )
                    }
                    delay(50L)
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
            stateFlow.map { (hue, sat, value, dist) ->
                when {
                    dist >= MAX_DISTANCE -> null
                    hue in 200f..320f && sat <= 0.65 -> ArtefactType.PURPLE
                    hue in 100f..160f && sat >= 0.5 -> ArtefactType.GREEN
                    else -> {
                        Log.e("Intake", "Problem reading: hue $hue, sat: $sat, value: $value")
                        null
                    }
                }?.also {
                    Log.v("Intake", "Found $it, hue: $hue, sat: $sat, value: $value")
                }
            }.distinctUntilChanged()
                .debounce(110L)
                .filterNotNull()

    companion object {
        @JvmField
        var MAX_DISTANCE = 4.0
        @JvmField
        var INTAKE_POWER = 0.7
        @JvmField
        var SLOW_POWER = 0.3
        @JvmField
        var ALPHA_THRESHOLD = 350.0
        @JvmField
        var ALPHA = 0.7
        @JvmField
        var GAIN = 1f
    }
}