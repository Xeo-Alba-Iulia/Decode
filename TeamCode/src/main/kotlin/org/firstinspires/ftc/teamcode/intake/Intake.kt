package org.firstinspires.ftc.teamcode.intake

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.ArtefactType

@Config
@Inject
class Intake(
    @Named("intakeMotor") private val motor: DcMotor,
    private val sensor: RevColorSensorV3,
    private val opModeScope: CoroutineScope,
) {
    private var _isRunning = false
    var isRunning
        get() = _isRunning
        set(value) {
            _isOuttake = false
            _isRunning = value
            motor.power = if (value) INTAKE_POWER else 0.0
        }

    private var _isOuttake = false
    var isOuttake
        get() = _isOuttake
        set(value) {
            _isOuttake = value
            _isRunning = false
            motor.power = if (value) -INTAKE_POWER else 0.0
        }

    init {
        sensor.gain = GAIN
    }

    val colorFlow = runBlocking {
        flow {
            while (true) {
                emit(sensor.normalizedColors)
                delay(20L)
            }
        }.stateIn(opModeScope)
    }

    val artefactFlow
        get() = colorFlow.map {
            when {
                it.alpha >= 0.6 && it.red >= 0.33 -> ArtefactType.PURPLE
                it.alpha >= 0.6 && it.red in 0.12..0.25 -> ArtefactType.GREEN
                else -> null
            }
        }.distinctUntilChanged()

    companion object {
        @JvmField
        var INTAKE_POWER = 0.7
        @JvmField
        var RED_THRESHOLD = 0.44f
        @JvmField
        var GREEN_THRESHOLD = 0.5f
        @JvmField
        var BLUE_THRESHOLD = 0.45f
        @JvmField
        var GAIN = 3.4f
    }
}