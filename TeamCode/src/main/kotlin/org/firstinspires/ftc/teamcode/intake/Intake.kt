package org.firstinspires.ftc.teamcode.intake

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.FlowPreview
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.debounce
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOn
import org.firstinspires.ftc.teamcode.ArtefactType

@Config
@Inject
class Intake(
    @Named("intakeMotor") private val motor: DcMotor,
    private val sensor: ColorSensor
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

    @OptIn(FlowPreview::class)
    val artefactFlow: Flow<ArtefactType?> =
        flow {
            while (_isRunning) {
                val colors = sensor.argb()
                val green = Color.green(colors)
                val blue = Color.blue(colors)

                when {
                    green > 500 -> emit(ArtefactType.GREEN)
                    blue > 500 -> emit(ArtefactType.PURPLE)
                    else -> emit(null)
                }
                delay(20L)
            }
        }.flowOn(Dispatchers.IO).debounce(50L).distinctUntilChanged()

    companion object {
        @JvmField
        var INTAKE_POWER = 0.7
    }
}