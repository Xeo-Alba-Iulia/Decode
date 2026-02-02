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
    )

    private var _isRunning = false
    var isRunning
        get() = _isRunning
        set(value) {
            _isOuttake = false
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
            motor.power = if (value) -INTAKE_POWER else 0.0
            servo.power = if (value) -SERVO_POWER else 0.0
        }

    var isServoRunning = false
        set(value) {
            field = value
            servo.power = if (value) SERVO_POWER else 0.0
        }

    init {
        sensor.gain = GAIN
    }

    val stateFlow =
        flow {
            while (true) {
                with(sensor) {
                    emit(
                        State(
                            alpha(),
                            red(),
                            green(),
                            blue(),
                            getDistance(DistanceUnit.CM)
                        )
                    )
                    delay(40L)
                }
            }
        }.shareIn(opModeScope, SharingStarted.WhileSubscribed(replayExpirationMillis = 100L), replay = 2)

    val distanceFlow
        get() =
            stateFlow
                .map { it.alpha >= 75.0 }
                .distinctUntilChanged()

    @OptIn(FlowPreview::class)
    val artefactFlow
        get() =
            stateFlow
                .map { (alpha, red, green, blue) ->
                    when {
                        alpha >= 75.0 -> null
                        red > RED_THRESHOLD && blue > BLUE_THRESHOLD -> ArtefactType.PURPLE
                        red < RED_THRESHOLD && green > GREEN_THRESHOLD -> ArtefactType.GREEN
                        else -> {
                            RobotLog.dd("Intake", "Unknown artefact color: A=$alpha R=$red, G=$green, B=$blue")
                            null
                        }
                    }
                }.debounce(75L)
                .filterNotNull()
                .distinctUntilChanged()

    companion object {
        @JvmField
        var INTAKE_POWER = 0.7
        @JvmField
        var SERVO_POWER = 1.0
        @JvmField
        var RED_THRESHOLD = 45
        @JvmField
        var GREEN_THRESHOLD = 13
        @JvmField
        var BLUE_THRESHOLD = 0
        @JvmField
        var GAIN = 15f
    }
}