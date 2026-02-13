package org.firstinspires.ftc.teamcode.elevator

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@SingleIn(OpModeScope::class)
@Inject
class Elevator(
    @Named("elevator") private val motor: DcMotorEx,
    private val opModeScope: CoroutineScope,
) : OpModeObserver {
    companion object {
        @JvmField
        var HEIGHT = 1200.0
        @JvmField
        var coefficients = PIDCoefficients(kP = 0.01, kD = 0.0001)
    }

    private var liftJob: Job? = null

    val positionFlow: StateFlow<Double>
        field = MutableStateFlow(0.0)

    var height = HEIGHT
    var power by motor::power


    private val controller = controlSystem {
        posPid(coefficients)
    }

    /**
     * Lifts the elevator to the specified height.
     * Not safe to call from more than one thread at a time.
     */
    fun lift(height: Double = this.height) {
        liftJob?.cancel("New liftJob launched")
        liftJob = opModeScope.launch {
            controller.goal = KineticState(position = height)
            try {
                while (true) {
                    val position = motor.currentPosition.toDouble()
                    positionFlow.value = position
                    power = controller.calculate(
                        KineticState(
                            position = position,
                            velocity = motor.velocity
                        )
                    )
                    delay(50L)
                }
            } finally {
                power = 0.0
            }
        }
    }
}