package org.firstinspires.ftc.teamcode.elevator

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.GravityFeedforwardParameters
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch

//@Config
//@SingleIn(OpModeScope::class)
//@Inject
class Elevator(
    private val motors: List<DcMotorEx>,
    private val opModeScope: CoroutineScope,
    private val tickFlow: StateFlow<Unit>,
) {
    companion object {
        @JvmField
        var HEIGHT = 2000.0
        @JvmField
        var POWER = 0.8
        @JvmField
        var coefficients = PIDCoefficients(0.0)
        @JvmField
        var feedforward = GravityFeedforwardParameters()
    }

    var height = HEIGHT
    private var power = 0.0
        set(value) {
            field = value.coerceIn(0.0..POWER)
            motors.forEach { it.power = field }
        }

    private val controller = controlSystem {
        posPid(coefficients)
        elevatorFF(feedforward)
    }

    fun lift(height: Double = this.height): Job {
        controller.goal = KineticState(position = height)
        return opModeScope.launch {
            try {
                tickFlow.collect {
                    power = controller.calculate(
                        KineticState(
                            position = motors.first().currentPosition.toDouble(),
                            velocity = motors.first().velocity
                        )
                    )
                }
            } finally {
                power = 0.0
            }
        }
    }
}