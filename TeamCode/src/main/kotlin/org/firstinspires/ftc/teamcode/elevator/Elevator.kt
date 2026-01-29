package org.firstinspires.ftc.teamcode.elevator

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.GravityFeedforwardParameters
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@SingleIn(OpModeScope::class)
@Inject
class Elevator(
    @Named("elevator") private val motor: DcMotorEx,
    private val opModeScope: CoroutineScope,
    private val tickFlow: SharedFlow<Unit>,
) : OpModeObserver {
    companion object {
        @JvmField
        var HEIGHT = 150.0
        @JvmField
        var POWER = 0.8
        @JvmField
        var coefficients = PIDCoefficients(0.0)
        @JvmField
        var feedforward = GravityFeedforwardParameters()
    }

    var height = HEIGHT
    var power
        get() = motor.power
        set(value) {
            motor.power = value.coerceIn(-POWER, POWER)
        }

    private val controller = controlSystem {
        posPid(coefficients)
        armFF(feedforward)
    }

    fun lift(height: Double = this.height): Job {
        controller.goal = KineticState(position = height)
        return opModeScope.launch {
            try {
                tickFlow.collect {
                    power = controller.calculate(
                        KineticState(
                            position = motor.currentPosition.toDouble(),
                            velocity = motor.velocity
                        )
                    )
                }
            } finally {
                power = 0.0
            }
        }
    }
}