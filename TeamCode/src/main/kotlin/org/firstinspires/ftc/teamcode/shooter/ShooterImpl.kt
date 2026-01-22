package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    @Named("shooterEncoder") private val encoder: DcMotorEx,
    private val opModeScope: CoroutineScope,
    private val tickFlow: SharedFlow<Unit>,
) : Shooter {

    companion object {
        @JvmField
        var MIN_LAUNCH_VELOCITY = 2200.0

        @JvmField
        var coefficients = PIDCoefficients(0.015, kD = 0.0004)

        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.06, kV = 0.000005)
    }


    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0..80.0)
            rotationServo.position = 0.5 - field / 160.0
            RobotLog.dd("Shooter angle degrees", field.toString())
        }

    override var hood by hoodServo::position
    override var velocity = MIN_LAUNCH_VELOCITY

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    override fun shoot() =
        opModeScope.launch {
            motor.power = 1.0
            try {
                tickFlow.collect {
                    val velocity = encoder.velocity
                    stateFlow.value = Shooter.State(velocity, velocity >= MIN_LAUNCH_VELOCITY)
                }
            } finally {
                motor.power = 0.0
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}

