package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.flow
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    private val tickFlow: SharedFlow<Unit>,
) : Shooter {

    companion object {
        @JvmField
        @Volatile
        var TOLERANCE = 20.0
    }

    init {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    override var angleDegrees by rotationServo::position
    override var hood by hoodServo::position
    override var velocity: Double
        get() = motor.velocity
        set(value) {
            desiredVelocity = value
            motor.velocity = value
        }

    private var desiredVelocity = 0.0

    override fun shoot(): Flow<Shooter.State> {
        motor.power = 1.0
        return flow {
            try {
                tickFlow.collect {
                    val currentVelocity = motor.velocity
                    emit(Shooter.State(hood, currentVelocity, abs(currentVelocity - desiredVelocity) <= TOLERANCE))
                }
            } finally {
                motor.power = 0.0
            }
        }
    }
}
