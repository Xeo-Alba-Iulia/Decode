package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class)
class ShooterNoEncoder(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
) : Shooter {

    override var angleDegrees: Double
        get() = (rotationServo.position * 120.0) - 60.0
        set(value) {
            rotationServo.position = (value + 60.0) / 120.0
        }
    override var hood by hoodServo::position
    override var shooterSpeed = 0.7
    override var isRunning = false
        set(value) {
            field = value
            motor.power = if (value) shooterSpeed else 0.0
        }

    override val isAtTarget = true

    override fun toString() =
        "ShooterNoEncoder(angleDegrees=$angleDegrees, hood=$hood, shooterSpeed=$shooterSpeed, isRunning=$isRunning)"
}