package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@SingleIn(OpModeScope::class)
@Inject
class ShooterNoMotorImpl(@param:Named("ShooterHoodServo") private val hoodServo: Servo) : Shooter {
    override var angleDegrees: Double = 0.0
        set(_) {}

    override var shooterSpeed: Double = 0.0
        set(_) {}

    override var hood by hoodServo::position
    override var isRunning = false
        set(_) {}

    override val isAtTarget = false

    override fun toString() = "ShooterNoMotorImpl(hood=$hood)"
}
