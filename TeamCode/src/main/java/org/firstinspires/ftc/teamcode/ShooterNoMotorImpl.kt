package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap

class ShooterNoMotorImpl(hardwareMap: HardwareMap) : Shooter {
    override var angleDegrees: Double = 0.0
        set(_) = TODO("Not yet implemented")

    override var shooterSpeed: Double = 0.0
        set(_) = TODO("Not yet implemented")

    private val hoodServo =
        hardwareMap.servo["hood"] ?: error("Nu s-a găsit servo-ul capacului de la shooter, vezi configurația")

    override var hood by hoodServo::position
    override var isRunning = false
        set(_) = TODO("Not yet implemented")

    override val isAtTarget = true

    override fun toString() = "Shooter(hood=$hood, isRunning=$isRunning, isAtTarget=$isAtTarget)"
}
