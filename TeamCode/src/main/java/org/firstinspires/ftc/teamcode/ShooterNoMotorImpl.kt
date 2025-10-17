package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ShooterNoMotorImpl(hardwareMap: HardwareMap) : Shooter {
    override var angleDegrees: Double = 0.0
        set(_) {}

    override var shooterSpeed: Double = 0.0
        set(_) {}

    private val hoodServo: Servo = hardwareMap.servo["hood"]

    override var hood by hoodServo::position
    override var isRunning = false
        set(_) {}

    override val isAtTarget = false

    override fun toString() = "ShooterNoMotorImpl(hood=$hood)"
}
