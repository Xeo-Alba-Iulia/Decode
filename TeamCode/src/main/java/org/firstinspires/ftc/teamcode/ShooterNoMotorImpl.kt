package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class ShooterNoMotorImpl(hardwareMap: HardwareMap) : Shooter {
    override var angleDegrees: Double = 0.0
        set(_) = TODO("Not yet implemented")

    override var shooterSpeed: Double = 0.0
        set(_) = TODO("Not yet implemented")

    private val hoodServo: Servo = hardwareMap.servo["hood"]

    override var hood by hoodServo::position
    override var isRunning = false
        set(_) = TODO("Not yet implemented")

    override val isAtTarget = true

    override fun toString() = "Shooter(hood=$hood, isRunning=$isRunning, isAtTarget=$isAtTarget)"
}
