package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var shooterSpeed: Double
    var isRunning: Boolean
    val isAtTarget: Boolean
}

fun Shooter(hardwareMap: HardwareMap): Shooter = ShooterNoMotorImpl(hardwareMap)
