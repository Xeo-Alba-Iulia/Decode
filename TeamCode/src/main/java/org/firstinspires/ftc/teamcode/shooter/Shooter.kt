package org.firstinspires.ftc.teamcode.shooter

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var shooterSpeed: Double
    var isRunning: Boolean
    val isAtTarget: Boolean
}
