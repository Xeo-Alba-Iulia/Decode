package org.firstinspires.ftc.teamcode.shooter

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var shooterSpeed: Double
    val isRunning: Boolean

    suspend fun turnOn()
    fun turnOff()
}
