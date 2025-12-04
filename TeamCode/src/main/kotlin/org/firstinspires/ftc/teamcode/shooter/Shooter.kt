package org.firstinspires.ftc.teamcode.shooter

import kotlinx.coroutines.flow.StateFlow

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var velocity: Double
    val isRunning: Boolean
    val isAtTarget: StateFlow<Boolean>

    suspend fun turnOn()
    fun turnOff()
}
