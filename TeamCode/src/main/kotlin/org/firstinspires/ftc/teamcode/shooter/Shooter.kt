package org.firstinspires.ftc.teamcode.shooter

import kotlinx.coroutines.flow.Flow

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var velocity: Double

    fun shoot(): Flow<State>

    data class State(val hood: Double, val velocity: Double, val canShoot: Boolean)
}
