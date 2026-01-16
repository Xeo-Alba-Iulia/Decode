package org.firstinspires.ftc.teamcode.shooter

import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.StateFlow

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var velocity: Double

    val stateFlow: StateFlow<State>

    fun shoot(): Job

    data class State(val hood: Double, val velocity: Double, val canShoot: Boolean)
}
