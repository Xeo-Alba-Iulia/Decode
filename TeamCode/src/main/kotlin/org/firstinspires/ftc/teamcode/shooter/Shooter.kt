package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.flow
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds

interface Shooter {
    var angleDegrees: Double
    val stateFlow: StateFlow<State>

    fun shoot(distanceFlow: Flow<Double>): Job
    fun shoot(currentDistance: () -> Double) = shoot(flow { while (true) emit(currentDistance()) })

    data class State(val velocity: Double, val canShoot: Boolean)
}

//TODO: Fix not going fully to -80
fun Shooter.alignToPose(currentPose: Pose, targetPose: Pose, offset: Double = 0.0) {
    val angle = atan2(
        targetPose.y - currentPose.y,
        targetPose.x - currentPose.x
    )
    val normalizedAngle = MathFunctions.normalizeAngle(currentPose.heading).let {
        if (it > PI * 3 / 2) it - 2 * PI else it
    }
    angleDegrees = Math.toDegrees(angle - normalizedAngle) + offset
}

suspend fun shootPattern(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    shooterJob: Job,
    shootOrder: List<ArtefactType>,
) {
    fun Iterator<ArtefactType>.nextOrNull() = if (hasNext()) next() else null
    fun Sorter.shootOrDefault(type: ArtefactType?): Boolean {
        var result = prepareShoot(type)
        if (!result && type != null) {
            RobotLog.ee("Shooter", "Failed to prepare artefact $type, shooting default")
            result = prepareShoot()
        }
        return result
    }
    val count = sorter.size
    if (count == 0) return
    Log.d("Shooter", "shootOrder: $shootOrder")
    if (shootOrder.size != sorter.size)
        Log.e("Shooter", "Sorter size is ${sorter.size}, but expected ${shootOrder.size}")
    val orderIterator = shootOrder.iterator()
    repeat(count) {
        assert(sorter.shootOrDefault(orderIterator.nextOrNull()))
        delay(450.milliseconds)
        sorter.isLifting = true
        delay(100.milliseconds)
        sorter.isLifting = false
    }
    sorter.prepareIntake()
    shooterJob.cancel()
}
