package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.zipWithNext
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2

interface Shooter {
    var angleDegrees: Double
    var hood: Double
    var velocityOffset: Double

    val stateFlow: StateFlow<State>

    fun shoot(currentDistance: () -> Double = { 0.0 }): Job

    data class State(val velocity: Double, val canShoot: Boolean)
}

fun Shooter.alignToPose(currentPose: Pose, targetPose: Pose, offset: Double = 0.0) {
    val angle = atan2(
        targetPose.y - currentPose.y,
        targetPose.x - currentPose.x
    )
    val normalizedAngle = MathFunctions.normalizeAngle(currentPose.heading).let {
        if (it > 3 / 2 * PI) it - 2 * PI else it
    }
    angleDegrees = Math.toDegrees(angle - normalizedAngle) + offset
}

private val shootCountMutex = Mutex()

suspend fun shootAll(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    shooterJob: Job? = null,
    shootOrder: List<ArtefactType> = emptyList(),
) {
    fun <T : Any> Iterator<T>.nextOrNull(): T? = if (hasNext()) next() else null
    if (shootCountMutex.isLocked) return
    if (shootOrder.isNotEmpty() && shootOrder.size != sorter.size)
        Log.e("Shooter", "shootAll called with order: $shootOrder but sorter size is ${sorter.size}")
    Log.d("ShooterImpl", "shootOrder: $shootOrder")
    val orderIterator = shootOrder.iterator()
    shootCountMutex.withLock {
        val count = sorter.size - 1
        if (count == 0) return@withLock
        val type = orderIterator.nextOrNull()
        sorter.prepareShoot(type)
        shootFlow
            .onEach { sorter.isLifting = it.canShoot }
            .map { it.velocity }
            .zipWithNext()
            .map { (prev, cur) -> prev - cur >= 120.0 }
            .distinctUntilChanged()
            .filter { it }
            .take(count)
            .onStart { RobotLog.dd("ShooterImpl", "Starting with $type") }
            .withIndex()
            .map { it.index }
            .collect { idx ->
                if (idx == count) return@collect
                val type = orderIterator.nextOrNull()
                RobotLog.dd("ShooterImpl", "Attempted next: $type")
                sorter.prepareShoot(type)
            }
        shooterJob?.cancel()
        sorter.prepareIntake()
    }
}
