package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import kotlinx.coroutines.selects.select
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
    fun shoot(currentDistance: () -> Double? = { null }): Job

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
    Log.d("Shooter", "shootOrder: $shootOrder")
    val orderIterator = shootOrder.iterator()
    shootCountMutex.withLock {
        val count = sorter.size
        if (count == 0) return@withLock
        val type = orderIterator.nextOrNull()
        sorter.prepareShoot(type)
        val collectorJob = coroutineScope {
            launch {
                shootFlow
                    .onEach { sorter.isLifting = it.canShoot }
                    .map { it.velocity }
                    .zipWithNext()
                    .map { (prev, cur) -> prev - cur >= 120.0 }
                    .distinctUntilChanged()
                    .filter { it }
                    .take(count)
                    .onStart { RobotLog.dd("Shooter", "Starting with $type") }
                    .withIndex()
                    .map { it.index }
                    .collect { idx ->
                        if (idx == count) return@collect
                        val type = orderIterator.nextOrNull()
                        RobotLog.dd("Shooter", "Attempted next: $type")
                        if (!sorter.prepareShoot(type)) {
                            RobotLog.ee("Shooter", "Failed to get artefact, attempting any")
                            if (!sorter.prepareShoot(null)) {
                                RobotLog.ee("Shooter", "Shooter was emptied faster than expected, stopping")
                                cancel("Shooter emptied before shooting $count, check wasShot detection")
                            }
                        }
                    }
            }
        }
        select {
            collectorJob.onJoin { RobotLog.vv("Shooter", "shootAll ended after shooting $count") }
            shooterJob?.onJoin { RobotLog.ww("Shooter", "Shooter job ended in shootAll") }
        }
        sorter.isLifting = false
        shooterJob?.cancel()
        sorter.prepareIntake()
    }
}
