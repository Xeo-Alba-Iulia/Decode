package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.async
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.selects.select
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.ArtefactType
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
    fun Sorter.shootOrDefault(type: ArtefactType?) {
        if (!prepareShoot(type) && type != null) {
            RobotLog.ee("Shooter", "Failed to prepare artefact $type, shooting default")
            prepareShoot()
        }
    }
    if (shootCountMutex.isLocked) return
    if (shootOrder.isNotEmpty() && shootOrder.size != sorter.size)
        Log.e("Shooter", "shootAll called with order: $shootOrder but sorter size is ${sorter.size}")
    Log.d("Shooter", "shootOrder: $shootOrder")
    val orderIterator = shootOrder.iterator()
    shootCountMutex.withLock {
        val count = sorter.size
        if (count == 0) return@withLock
        val type = orderIterator.nextOrNull()
        sorter.shootOrDefault(type)
        val collectorJob = coroutineScope {
            async {
                var isFirst = true
                var shotCount = 0
                shootFlow
                    .map { it.canShoot }
                    .dropWhile { !it }
                    .distinctUntilChanged()
                    .filter { it }
                    .onEach {
                        if (isFirst) {
                            sorter.isLifting = true
                            delay(400L)
                            isFirst = false
                        }
                    }
                    .takeWhile { !sorter.isEmpty }
                    .withIndex()
                    .map { it.index }
                    .onStart { RobotLog.dd("Shooter", "Starting with $type") }
                    .onCompletion {
                        sorter.prepareIntake()
                        if (it != null) throw it
                        ++shotCount
                    }
                    .collect {
                        val nextType = orderIterator.nextOrNull()
                        sorter.shootOrDefault(nextType)
                        ++shotCount
                    }
                shotCount
            }
        }
        select {
            collectorJob.onAwait { RobotLog.vv("Shooter", "shootAll ended after shooting $it") }
            shooterJob?.onJoin { RobotLog.ww("Shooter", "Shooter job ended in shootAll") }
        }
        sorter.isLifting = false
        shooterJob?.cancel()
        collectorJob.cancel()
    }
}
