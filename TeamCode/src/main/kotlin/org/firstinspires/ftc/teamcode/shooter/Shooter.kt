package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.PI
import kotlin.math.atan2

interface Shooter {
    var angleDegrees: Double
    val stateFlow: StateFlow<State>

    fun shoot(distanceFlow: Flow<Double>): Job
    fun shoot(currentDistance: () -> Double) = shoot(flow { while (true) emit(currentDistance()) })

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
    shooterJob: Job,
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
        sorter.isLifting = true
        delay(400L)
        shootFlow.map { it.canShoot }.filter { it }.first()
        for (i in 1..<count) {
            sorter.shootOrDefault(orderIterator.nextOrNull())
            RobotLog.dd("Shooter", "Shot $i balls")
            delay(400L)
        }
        delay(450L)
        sorter.prepareIntake()
        delay(100L)
        sorter.isLifting = false
        shooterJob.cancel()
    }
}

suspend fun shootAuto(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    shooterJob: Job,
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
        sorter.prepareShoot(type)
        sorter.isLifting = true
        delay(1000L)
        shootFlow
            .map { it.canShoot }
            .distinctUntilChanged()
            .filter { it }
            .take(count)
            .onStart { RobotLog.dd("Shooter", "Starting with $type") }
            .collect {
                val type = orderIterator.nextOrNull()
                RobotLog.dd("Shooter", "Attempted next: $type")
                sorter.shootOrDefault(type)
                delay(1000L)
            }
        sorter.isLifting = false
        shooterJob.cancel()
        sorter.prepareIntake()
    }
}
