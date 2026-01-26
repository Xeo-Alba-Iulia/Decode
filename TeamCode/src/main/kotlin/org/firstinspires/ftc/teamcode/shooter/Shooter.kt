package org.firstinspires.ftc.teamcode.shooter

import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.FlowPreview
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
    var velocity: Double

    val stateFlow: StateFlow<State>

    fun shoot(currentDistance: () -> Double = { 2.2 }): Job

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

@OptIn(FlowPreview::class)
suspend fun shootAll(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    shooterJob: Job? = null,
    vararg shootOrder: ArtefactType,
) {
    fun <T : Any> Iterator<T>.nextOrNull(): T? = if (hasNext()) next() else null
    if (shootCountMutex.isLocked) return
    val orderIterator = shootOrder.iterator()
    shootCountMutex.withLock {
        val count = sorter.size
        if (count == 0) return@withLock
        sorter.prepareShoot(orderIterator.nextOrNull())
        var alreadyShot = 0
        shootFlow
            .map { it.velocity }
            .zipWithNext()
            .map { (prev, cur) -> prev - cur >= 100.0 }
            .distinctUntilChanged()
            .filter { it }
            .take(count)
            .collect {
                RobotLog.dd("ShooterImpl", "Attempted next")
                if (++alreadyShot == count) return@collect
                sorter.prepareShoot(orderIterator.nextOrNull())
            }
        sorter.prepareIntake()
        shooterJob?.cancel()
    }
}
