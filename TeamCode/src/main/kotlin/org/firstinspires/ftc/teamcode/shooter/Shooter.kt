package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
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
import org.firstinspires.ftc.teamcode.sorter.SorterImpl
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

@Config
object ShooterConfig {
    @JvmField
    var SHOOTER_BACK_OFFSET_INCHES = 2.0
    @JvmField
    var APRIL_TAG_HEIGHT_METERS = 0.74
}

interface Shooter {
    var angleDegrees: Double
    val stateFlow: StateFlow<State>

    fun shoot(distanceFlow: Flow<Double>): Job
    fun shoot(currentDistance: () -> Double) = shoot(flow { while (true) emit(currentDistance()) })

    data class State(val velocity: Double, val canShoot: Boolean, val trajectoryF: Double = Double.NaN)
}

fun getShooterPose(robotPose: Pose): Pose =
    Pose(
        robotPose.x - cos(robotPose.heading) * ShooterConfig.SHOOTER_BACK_OFFSET_INCHES,
        robotPose.y - sin(robotPose.heading) * ShooterConfig.SHOOTER_BACK_OFFSET_INCHES,
        robotPose.heading
    )

fun limelightGroundDistanceMeters(xMeters: Double, zMeters: Double): Double {
    val limelightDistance = hypot(xMeters, zMeters)
    return sqrt(max(0.0, limelightDistance * limelightDistance - ShooterConfig.APRIL_TAG_HEIGHT_METERS * ShooterConfig.APRIL_TAG_HEIGHT_METERS))
}

fun Shooter.alignToPose(currentPose: Pose, targetPose: Pose, offset: Double = 0.0) {
    val shooterPose = getShooterPose(currentPose)
    val angle = atan2(
        targetPose.y - shooterPose.y,
        targetPose.x - shooterPose.x
    )
    val normalizedAngle = MathFunctions.normalizeAngle(shooterPose.heading).let {
        if (it > PI + angle) it - 2 * PI else it
    }
    val maxSetAngle = ShooterImpl.MAX_TURRET_ANGLE + 20.0
    (Math.toDegrees(angle - normalizedAngle) + offset)
        .takeIf { it in -maxSetAngle..maxSetAngle }?.let { angleDegrees = it }
}

suspend fun shootPattern(
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
    Log.d("Shooter", "count: $count")
    if (shootOrder.size != sorter.size)
        Log.e("Shooter", "Sorter size is ${sorter.size}, but expected ${shootOrder.size}")
    val orderIterator = shootOrder.iterator()
    repeat(count) {
        assert(sorter.shootOrDefault(orderIterator.nextOrNull()))
        delay(300.milliseconds)
        sorter.isLifting = true
        delay(200.milliseconds)
        sorter.isLifting = false
    }
    sorter.prepareIntake()
    shooterJob.cancel()
}

suspend fun Sorter.fastShoot() {
    isLifting = true
    position = SorterImpl.SHOOTER_POSITIONS[2]
    delay(0.8.seconds)
    for (i in artefacts.indices)
        artefacts[i] = null
    (this as? SorterImpl)?.run { size = 0 }
    isLifting = false
    prepareIntake()
}

fun Sorter.prepareFastShoot() {
    val size = size
    if (size != 3)
        Log.e("Auto", "Prepare fast shoot with size: $size")
    position = SorterImpl.SHOOTER_POSITIONS[0] - .02
}
