package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import org.firstinspires.ftc.teamcode.ArtefactType
import java.util.*
import kotlin.math.abs
import kotlin.time.measureTime

@Inject
//@ContributesBinding(OpModeScope::class, binding<Sorter>(), [SorterImpl::class, SorterWrapped::class])
class SorterOptimized(
    @Named("sorterServo") servo: Servo,
    transfer: Transfer,
    isAuto: Boolean,
) : SorterWrapped(servo, transfer, isAuto) {
    val closestPosition = List(3) { artefactIdx ->
        buildMap {
            val positions = (0..5)
                .map {
                    OFFSET - HALF_ROTATION + HALF_ROTATION * artefactIdx * 2 / 3 + 2 * it * HALF_ROTATION
                }.dropWhile { it < 0.0 }
                .takeWhile { it <= 1.0 }
            put(0, positions.first())
            positions
                .zipWithNext()
                .forEach { (a, b) ->
                    val mid = (a + b) / 2
                    put((mid * 1000.0).toInt(), b)
                }
        }.toSortedMap() as NavigableMap
    }

    override suspend fun prepareShoot(type: ArtefactType?): Boolean {
        val result: Boolean
        val oldPosition = position
        val usedTime = measureTime {
            result = artefacts
                .indices
                .filter { type?.equals(artefacts[it]) ?: (artefacts[it] != null) }
                .map { it to closestPosition[it].floorEntry((oldPosition * 1000.0).toInt())!!.value }
                .minByOrNull { (_, pos) -> abs(oldPosition - pos) }
                ?.let { (idx, pos) ->
                    position = pos
                    artefacts[idx] = null
                    true
                } ?: false
        }
        RobotLog.dd("SorterOptimized", "prepareShoot took $usedTime")
        return result
    }

    override fun toString() = "SorterOptimized(artefacts = ${artefacts.contentToString()}, position = $position)"
}