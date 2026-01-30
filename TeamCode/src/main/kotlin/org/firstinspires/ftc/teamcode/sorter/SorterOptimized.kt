package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.binding
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import java.util.*
import kotlin.math.abs
import kotlin.time.measureTime

@Inject
@ContributesBinding(OpModeScope::class, binding<Sorter>(), [SorterImpl::class, SorterWrapped::class])
class SorterOptimized(
    @Named("sorterServo") servo: Servo,
    transfer: Transfer,
    isAuto: Boolean,
) : SorterWrapped(servo, transfer, isAuto) {
    val closestPosition = Array(3) { artefactIdx ->
        val map = buildMap {
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
        DoubleArray(1001) { map.floorEntry(it)!!.value }
    }

    val distances = Array(3) { artefactIdx ->
        DoubleArray(1001) { posIdx ->
            abs(posIdx / 1000.0 - closestPosition[artefactIdx][posIdx])
        }
    }

    override fun prepareShoot(type: ArtefactType?): Boolean {
        val result: Boolean
        val oldPosition = (position * 1000.0).toInt()
        val usedTime = measureTime {
            result = artefacts
                .indices
                .filter { type?.equals(artefacts[it]) ?: (artefacts[it] != null) }
                .minByOrNull { distances[it][oldPosition] }
                ?.let {
                    position = closestPosition[it][oldPosition]
                    artefacts[it] = null
                    true
                } ?: false
        }
        println("SorterOptimized prepareShoot took $usedTime")
//        RobotLog.dd("SorterOptimized", "prepareShoot took $usedTime")
        return result
    }

    override fun toString() = "SorterOptimized(artefacts = ${artefacts.contentToString()}, position = $position)"
}