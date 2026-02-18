package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import java.util.*
import kotlin.math.abs

@Inject
@SingleIn(OpModeScope::class)
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
        DoubleArray(1001) { posIdx -> abs(posIdx / 1000.0 - closestPosition[artefactIdx][posIdx]) }
    }

    override fun prepareShoot(type: ArtefactType?): Boolean {
        val oldPosition = (position * 1000.0).toInt()
        return artefacts
                .indices
                .filter { type?.equals(artefacts[it]) ?: (artefacts[it] != null) }
                .minByOrNull { distances[it][oldPosition] }
                ?.let {
                    position = closestPosition[it][oldPosition]
                    artefacts[it] = null
                    true
                } ?: false
    }

    override fun toString() = "SorterOptimized(artefacts = ${artefacts.contentToString()}, position = $position)"
}