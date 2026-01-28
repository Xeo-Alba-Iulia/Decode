package org.firstinspires.ftc.teamcode.sorter

import dev.zacsweers.metro.Inject
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped.Companion.HALF_ROTATION
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped.Companion.OFFSET
import java.util.*
import kotlin.math.abs

@Inject
class SorterOptimized(private val sorter: SorterWrapped) : Sorter by sorter {
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
        }.toSortedMap() as TreeMap
    }

    override suspend fun prepareShoot(type: ArtefactType?) =
        sorter.artefacts
            .indices
            .filter { type?.equals(sorter.artefacts[it]) ?: (sorter.artefacts[it] != null) }
            .map { it to closestPosition[it].floorEntry((sorter.position * 1000.0).toInt())!!.value }
            .minByOrNull { (_, pos) -> abs(sorter.position - pos) }
            ?.let { (idx, pos) ->
                sorter.position = pos
                sorter.artefacts[idx] = null
                true
            } ?: false
}