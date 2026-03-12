package org.firstinspires.ftc.teamcode.sorter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import dev.zacsweers.metro.binding
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class, binding = binding<Sorter>(), replaces = [SorterImpl::class])
open class SorterWrapped(
    @Named("sorterServo") private val servo: Servo,
    private val transfer: Transfer,
    isAuto: Boolean,
) : Sorter {

    companion object {
        @JvmField
        var HALF_ROTATION = 0.285

        @JvmField
        var OFFSET = 0.024
    }

    override var isLifting by transfer::isRunning

    override var position by servo::position

    override val artefacts: Array<ArtefactType?> =
        if (isAuto)
            arrayOf(ArtefactType.PURPLE, ArtefactType.GREEN, ArtefactType.PURPLE)
        else
            arrayOfNulls(3)

    val positionsList = List(3) {
        buildList {
            var position = OFFSET - HALF_ROTATION + it * HALF_ROTATION * 2 / 3
            while (position < 1.0) {
                if (position >= 0.0)
                    add(it to position)
                position += 2 * HALF_ROTATION
            }
        }
    }

    private var currentIntakeSlot = -1

    override fun prepareIntake() {
        if (isFull) return
        currentIntakeSlot = artefacts.indexOfFirst { it == null }.also {
            servo.position = OFFSET + it * HALF_ROTATION * 2.0 / 3.0
        }
    }

    override fun intake(type: ArtefactType) {
        if (currentIntakeSlot == -1) {
            Log.e("SorterWrapped", "Intake called without prepareIntake")
            return
        }
        artefacts[currentIntakeSlot] = type
        currentIntakeSlot = -1
        if (!isFull) prepareIntake()
    }

    /* TODO: This can be heavily optimized if needed by precomputing positions
       From testing after warmup this takes 6-8ms when it has a match, and 500μs without one
    */
    override fun prepareShoot(type: ArtefactType?): Boolean {
        val oldPosition = servo.position
        val validIndices =
            artefacts
                .indices
                .filter { type?.equals(artefacts[it]) ?: (artefacts[it] != null) }
        val correctedList = validIndices.flatMap { positionsList[it] }
        val closest = correctedList.minByOrNull { (_, pos) -> abs(pos - oldPosition) }
        return closest?.let { (idx, position) ->
            servo.position = position
            artefacts[idx] = null
            true
        } ?: false
    }

    override val size get() = artefacts.count { it != null }

    override fun toString() = "SorterWrapped(artefacts = ${artefacts.contentToString()}, position = $position)"
}