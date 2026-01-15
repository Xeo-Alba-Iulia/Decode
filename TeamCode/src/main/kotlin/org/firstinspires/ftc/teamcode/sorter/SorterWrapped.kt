package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import dev.zacsweers.metro.binding
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class, binding = binding<Sorter>(), replaces = [SorterImpl::class])
class SorterWrapped(
    @Named("sorterServo") private val servo: Servo,
    private val transfer: Transfer,
) : Sorter, OpModeObserver {
    companion object {
        @JvmField
        var HALF_ROTATION = 0.285

        @JvmField
        var OFFSET = 0.015

        @JvmField
        var SPEED = 1000.0
    }

    override var isLifting by transfer::isRunning

    override var position by servo::position

    private var artefacts = arrayOfNulls<ArtefactType>(3)
    private var currentIntakeSlot: Int = -1

    override suspend fun prepareIntake() {
        require(!isFull) { "Sorter is full" }
        currentIntakeSlot = artefacts.indexOfFirst { it == null }.also {
            servo.position = OFFSET + it * HALF_ROTATION * 2.0 / 3.0
        }
    }

    override suspend fun intake(type: ArtefactType) {
        require(currentIntakeSlot != -1) { "Sorter not prepared for intake" }
        artefacts[currentIntakeSlot] = type
        size++
        if (!isFull) prepareIntake() else currentIntakeSlot = -1
    }

    //TODO: This can be heavily optimized if needed by precomputing positions
    override suspend fun prepareShoot(type: ArtefactType?) =
        artefacts.asSequence().withIndex()
            .filter { (_, storedType) -> type?.equals(storedType) ?: (storedType != null) }
            .map(IndexedValue<*>::index)
            .flatMap {
                buildList {
                    var position = OFFSET + HALF_ROTATION + it * HALF_ROTATION * 2 / 3
                    while (position < 1.0) {
                        add(it to position)
                        position += 2 * HALF_ROTATION
                    }
                }
            }.minByOrNull { (_, position) ->
                abs(servo.position - position)
            }?.let { (idx, position) ->
                val oldPosition = servo.position
                servo.position = position
                artefacts[idx] = null
                size--
                delay((abs(servo.position - oldPosition) * SPEED).toLong())
                true
            } ?: false

    override suspend fun onStart(opMode: OpMode) = prepareIntake()

    override var size = 0
        private set

    override fun toString() = "SorterWrapped(artefacts = ${artefacts.contentToString()}, position = $position)"
}