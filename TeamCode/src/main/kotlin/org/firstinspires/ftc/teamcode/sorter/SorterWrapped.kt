package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import dev.zacsweers.metro.binding
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs
import kotlin.time.measureTime

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class, binding = binding<Sorter>(), replaces = [SorterImpl::class])
class SorterWrapped(
    @Named("sorterServo") private val servo: Servo,
    private val transfer: Transfer,
    isAuto: Boolean,
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

    private var artefacts: Array<ArtefactType?> =
        if (isAuto)
            arrayOf(ArtefactType.PURPLE, ArtefactType.PURPLE, ArtefactType.GREEN)
        else
            arrayOfNulls(3)

    @Volatile
    private var currentIntakeSlot: Int = -1

    override suspend fun prepareIntake() {
        if (isFull) return
        currentIntakeSlot = artefacts.indexOfFirst { it == null }.also {
            servo.position = OFFSET + it * HALF_ROTATION * 2.0 / 3.0
        }
    }

    override suspend fun intake(type: ArtefactType) {
        if (currentIntakeSlot == -1) return
        artefacts[currentIntakeSlot] = type
        size++
        currentIntakeSlot = -1
        if (!isFull) prepareIntake()
    }

    /* TODO: This can be heavily optimized if needed by precomputing positions
       From testing after warmup this takes 6-8ms when it has a match, and 500μs without one
    */
    override suspend fun prepareShoot(type: ArtefactType?): Boolean {
        val wasSuccessful: Boolean
        val oldPosition = servo.position
        val usedTime = measureTime {
            wasSuccessful = artefacts.asSequence().withIndex()
                .filter { (_, storedType) -> type?.equals(storedType) ?: (storedType != null) }
                .map(IndexedValue<*>::index)
                .flatMap {
                    buildList {
                        var position = OFFSET - HALF_ROTATION + it * HALF_ROTATION * 2 / 3
                        while (position < 1.0) {
                            if (position >= 0.0)
                                add(it to position)
                            position += 2 * HALF_ROTATION
                        }
                    }
                }.minByOrNull { (_, position) ->
                    abs(servo.position - position)
                }?.let { (idx, position) ->
                    servo.position = position
                    artefacts[idx] = null
                    size--
                    true
                } ?: false
        }
        RobotLog.dd("Sorter", "prepareShoot took $usedTime")
        delay((abs(servo.position - oldPosition) * SPEED).toLong())
        return wasSuccessful
    }

    override suspend fun onStart(opMode: OpMode) = prepareIntake()

    override var size = if (isAuto) 3 else 0
        private set

    override fun toString() = "SorterWrapped(artefacts = ${artefacts.contentToString()}, position = $position)"
}