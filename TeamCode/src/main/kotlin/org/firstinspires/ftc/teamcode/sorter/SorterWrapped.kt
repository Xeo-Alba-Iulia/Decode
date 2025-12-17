package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import dev.zacsweers.metro.binding
import org.firstinspires.ftc.teamcode.KotlinVariableProvider
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs
import kotlin.properties.Delegates.observable

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class, binding = binding<Sorter>(), replaces = [SorterImpl::class])
class SorterWrapped(@Named("sorterServo") private val servo: Servo) : Sorter, OpModeObserver {
    companion object {
        var HALF_ROTATION by observable(0.283) { _, _, _ -> updatePositions() }

        var OFFSET by observable(0.122) { _, _, _ -> updatePositions() }

        private fun updatePositions() {}

        init {
            FtcDashboard.getInstance().run {
                addConfigVariable(
                    SorterWrapped::class.simpleName,
                    ::HALF_ROTATION.name,
                    KotlinVariableProvider(::HALF_ROTATION)
                )
                addConfigVariable(
                    SorterWrapped::class.simpleName,
                    ::OFFSET.name,
                    KotlinVariableProvider(::OFFSET)
                )
            }
        }
    }

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

    override suspend fun shoot(type: ArtefactType?) =
        artefacts.withIndex().filter { (_, storedType) -> type?.equals(storedType) ?: (storedType != null) }
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
                servo.position = position
                artefacts[idx] = null
                size--
                true
            } ?: false

    override suspend fun onStart(opMode: OpMode) = prepareIntake()

    override var size = 0
        private set

    override fun toString() = "SorterWrapped(artefacts = ${artefacts.contentToString()}, position = $position)"
}