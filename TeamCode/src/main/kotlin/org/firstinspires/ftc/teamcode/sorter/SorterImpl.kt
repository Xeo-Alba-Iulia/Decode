package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import dev.zacsweers.metro.binding
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped.Companion.HALF_ROTATION
import org.firstinspires.ftc.teamcode.sorter.SorterWrapped.Companion.OFFSET

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class, binding<Sorter>())
class SorterImpl(
    @Named("sorterServo") private val servo: Servo,
    private val transfer: Transfer
) : Sorter, OpModeObserver {

    companion object {
        @JvmField
        var INTAKE_POSITIONS =
            (0..2).map { OFFSET + it * HALF_ROTATION * 2 / 3 }.toDoubleArray()

        @JvmField
        var SHOOTER_POSITIONS =
            (0..2).map { OFFSET + HALF_ROTATION + HALF_ROTATION * 2 / 3 * it }.toDoubleArray()
    }

    override var isLifting by transfer::isRunning

    override var position by servo::position

    override val artefacts = arrayOfNulls<ArtefactType>(3) // TODO: Auto start config

    private var currentIntakeSlot: Int = -1

    override var size = 0

    fun intakePosition(index: Int) {
        servo.position = INTAKE_POSITIONS[index]
    }

    fun shooterPosition(index: Int) {
        servo.position = SHOOTER_POSITIONS[index]
    }

    override fun prepareIntake() {
        if (isFull) return
        currentIntakeSlot = artefacts.indexOfFirst { it == null }.also { intakePosition(it) }
    }

    override fun intake(type: ArtefactType) {
        if (currentIntakeSlot == -1) return
        artefacts[currentIntakeSlot] = type
        size++
        if (!isFull) prepareIntake() else currentIntakeSlot = -1
    }

    override fun prepareShoot(type: ArtefactType?): Boolean {
        if (isEmpty) return false
        currentIntakeSlot = -1
        return artefacts.indexOfFirst { storedType ->
            if (type == null) storedType != null else type == storedType
        }.takeIf { it != -1 }?.let {
            shooterPosition(it)
            artefacts[it] = null
            size--
            true
        } ?: false
    }

    override suspend fun onStart(opMode: OpMode) = prepareIntake()

    override fun toString() = "SorterImpl(artefacts = ${artefacts.contentToString()}, position = $position)"
}