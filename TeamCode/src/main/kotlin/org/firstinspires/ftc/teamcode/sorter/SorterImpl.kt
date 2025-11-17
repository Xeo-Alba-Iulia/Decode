package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@SingleIn(OpModeScope::class)
@ContributesBinding(OpModeScope::class)
class SorterImpl(@Named("SorterServo") private val servo: Servo) : Sorter {
    companion object {
        @JvmField
        var INTAKE_POSITIONS = doubleArrayOf(0.05, 0.412, 0.81)

        @JvmField
        var SHOOTER_POSITIONS = doubleArrayOf(0.593, 0.991, 0.231)
    }

    private val artefacts = arrayOfNulls<ArtefactType>(3)

    private var currentIntakeSlot: Int = -1

    override var size = 0
        private set

    fun intakePosition(index: Int) {
        servo.position = INTAKE_POSITIONS[index]
    }

    fun shooterPosition(index: Int) {
        servo.position = SHOOTER_POSITIONS[index]
    }

    init { prepareIntake() }

    override fun prepareIntake() {
        require(!isFull) { "Sorter is full" }
        currentIntakeSlot = artefacts.indexOfFirst { it == null }.also { intakePosition(it) }
    }

    override fun intake(type: ArtefactType) {
        require(currentIntakeSlot != -1) { "Sorter not prepared for intake" }
        artefacts[currentIntakeSlot] = type
        size++
        if (!isFull) prepareIntake() else currentIntakeSlot = -1
    }

    override fun shoot(type: ArtefactType?): Boolean {
        require(!isEmpty) { "Sorter is empty" }
        currentIntakeSlot = -1

        return artefacts.indexOfLast { storedType ->
            if (type == null) storedType != null else type == storedType
        }.takeIf { it != -1 }?.let {
            shooterPosition(it)
            size--
            if (isEmpty)
                prepareIntake()
            true
        } ?: false
    }

    override fun toString() = "SorterImpl(artefacts = ${artefacts.contentToString()})"
}