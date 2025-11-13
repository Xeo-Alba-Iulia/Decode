package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesBinding
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@ContributesBinding(OpModeScope::class)
class SorterImpl(map: HardwareMap) : Sorter {
    companion object {
        @JvmField
        var INTAKE_POSITIONS = doubleArrayOf(0.05, 0.412, 0.81)

        @JvmField
        var SHOOTER_POSITIONS = doubleArrayOf(0.593, 0.991, 0.231)
    }

    private val servo = map.servo["sorter"]
    private val artefacts = arrayOfNulls<ArtefactType>(3)

    fun intakePosition(index: Int) {
        servo.position = INTAKE_POSITIONS[index]
    }

    fun shooterPosition(index: Int) {
        servo.position = SHOOTER_POSITIONS[index]
    }

    override fun intake(type: ArtefactType) =
        artefacts.indexOfFirst { it == null }.takeUnless { it == -1 }?.let {
            intakePosition(it)
            artefacts[it] = type
            true
        } ?: false


    override fun shoot(type: ArtefactType?) =
        artefacts.indexOfLast { storedType ->
            if (type == null) storedType != null else storedType == type
        }.takeUnless { it == -1 }?.let {
            shooterPosition(it)
            artefacts[it] = null
            true
        } ?: false
}