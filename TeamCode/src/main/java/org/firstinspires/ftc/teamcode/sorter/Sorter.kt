package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
class Sorter(map: HardwareMap) {
    companion object {
        @JvmField
        var INTAKE_POSITIONS = doubleArrayOf(0.05, 0.412, 0.81)

        @JvmField
        var SHOOTER_POSITIONS = doubleArrayOf(0.593, 0.991, 0.231)
    }

    private val servo = map.servo["sorter"]
    private val ballArray = booleanArrayOf(false, false, false)
    private var currentIndex = 0

    fun getBall() {
        for ((i, hasBall) in ballArray.withIndex().reversed()) {
            if (hasBall) {
                servo.position = SHOOTER_POSITIONS[i]
                ballArray[i] = false
                currentIndex = i
                return
            }
        }
        throw EmptySorter()
    }

    fun intakeBall(): Int {
        for ((i, hasBall) in ballArray.withIndex()) {
            if (!hasBall) {
                servo.position = INTAKE_POSITIONS[i]
                ballArray[i] = true
                currentIndex = i
                return i
            }
        }
        throw FullSorter()
    }

    fun resetPosition() {
        servo.position = INTAKE_POSITIONS[currentIndex]
    }
}