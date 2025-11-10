package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap

@Config
class Sorter(map: HardwareMap) {
    companion object {
        @JvmField
        var SERVO_POSITIONS = doubleArrayOf(0.2, 0.5, 0.8)
    }

    private val servo = map.servo["sorter"]
    private val ballArray = booleanArrayOf(false, false, false)

    fun getBall() {
        for((i, hasBall) in ballArray.withIndex()) {
            if (hasBall) {
                servo.position = SERVO_POSITIONS[i]
                ballArray[i] = false
                return
            }
        }
        error("No balls to get")
    }

    fun intakeBall(): Int {
        for ((i, hasBall) in ballArray.withIndex()) {
            if (!hasBall) {
                servo.position = SERVO_POSITIONS[i]
                ballArray[i] = true
                return i
            }
        }
        error("No space for more balls")
    }
}