package org.firstinspires.ftc.teamcode.sorter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@Config
@Inject
@SingleIn(OpModeScope::class)
class Transfer(@Named("transferServo") private val servo: DcMotor) {
    var isRunning = false
        set(value) {
            field = value
            servo.power = if (value) POWER else 0.0
        }

    init {
        servo.power = 0.0
    }

    companion object {
        @JvmField
        var POWER = 1.0
    }
}