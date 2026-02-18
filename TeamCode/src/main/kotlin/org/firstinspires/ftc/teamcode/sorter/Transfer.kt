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
class Transfer(@Named("transfer") private val motor: DcMotor) {
    var isRunning = false
        set(value) {
            field = value
            motor.power = if (value) POWER else 0.0
        }

    init {
        motor.power = 0.0
    }

    companion object {
        @JvmField
        var POWER = 1.0
    }
}