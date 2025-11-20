package org.firstinspires.ftc.teamcode.intake

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named

@Configurable
@Inject
class Intake(@Named("intakeMotor") private val motor: DcMotor) {
    var isRunning: Boolean
        get() = motor.power != 0.0
        set(value) {
            motor.power = if (value) INTAKE_POWER else 0.0
        }

    companion object {
        @JvmField
        var INTAKE_POWER = 0.7
    }
}