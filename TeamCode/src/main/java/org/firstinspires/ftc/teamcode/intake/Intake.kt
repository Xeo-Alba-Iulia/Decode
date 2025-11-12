package org.firstinspires.ftc.teamcode.intake

import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hardwareMap: HardwareMap) {
    private val intakeMotor = hardwareMap.dcMotor["IntakeMotor"]

    var isRunning: Boolean
        get() = intakeMotor.power != 0.0
        set(value) {
            intakeMotor.power = if (value) 1.0 else 0.0
        }
}