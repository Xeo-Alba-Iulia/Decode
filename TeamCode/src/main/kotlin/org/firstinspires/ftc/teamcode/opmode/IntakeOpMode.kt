package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.intake.Intake

@TeleOp(group = "Tests")
open class IntakeOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var intake: Intake

    override fun init() {
        telemetry = opModeGraph.telemetry
        intake = opModeGraph.intake
    }

    override fun loop() {
        when {
            gamepad1.aWasPressed() -> intake.isRunning = true
            gamepad1.bWasPressed() -> intake.isRunning = false
        }

        telemetry.addData("Runtime", runtime)
        telemetry.addData("Intake Running", intake.isRunning)
        telemetry.addData("Intake Power", Intake.INTAKE_POWER)
        telemetry.update()
    }
}