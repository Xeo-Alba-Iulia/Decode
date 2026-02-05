package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.intake.Intake

@TeleOp(group = "Systems")
open class IntakeOpMode : CoroutineOpMode() {
    lateinit var intake: Intake
    lateinit var dashTelemetry: Telemetry

    override fun init() {
        intake = opModeGraph.intake.apply { isDebug = true }
        dashTelemetry = FtcDashboard.getInstance().telemetry
//        sorter = opModeGraph.sorter.also {
//            opModeScope.launch { it.prepareIntake() }
//        }

//        intake.artefactFlow
//            .onEach {
//                telemetry.addData("ArtefactType", it)
//            }
//            .zipWithNext()
//            .onEach { (previous, _) ->
//                if (previous != null) {
//                    sorter.intake(previous)
//                }
//            }
//            .launchIn(opModeScope)
    }

    override fun loop() {
        when {
            gamepad1.aWasPressed() -> intake.isRunning = true
            gamepad1.aWasReleased() -> intake.isRunning = false
            gamepad1.bWasPressed() -> intake.isOuttake = true
            gamepad1.bWasReleased() -> intake.isOuttake = false
        }
        dashTelemetry.addData("Intake state", intake.stateFlow.value)
        dashTelemetry.update()
    }
}