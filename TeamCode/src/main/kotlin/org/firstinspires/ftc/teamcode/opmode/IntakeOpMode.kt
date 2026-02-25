package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.intake.Intake

@TeleOp(group = "Systems")
open class IntakeOpMode : CoroutineOpMode() {
    lateinit var intake: Intake
    lateinit var dashTelemetry: Telemetry

    override fun init() {
        dashTelemetry = FtcDashboard.getInstance().telemetry
        intake = opModeGraph.intake.apply { isDebug = true }

        intake.artefactFlow
            .onEach {
                Log.d("SorterOpMode", "Detected $it")
                Log.d("SorterOpMode", "stateFlow is: ${intake.stateFlow}")
            }.launchIn(opModeScope)

        intake.stateFlow
            .onEach { (alpha, red, green, blue, dist) ->
                dashTelemetry.addData("Alpha", alpha)
                dashTelemetry.addData("Red", red)
                dashTelemetry.addData("Green", green)
                dashTelemetry.addData("Blue", blue)
                dashTelemetry.addData("Distance", dist)
                dashTelemetry.update()
            }.launchIn(opModeScope)
    }

    override fun loop() {
        when {
            gamepad1.aWasPressed() -> intake.isRunning = true
            gamepad1.aWasReleased() -> intake.isRunning = false
            gamepad1.bWasPressed() -> intake.isOuttake = true
            gamepad1.bWasReleased() -> intake.isOuttake = false
        }
    }
}