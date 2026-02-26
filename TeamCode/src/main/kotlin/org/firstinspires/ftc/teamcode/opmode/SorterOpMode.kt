package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Systems")
open class SorterOpMode : CoroutineOpMode() {
    lateinit var sorter: Sorter
    lateinit var intake: Intake
    lateinit var dashTelemetry: Telemetry

    override fun init() {
        dashTelemetry = FtcDashboard.getInstance().telemetry
        sorter = opModeGraph.sorter
        intake = opModeGraph.intake.apply { isDebug = true }
        observers += sorter

        intake.artefactFlow
            .onEach {
                Log.d("SorterOpMode", "Detected $it")
                Log.d("SorterOpMode", "stateFlow is: ${intake.stateFlow.value}")
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
            gamepad1.dpadUpWasPressed() -> sorter.intake(ArtefactType.PURPLE)
            gamepad1.dpadDownWasPressed() -> sorter.intake(ArtefactType.GREEN)
        }

        when {
            gamepad1.leftBumperWasPressed() -> sorter.prepareShoot(ArtefactType.PURPLE)
            gamepad1.rightBumperWasPressed() -> sorter.prepareShoot(ArtefactType.GREEN)
            gamepad1.backWasPressed() -> sorter.prepareShoot()
        }

        if (gamepad1.startWasPressed())
            sorter.prepareIntake()

        when {
            sorter.isFull -> intake.isRunning = false
            gamepad1.crossWasPressed() -> intake.isRunning = !intake.isRunning
            gamepad1.circleWasPressed() -> intake.isOuttake = true
            gamepad1.circleWasReleased() -> intake.isOuttake = false
        }

        when {
            gamepad1.squareWasPressed() -> sorter.isLifting = true
            gamepad1.squareWasReleased() -> sorter.isLifting = false
        }

        sorter.position += (gamepad1.right_trigger - gamepad1.left_trigger).toDouble() * 0.001
    }

    override fun stop() {
        super.stop()
        RobotLog.dd("SorterOpMode", "Sorter: $sorter")
    }
}