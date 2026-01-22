package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.buffer
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.ArtefactType
import org.firstinspires.ftc.teamcode.intake.*
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Systems")
open class SorterOpMode : CoroutineOpMode() {
    lateinit var sorter: Sorter
    lateinit var intake: Intake

    override fun init() {
        telemetry = opModeGraph.telemetry
        sorter = opModeGraph.sorter
        intake = opModeGraph.intake
        observers += sorter

        intake.artefactFlow
            .zipWithNext()
            .buffer(capacity = 2)
            .onEach { (previous, _) ->
                if (previous != null) {
                    delay(150L)
                    sorter.intake(previous)
                }
            }
            .launchIn(opModeScope)


        intake.colorFlow
            .onEach {
                val (alpha, red, green, blue) = it
                telemetry.addData("Alpha", alpha)
                telemetry.addData("Red", red)
                telemetry.addData("Green", green)
                telemetry.addData("Blue", blue)
            }
            .launchIn(opModeScope)
    }

    override fun loop() {
        runBlocking {
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
                gamepad1.crossWasPressed() -> intake.isRunning = !intake.isRunning
                gamepad1.circleWasPressed() -> intake.isOuttake = true
                gamepad1.circleWasReleased() -> intake.isOuttake = false
            }

            when {
                gamepad1.squareWasPressed() -> true
                gamepad1.squareWasReleased() -> false
                else -> null
            }?.let { sorter.isLifting = it }

            sorter.position += (gamepad1.right_trigger - gamepad1.left_trigger).toDouble() * 0.001

            val (alpha, red, green, blue) = intake.colorFlow.value
            telemetry.addData("Alpha", alpha)
            telemetry.addData("Red", red)
            telemetry.addData("Green", green)
            telemetry.addData("Blue", blue)
            telemetry.addData("Sorter", sorter)
            telemetry.update()
        }
    }
}