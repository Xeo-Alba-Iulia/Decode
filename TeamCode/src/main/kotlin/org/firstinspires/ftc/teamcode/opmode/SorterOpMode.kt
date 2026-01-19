package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.sorter.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Systems")
open class SorterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var sorter: Sorter

    override fun init() {
        telemetry = opModeGraph.telemetry
        sorter = opModeGraph.sorter
        observers += sorter
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
                gamepad1.aWasPressed() -> true
                gamepad1.aWasReleased() -> false
                else -> null
            }?.let { sorter.isLifting = it }

            sorter.position += (gamepad1.right_trigger - gamepad1.left_trigger).toDouble() * 0.001

            telemetry.addData("Runtime", runtime)
            telemetry.addData("Sorter", sorter)
            telemetry.update()
        }
    }
}