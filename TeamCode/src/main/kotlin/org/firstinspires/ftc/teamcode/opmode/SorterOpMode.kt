package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.OpModeObserver
import org.firstinspires.ftc.teamcode.sorter.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Tests")
class SorterOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var sorter: Sorter

    override fun init() {
        telemetry = opModeGraph.telemetry
        sorter = opModeGraph.sorter
        (sorter as? OpModeObserver)?.let { addObserver(it) }
        telemetry.isAutoClear = false
    }

    override fun start() {
        telemetry.addData("Sorter", ::sorter)
        super.start()
    }

    override fun loop() {
        runBlocking {
            when {
                gamepad1.dpadUpWasPressed() -> sorter.intake(ArtefactType.PURPLE)
                gamepad1.dpadDownWasPressed() -> sorter.intake(ArtefactType.GREEN)
            }

            when {
                gamepad1.leftBumperWasPressed() -> sorter.shoot(ArtefactType.PURPLE)
                gamepad1.rightBumperWasPressed() -> sorter.shoot(ArtefactType.GREEN)
                gamepad1.backWasPressed() -> sorter.shoot()
            }

            telemetry.update()
        }
    }
}