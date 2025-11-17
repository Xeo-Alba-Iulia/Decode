package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.sorter.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Tests")
class SorterOpMode : OpMode() {
    val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var sorter: Sorter

    override fun init() {
        telemetry = appGraph.telemetry
        telemetry.isAutoClear = false
    }

    override fun start() {
        sorter = appGraph.sorter
        telemetry.addData("Sorter", ::sorter)
    }

    override fun loop() {
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