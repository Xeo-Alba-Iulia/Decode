package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.sorter.ArtefactType
import org.firstinspires.ftc.teamcode.sorter.Sorter
import java.util.*
import kotlin.jvm.optionals.getOrNull

@TeleOp(group = "Tests")
class SorterOpMode : OpMode() {
    val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var sorter: Sorter

    override fun init() {
        sorter = appGraph.sorter
        telemetry = appGraph.telemetry
        telemetry.isAutoClear = false
    }

    override fun loop() {
        val successfulIntake = when {
            gamepad1.dpadUpWasPressed() -> sorter.intake(ArtefactType.PURPLE)
            gamepad1.dpadDownWasPressed() -> sorter.intake(ArtefactType.GREEN)
            else -> null
        }
        if (successfulIntake == false) {
            telemetry.clear()
            telemetry.addLine("Intake full")
        }

        when {
            gamepad1.leftBumperWasPressed() -> Optional.of(ArtefactType.PURPLE)
            gamepad1.rightBumperWasPressed() -> Optional.of(ArtefactType.GREEN)
            gamepad1.backWasPressed() -> Optional.empty()
            else -> null
        }?.let {
            if (!sorter.shoot(it.getOrNull())) {
                telemetry.clear()
                telemetry.addLine("Nu mai sunt artefacte ${it.map(ArtefactType::name).orElse("")}")
            }
        }

        telemetry.addLine()
    }
}