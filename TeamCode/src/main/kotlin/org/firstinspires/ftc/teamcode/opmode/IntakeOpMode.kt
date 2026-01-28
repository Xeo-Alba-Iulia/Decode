package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.sorter.Sorter

@TeleOp(group = "Systems")
open class IntakeOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var intake: Intake
    lateinit var sorter: Sorter

    override fun init() {
        telemetry = opModeGraph.telemetry
        intake = opModeGraph.intake
//        sorter = opModeGraph.sorter.also {
//            opModeScope.launch { it.prepareIntake() }
//        }
        intake.stateFlow
            .onEach {
                telemetry.addData("Sensor", it)
            }
            .launchIn(opModeScope)

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
            gamepad1.bWasPressed() -> intake.isRunning = false
        }
//        telemetry.addData("Sorter", sorter)
        telemetry.update()
    }
}