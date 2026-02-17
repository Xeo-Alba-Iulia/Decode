package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.constrainedDouble

@TeleOp(group = "Systems")
@Disabled
class ElevatorOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry

    private var targetPosition by constrainedDouble(0.0..1300.0, 0.0)

    override fun init() {
        telemetry = opModeGraph.telemetry
    }

    override fun loop() {
        when {
            gamepad1.dpadUpWasPressed() -> targetPosition += 50.0
            gamepad1.dpadDownWasPressed() -> targetPosition -= 50.0
        }
        // Elevator removed from graph.
    }
}