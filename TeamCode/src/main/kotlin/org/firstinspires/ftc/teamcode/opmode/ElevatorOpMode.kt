package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.elevator.Elevator

@TeleOp(group = "Systems")
class ElevatorOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var elevator: Elevator
    var liftJob: Job? = null

    override fun init() {
        elevator = opModeGraph.elevator
        telemetry = opModeGraph.telemetry
    }

    override fun loop() {
        when {
            gamepad1.crossWasPressed() && liftJob?.isCancelled != false -> liftJob = elevator.lift()
            gamepad1.circleWasPressed() -> liftJob?.cancel()
        }
    }
}