package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
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
        observers += elevator
    }

    override fun start() {
        super.start()
        opModeScope.launch {
            while (true) {
                elevator.setHold()
                while (gamepad1.crossWasPressed())
                    delay(100L)
                elevator.lift().join()
            }
        }
    }

    override fun loop() {}
}