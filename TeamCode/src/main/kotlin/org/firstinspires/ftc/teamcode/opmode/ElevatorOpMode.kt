package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constrainedDouble
import org.firstinspires.ftc.teamcode.elevator.Elevator

@TeleOp(group = "Systems")
class ElevatorOpMode : CoroutineOpMode() {
    lateinit var elevator: Elevator

    private var targetPosition by constrainedDouble(0.0..1300.0, 0.0)

    override fun init() {
        elevator = opModeGraph.elevator
        telemetry = opModeGraph.telemetry
    }

    override fun loop() {
        telemetry.addData("Target Position", targetPosition)
        when {
            gamepad1.dpadUpWasPressed() -> targetPosition += 50.0
            gamepad1.dpadDownWasPressed() -> targetPosition -= 50.0
        }
        if (gamepad1.aWasPressed())
            elevator.lift(targetPosition)
    }
}
