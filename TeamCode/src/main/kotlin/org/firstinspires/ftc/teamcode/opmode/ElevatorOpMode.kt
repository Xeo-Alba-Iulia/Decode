package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.constrainedDouble
import org.firstinspires.ftc.teamcode.elevator.Elevator
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Systems")
class ElevatorOpMode : CoroutineOpMode() {
    lateinit var elevator: Elevator
    lateinit var shooter: Shooter

    private var targetPosition by constrainedDouble(0.0..1300.0, 0.0)

    override fun init() {
        elevator = opModeGraph.elevator
        telemetry = opModeGraph.telemetry
        shooter = opModeGraph.shooter
    }

    override fun start() {
        shooter.angleDegrees = -90.0
        opModeScope.launch {
            elevator.positionFlow
                .filter { it >= 550.0 }
                .first()
            shooter.angleDegrees = 0.0
        }
    }

    override fun loop() {
        telemetry.addData("Target Position", targetPosition)
        when {
            gamepad1.dpadUpWasPressed() -> targetPosition += 50.0
            gamepad1.dpadDownWasPressed() -> targetPosition -= 50.0
        }
        if (gamepad1.aWasPressed())
            elevator.goUp()
        if (gamepad1.bWasPressed())
            elevator.goPark()
    }
}
