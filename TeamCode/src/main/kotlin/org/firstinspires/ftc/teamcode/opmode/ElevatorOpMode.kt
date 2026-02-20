package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.elevator.Elevator

@TeleOp(group = "Systems")
class ElevatorOpMode : CoroutineOpMode() {
    lateinit var elevator: Elevator

    override fun init() {
        elevator = opModeGraph.elevator
        telemetry = opModeGraph.telemetry
        observers += elevator
    }

    override fun loop() {
        when {
            gamepad1.dpad_up -> elevator.goUp()
            gamepad1.dpad_down -> elevator.goDown()
        }

        elevator.position += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.01
        telemetry.addData("Position", elevator.position)
    }
}