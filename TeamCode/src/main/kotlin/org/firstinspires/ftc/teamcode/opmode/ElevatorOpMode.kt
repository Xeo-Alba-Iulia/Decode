package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(group = "Systems")
class ElevatorOpMode : OpMode() {
    //    lateinit var elevator: Elevator
    lateinit var motors: List<DcMotor>

    override fun init() {
        motors = listOf(hardwareMap.dcMotor["encoder"], hardwareMap.dcMotor["lift right"])
    }

    override fun loop() {
        val power = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
        motors.forEach { it.power = power }
    }
}