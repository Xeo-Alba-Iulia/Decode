package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class BasicOpMode : OpMode() {
    private lateinit var pendulServo1: Servo
    private lateinit var pendulServo2: Servo

    override fun init() {
        telemetry.addLine("Hello, FTC!")
        telemetry.update()
        pendulServo1 = hardwareMap.servo["pendulServo1"]
        pendulServo2 = hardwareMap.servo["pendulServo2"]
    }

    override fun loop() {
        telemetry.addLine("Running...")
        telemetry.update()
        pendulServo1.position = 0.5
        pendulServo2.position = 0.5
    }
}