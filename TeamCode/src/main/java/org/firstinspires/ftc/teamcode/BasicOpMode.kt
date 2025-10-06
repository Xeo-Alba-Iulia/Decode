package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class BasicOpMode : OpMode() {
    lateinit var pendul: Pendul

    override fun init() {
        telemetry.addLine("Hello, FTC!")
        telemetry.update()
        pendul = Pendul(hardwareMap)
        pendul.setPosition(0.5)
    }

    override fun loop() {
        telemetry.addLine("Running...")
        telemetry.update()
        val delta = gamepad1.right_trigger - gamepad1.left_trigger
        if (delta != 0.0f) {
            pendul.setPosition(pendul.getPosition() + delta * 0.1)
        }
    }
}