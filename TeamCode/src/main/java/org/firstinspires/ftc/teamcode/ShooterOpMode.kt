package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ShooterOpMode : OpMode() {
    lateinit var shooter: Shooter

    override fun init() {
        shooter = Shooter(hardwareMap)
        shooter.hood = 0.5
    }

    override fun loop() {
//        if (gamepad1.a) {
//            shooter.isRunning = true
//        }
//        if (gamepad1.b) {
//            shooter.isRunning = false
//        }
//        if (gamepad1.y) shooter.shooterSpeed += 0.01
//        if (gamepad1.x) shooter.shooterSpeed -= 0.01
        if (gamepad1.dpad_up) {
            shooter.hood += 0.0003
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.0003
        }

        telemetry.addData("Hood", shooter.hood)
        telemetry.addData("Running", shooter.isRunning)
        telemetry.addData("At Target", shooter.isAtTarget)
        telemetry.update()
    }
}
