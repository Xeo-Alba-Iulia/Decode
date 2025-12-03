package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Shooter")
open class ShooterOpMode : OpMode() {
    lateinit var shooter: Shooter
    lateinit var follower: Follower

    open val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    override fun init() {
        shooter = appGraph.shooter
        shooter.hood = 0.5
        telemetry = appGraph.telemetry
        follower = appGraph.follower
        follower.startTeleopDrive()
    }

    override fun loop() {
        if (gamepad1.aWasPressed()) {
            appGraph.opModeScope.launch { shooter.turnOn() }
        }
        if (gamepad1.bWasPressed()) {
            shooter.turnOff()
        }
        if (gamepad1.y) shooter.shooterSpeed += 0.01
        if (gamepad1.x) shooter.shooterSpeed -= 0.01
        if (gamepad1.dpad_up) {
            shooter.hood += 0.01
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.01
        }
        when {
            gamepad1.dpad_right -> shooter.angleDegrees += 0.01
            gamepad1.dpad_left -> shooter.angleDegrees -= 0.01
        }

        follower.setTeleOpDrive(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
            /* isRobotCentric = */ false
        )
        follower.update()

        this.telemetry.addLine(shooter.toString())
        this.telemetry.update()
    }
}
