package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.elevator.Elevator
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Systems")
class ElevatorOpMode : CoroutineOpMode() {
    @Suppress("PROPERTY_HIDES_JAVA_FIELD")
    lateinit var telemetry: Telemetry
    lateinit var elevator: Elevator
    lateinit var shooter: Shooter
    var liftJob: Job? = null

    override fun init() {
        elevator = opModeGraph.elevator
        telemetry = opModeGraph.telemetry
        shooter = opModeGraph.shooter
        elevator.power = -0.2
    }

    override fun start() {
        val _ = shooter.shoot()
    }

    override fun loop() {}
}