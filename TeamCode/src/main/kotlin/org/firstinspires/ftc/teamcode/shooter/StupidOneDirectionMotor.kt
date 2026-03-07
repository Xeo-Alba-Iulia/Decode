package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.min

/**
 * Literally a class that deals with the bad motor port on the expansion hub
 */
@JvmInline
value class StupidOneDirectionMotor(private val motor: DcMotorEx) : DcMotorEx by motor {
    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        assert(motor.portNumber == 3)
    }

    override fun setDirection(direction: DcMotorSimple.Direction) =
        throw UnsupportedOperationException("Attempted to switch direction on motor from bad port")

    override fun getPower() = min(motor.power, 0.0)
    override fun setPower(power: Double) {
        motor.power = power.takeIf { power > 0.0 } ?: -0.1
    }
}