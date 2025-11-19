package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotor
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.teamcode.ConstrainedDouble
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@SingleIn(OpModeScope::class)
@Inject
class ShooterNoDirectionImpl(
    @Named("shootingMotor") private val motor: DcMotor,
    private val shooterNoMotorImpl: ShooterNoMotorImpl,
) : Shooter by shooterNoMotorImpl {

    override var shooterSpeed by ConstrainedDouble(-1.0..1.0, 0.5)

    override var isRunning = false
        set(value) {
            field = value
            motor.power = if (value) shooterSpeed else 0.0
        }

    override fun toString() =
        "ShooterNoDirectionImpl(hood = $hood, shooterSpeed = $shooterSpeed, isRunning = $isRunning)"
}