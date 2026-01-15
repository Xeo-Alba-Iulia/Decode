package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.abs

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    private val tickFlow: SharedFlow<Unit>,
) : Shooter {

    companion object {
        @JvmField
        @Volatile
        var TOLERANCE = 20.0

        @JvmField
        var VELOCITY = 500.0

        @JvmField
        var coefficients = PIDFCoefficients()
    }

    init {
        motor.setPIDFCoefficients(
            DcMotor.RunMode.RUN_USING_ENCODER,
            coefficients
        )
    }

    override var angleDegrees by rotationServo::position
    override var hood by hoodServo::position

    override var velocity = VELOCITY

    override fun shoot(): Flow<Shooter.State> {
        motor.power = 1.0
        return flow {
            try {
                tickFlow.collect {
                    val currentVelocity = motor.velocity
                    emit(Shooter.State(hood, currentVelocity, abs(currentVelocity - velocity) <= TOLERANCE))
                }
            } finally {
                motor.power = 0.0
            }
        }
    }
}

suspend fun shootCount(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    count: Int = sorter.size
) {
    require(count <= sorter.size)
    sorter.prepareShoot()
    shootFlow
        .distinctUntilChanged { state1, state2 -> state1.canShoot == state2.canShoot }
        .onEach {
            if (!it.canShoot) {
                RobotLog.ii("Shooter", "State after shooting is: $it")
                if (sorter.isLifting)
                    sorter.prepareShoot()
                sorter.isLifting = false
            }
        }
        .filter { it.canShoot }
        .take(count).collect {
            sorter.isLifting = true
            RobotLog.ii("Shooter", "Shot item with state: $it")
        }
}
