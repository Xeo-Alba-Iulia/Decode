package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.sorter.Sorter
import kotlin.math.abs

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooterMotor") private val motor: DcMotorEx,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    @Named("shooterEncoder") private val encoder: DcMotorEx,
    private val opModeScope: CoroutineScope,
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

    override var angleDegrees by rotationServo::position
    override var hood by hoodServo::position

    override var velocity = VELOCITY

    private val controller = controlSystem {
        velPid(0.1, kD = 0.001)
    }

    override fun shoot(): Flow<Shooter.State> {
        val pidJob = opModeScope.launch {
            try {
                tickFlow.collect {
                    controller.goal = KineticState(velocity = velocity)
                    motor.power =
                        controller.calculate(KineticState(encoder.currentPosition.toDouble(), encoder.velocity))
                }
            } finally {
                motor.power = 0.0
            }
        }
        return flow {
            try {
                tickFlow.collect {
                    val currentVelocity = motor.velocity
                    emit(Shooter.State(hood, currentVelocity, abs(currentVelocity - velocity) <= TOLERANCE))
                }
            } finally {
                pidJob.cancel()
            }
        }
    }
}

private val shootCountMutex = Mutex()

suspend fun shootCount(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    count: Int = sorter.size
) = shootCountMutex.withLock {
    if (count <= 0) return@withLock
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
    sorter.isLifting = false
    if (sorter.isEmpty)
        coroutineScope {
            launch {
                sorter.prepareIntake()
            }
        }
}
