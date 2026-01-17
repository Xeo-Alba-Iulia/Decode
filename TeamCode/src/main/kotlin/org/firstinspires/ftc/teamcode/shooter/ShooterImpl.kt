package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.*
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.sorter.Sorter

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
        var MIN_LAUNCH_VELOCITY = 2200.0

        @JvmField
        var coefficients = PIDCoefficients(0.015, kD = 0.0004)

        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.06, kV = 0.000005)
    }


    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0..80.0)
            rotationServo.position = 0.5 - field / 160.0
            RobotLog.dd("Shooter angle degrees", field.toString())
        }

    override var hood by hoodServo::position
    override var velocity = MIN_LAUNCH_VELOCITY

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.5, 0.0, false))

    override fun shoot() =
        opModeScope.launch {
            motor.power = 1.0
            try {
                tickFlow.collect {
                    val currentVelocity = encoder.velocity
//                    controller.goal = KineticState(velocity = velocity)
//                    motor.power =
//                        controller.calculate(KineticState(encoder.currentPosition.toDouble(), currentVelocity))
                    stateFlow.value =
                        Shooter.State(hood, currentVelocity, currentVelocity >= MIN_LAUNCH_VELOCITY)
                }
            } finally {
                motor.power = 0.0
                stateFlow.value = Shooter.State(hood, 0.0, false)
            }
        }
}

private val shootCountMutex = Mutex()

suspend fun shootCount(
    shootFlow: Flow<Shooter.State>,
    sorter: Sorter,
    shooterJob: Job? = null,
    count: Int = sorter.size
) = shootCountMutex.withLock {
    if (count <= 0) return@withLock
    require(count <= sorter.size)
    sorter.prepareShoot()
    var alreadyShot = 0
    shootFlow
        .dropWhile { (_, _, canShoot) -> !canShoot }
        .distinctUntilChanged { state1, state2 -> state1.canShoot == state2.canShoot }
        .filter { !it.canShoot }
        .take(count)
        .collect {
            if (++alreadyShot == count) return@collect
            sorter.prepareShoot()
        }
    sorter.prepareIntake()
    shooterJob?.cancel()
}
