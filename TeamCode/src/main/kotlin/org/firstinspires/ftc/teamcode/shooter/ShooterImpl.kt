package org.firstinspires.ftc.teamcode.shooter

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.RobotLog
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.InterpLUT
import org.firstinspires.ftc.teamcode.metro.OpModeScope

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

    val distances = listOf(0.91, 1.17, 1.48, 1.8, 2.2, 2.8, 3.18, 3.3)
    val hoodLut = InterpLUT(distances, listOf(0.3031, 0.505, .57, .7284, .78, .8858, .95), true).apply {
        createLUT()
    }
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1750.0, 1850.0, 1950.0, 2000.0, 2200.0, 2440.0, 2500.0, 2550.0).map { it + 100.0 },
        /* safeMode = */ true
    ).createLUT()

    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0..80.0)
            rotationServo.position = 0.5 - field / 160.0
            RobotLog.dd("Shooter angle degrees", field.toString())
        }

    override var hood = 0.0

    init {
        hoodServo.position = 0.85
    }
    override var velocity = MIN_LAUNCH_VELOCITY
        set(value) {
            field = value.coerceIn(0.0..3600.0)
            if (stateFlow.value.velocity != 0.0)
                controller.goal = KineticState(velocity = field)
        }

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
        // 3.18: 2500, 0.95
        // 2.9m: 2440, 0.8858
        // 2.2m: 2200, 0.78
        // 1.8m: 2000, 0.7284
        // 1.48: 1950, 0.57
        // 1.17: 1850, 0.505
        // 0.91: 1750, 0.3031
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    override fun shoot(currentDistance: () -> Double): Job =
        opModeScope.launch {
            try {
                tickFlow.collect {
                    val dist = currentDistance()
                    val desiredVelocity = velocityLUT[dist]
                    controller.goal = KineticState(velocity = desiredVelocity)
                    val position = encoder.currentPosition.toDouble()
                    val velocity = encoder.velocity
                    RobotLog.vv("ShooterImpl", "Velocity: $velocity, target = $desiredVelocity")
                    motor.power = controller.calculate(KineticState(position, velocity))
                    val diff = if (stateFlow.value.canShoot)
                        stateFlow.value.velocity - velocity
                    else
                        desiredVelocity - velocity
                    RobotLog.vv("ShooterImpl", "Diff: $diff, lastCanShoot: ${stateFlow.value.canShoot}")
                    val canShoot = diff <= 100.0
                    if (stateFlow.value.canShoot && !canShoot)
                        RobotLog.vv("ShooterImpl", "WARN")
                    stateFlow.value = Shooter.State(velocity, canShoot)
                }
            } finally {
                motor.power = 0.0
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}

