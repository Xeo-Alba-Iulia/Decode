package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.InterpLUT
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.abs

@Config
@ContributesBinding(OpModeScope::class)
class ShooterImpl(
    @Named("shooter") private val motors: List<DcMotorEx>,
    @Named("shooterHoodServo") private val hoodServo: Servo,
    @Named("shooterRotationServo") private val rotationServo: Servo,
    @Named("shooterEncoder") private val encoder: DcMotorEx,
    private val opModeScope: CoroutineScope,
) : Shooter {

    companion object {
        @JvmField
        var coefficients = PIDCoefficients(0.001)

        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.15, kV = 0.00033)
    }

    /*
       0.92: 1420 0.0
       1.40: 1500 0.053
       1.81: 1620 0.25
       2.25: 1850 0.38
       3.01: 2010 0.35
       3.42: 2040 0.332
     */
    val distances = listOf(0.92, 1.4, 1.81, 2.25, 3.01, 3.42)
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1420.0, 1500.0, 1620.0, 1800.0, 2010.0, 2040.0),
        /* safeMode = */ true
    ).createLUT()

    val hoodLUT: InterpLUT = InterpLUT(
        distances,
        listOf(0.0, 0.053, 0.25, 0.38, 0.35, 0.332),
        true
    ).createLUT()

    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0..80.0)
            rotationServo.position = 0.5 - field / 160.0
        }

    override var hood by hoodServo::position
    override var velocityOffset = 0.0

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    private fun setPower(power: Double) = motors.forEach { it.power = power }

    override fun shoot(currentDistance: () -> Double?): Job =
        opModeScope.launch {
            try {
                while (true) {
                    val curDist = currentDistance()
                    val desiredVelocity = (curDist?.let { velocityLUT[it] } ?: 0.0) + velocityOffset
                    curDist?.let { hood = hoodLUT[it] }
                    controller.goal = KineticState(velocity = desiredVelocity)
                    val position = encoder.currentPosition.toDouble()
                    val velocity = encoder.velocity
                    Log.v("ShooterImpl", "Velocity: $velocity, Desired: $desiredVelocity")
                    setPower(controller.calculate(KineticState(position, velocity)))
                    stateFlow.value = Shooter.State(velocity, abs(velocity - desiredVelocity) <= 80.0)
                    delay(50L)
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}

