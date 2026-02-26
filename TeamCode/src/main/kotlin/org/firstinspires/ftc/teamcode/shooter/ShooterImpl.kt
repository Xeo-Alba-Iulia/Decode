package org.firstinspires.ftc.teamcode.shooter

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.ftcrobotcontroller.BuildConfig
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.zacsweers.metro.ContributesBinding
import dev.zacsweers.metro.Named
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.firstinspires.ftc.teamcode.InterpLUT
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import kotlin.math.cos
import kotlin.math.sin

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
        var coefficients = PIDCoefficients(0.0015)

        @JvmField
        var parameters = BasicFeedforwardParameters(kS = 0.05, kV = 0.00033)

        private const val QUEUE_SIZE = 7
    }

    /*
       0.92: 1420 0.0
       1.40: 1500 0.053
       1.67: 1800 0.41
       1.97: 1750 0.35
       2.3:  1750 0.31
       3.01: 2010 0.35
       3.42: 2040 0.332
     */
    val distances = listOf(0.92, 1.4, 1.67, 1.97, 2.3, 3.01, 3.42)
    val velocityLUT: InterpLUT = InterpLUT(
        /* input = */ distances,
        /* output = */ listOf(1580.0, 1680.0, 1780.0, 1850.0, 1960.0, 2160.0, 2280.0),
        /* safeMode = */ true
    ).createLUT()

    val hoodLUT: InterpLUT = InterpLUT(
        listOf(22.5, 24.0, 27.0, 29.0, 31.0, 34.0, 36.5, 39.0, 41.1, 43.0, 45.0).map { it + 6.0 },
        (0..10).map { it / 10.0 },
        true
    ).createLUT()

    override var angleDegrees = 0.0
        set(value) {
            field = value.coerceIn(-80.0, 80.0)
            rotationServo.position = 0.5 - field / 160.0
        }

    var hood by hoodServo::position

    private val controller = controlSystem {
        velPid(coefficients)
        basicFF(parameters)
    }

    @Suppress("RedundantModalityModifier")
    final override val stateFlow: StateFlow<Shooter.State>
        field = MutableStateFlow(Shooter.State(0.0, false))

    private fun setPower(power: Double) = motors.forEach { it.power = power }

    /**
     * Finds the launch angle for a given distance and velocity using Newton's method.
     *
     * @param distance The horizontal distance to the target in meters.
     * @param velocity The launch velocity in meters per second.
     * @param guess An initial guess for the launch angle in degrees.
     *
     * @return The launch angle in degrees that will hit the target at the given distance and velocity.
     */
    private tailrec fun findLaunchAngle(
        distance: Double,
        velocity: Double,
        guess: Double = Math.toRadians(31.0),
        repetitions: Int = 4
    ): Double? {
        val g = 8.9
        val height = 0.65
        val d = distance
        val v = velocity
        val sin = sin(guess)
        val cos = cos(guess)
        val f = d * sin / cos - (d * d * g) / (2 * v * v * cos * cos) - height
        if (repetitions == 0) {
            val guess = guess.takeIf { f >= -0.1 }
            if (BuildConfig.DEBUG) {
                val guessDeg = guess?.let { Math.toDegrees(it) }
                Log.v("Angle", "Found: $guessDeg, distance = $distance, velocity = $velocity, f = $f")
            }
            return guess
        }
        val df = d / (cos * cos) - (d * d * g * sin) / (v * v * cos * cos * cos)
        return findLaunchAngle(distance, velocity, guess - f / df, repetitions - 1)
    }

    private val speedQueue = ArrayDeque(List(QUEUE_SIZE) { 0.0 })
    private var speedSum = speedQueue.sum()

    private fun update(distance: Double) {
        val position = encoder.currentPosition.toDouble()
        val velocity = encoder.velocity
        val desiredVelocity = controller.goal.velocity
        Log.v("ShooterImpl", "Velocity: $velocity, Desired: $desiredVelocity")
        setPower(controller.calculate(KineticState(position, velocity)))
        speedSum += velocity - speedQueue.removeFirst()
        speedQueue.addLast(velocity)
        if (distance == 0.0 || velocity == 0.0)
            return
        stateFlow.value =
            Shooter.State(velocity, findLaunchAngle(distance, ((speedSum / QUEUE_SIZE) / 28) * .083)?.let { result ->
                hood = hoodLUT[Math.toDegrees(result)]
                true
            } ?: false)
    }

    override fun shoot(distanceFlow: Flow<Double>): Job =
        opModeScope.launch {
            try {
                var dist = 0.0
                distanceFlow.onEach { distance ->
                    dist = distance
                    controller.goal = KineticState(velocity = velocityLUT[distance])
                }.launchIn(this + Dispatchers.IO)
                withContext(Dispatchers.Default) {
                    while (true) {
                        update(dist)
                        delay(50L)
                    }
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }

    fun shoot(velocityFn: () -> Double, hoodFn: () -> Double): Job =
        opModeScope.launch {
            try {
                while (true) {
                    controller.goal = KineticState(velocity = velocityFn())
                    hood = hoodFn()
                    update(0.0)
                    delay(50L)
                }
            } finally {
                setPower(0.0)
                stateFlow.value = Shooter.State(0.0, false)
            }
        }
}

