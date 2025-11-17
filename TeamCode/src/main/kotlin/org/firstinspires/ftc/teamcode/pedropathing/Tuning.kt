package org.firstinspires.ftc.teamcode.pedropathing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.config.ValueProvider
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.telemetry.SelectScope
import com.pedropathing.telemetry.SelectableOpMode
import com.pedropathing.util.PoseHistory
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.Inject
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.changes
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.draw
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.drawOnlyCurrent
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.follower
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.stopRobot
import org.firstinspires.ftc.teamcode.pedropathing.Tuning.Companion.telemetryA
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.pow


/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Config
@TeleOp(name = "Tuning", group = "Pedro Pathing")
class Tuning : SelectableOpMode("Select a Tuning OpMode", { s: SelectScope<Supplier<OpMode?>?>? ->
    s!!.folder("Localization") { l ->
        l.add("Localization Test") { localizationOpMode.value }
        l.add("Forward Tuner", ::ForwardTuner)
        l.add("Lateral Tuner", ::LateralTuner)
        l.add("Turn Tuner", ::TurnTuner)
    }
    s.folder("Automatic") { a ->
        a.add("Forward Velocity Tuner", Supplier { ForwardVelocityTuner() })
        a.add("Lateral Velocity Tuner", Supplier { LateralVelocityTuner() })
        a.add("Forward Zero Power Acceleration Tuner", Supplier { ForwardZeroPowerAccelerationTuner() })
        a.add("Lateral Zero Power Acceleration Tuner", Supplier { LateralZeroPowerAccelerationTuner() })
    }
    s.folder("Manual") { p ->
        p.add("Translational Tuner", Supplier { TranslationalTuner() })
        p.add("Heading Tuner", Supplier { HeadingTuner() })
        p.add("Drive Tuner", Supplier { DriveTuner() })
        p.add("Centripetal Tuner", Supplier { CentripetalTuner() })
    }
    s.folder("Tests") { p ->
        p.add("Line") { Line() }
        p.add("Triangle", Supplier { Triangle() })
        p.add("Circle", Supplier { Circle() })
    }
}) {
    val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    init {
        appGraph.inject(this)
    }

    @Inject
    lateinit var followerLazy: Lazy<Follower>

    @Inject
    lateinit var telemetryLazy: Lazy<Telemetry>

    @Inject
    private lateinit var localizationOpMode: Lazy<LocalizationTest>

    @Suppress("RedundantCompanionReference")
    public override fun onSelect() {
        follower = followerLazy.value
        Companion.localizationOpMode = localizationOpMode

        follower.setStartingPose(Pose())

        poseHistory = follower.poseHistory

        telemetryA = telemetryLazy.value

        FtcDashboard.getInstance().updateConfig()
    }

    public override fun onLog(lines: List<String>) {}

    companion object {
        lateinit var follower: Follower

        lateinit var poseHistory: PoseHistory

        lateinit var telemetryA: Telemetry

        lateinit var localizationOpMode: Lazy<OpMode>

        var changes = ArrayList<String>()

        val isInitialized get() = ::follower.isInitialized

        fun drawOnlyCurrent() {
            try {
                drawRobot(follower.pose)
                sendPacket()
            } catch (e: Exception) {
                throw RuntimeException("Drawing failed $e")
            }
        }

        fun draw() {
            drawDebug(follower)
        }

        /** This creates a full stop of the robot by setting the drive motors to run at 0 power.  */
        fun stopRobot() {
            follower.startTeleopDrive(true)
            follower.setTeleOpDrive(0.0, 0.0, 0.0)
        }
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@Inject
internal class LocalizationTest(
    private val follower: Lazy<Follower>,
    @Suppress("PROPERTY_HIDES_JAVA_FIELD") private val telemetry: Telemetry
) : OpMode() {
    private var multiplier = 1.0

    override fun init() {}

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.  */
    override fun init_loop() {
        telemetry.addLine(
            "This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1."
        )
        telemetry.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower.value.startTeleopDrive()
        FtcDashboard.getInstance().run {
            addConfigVariable("Localization", "MULTIPLIER", object : ValueProvider<Double> {
                override fun get() = multiplier
                override fun set(value: Double) {
                    multiplier = value
                }
            })
            updateConfig()
        }
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    override fun loop() {
        follower.value.setTeleOpDrive(
            -gamepad1.left_stick_y.toDouble() * multiplier,
            -gamepad1.left_stick_x.toDouble() * multiplier,
            -gamepad1.right_stick_x.toDouble(),
        )
        follower.value.update()

        telemetry.addData("Pose", follower.value.pose)
        telemetry.addData("Total Heading", follower.value.totalHeading)
        telemetry.update()

        draw()
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@Inject
internal class ForwardTuner : OpMode() {
    override fun init() {
        follower.update()
        drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Pull your robot forward $DISTANCE inches. Your forward ticks to inches will be shown on the telemetry.")
        telemetryA.update()
        drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower!!.update()

        telemetryA.addData("Distance Moved", follower!!.getPose().getX())
        telemetryA.addLine("The multiplier will display what your forward ticks to inches should be to scale your current distance to $DISTANCE inches.")
        telemetryA.addData(
            "Multiplier", (DISTANCE / (follower!!.getPose().getX() / follower!!.getPoseTracker().getLocalizer()
                .getForwardMultiplier()))
        )
        telemetryA.update()

        draw()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
@Inject
internal class LateralTuner : OpMode() {
    override fun init() {
        follower!!.update()
        drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Pull your robot to the right $DISTANCE inches. Your strafe ticks to inches will be shown on the telemetry.")
        telemetryA.update()
        drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower!!.update()

        telemetryA.addData("Distance Moved", follower!!.getPose().getY())
        telemetryA.addLine("The multiplier will display what your strafe ticks to inches should be to scale your current distance to $DISTANCE inches.")
        telemetryA.addData(
            "Multiplier", (DISTANCE / (follower!!.getPose().getY() / follower!!.getPoseTracker().getLocalizer()
                .getLateralMultiplier()))
        )
        telemetryA.update()

        draw()
    }

    companion object {
        var DISTANCE: Double = 48.0
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
@Inject
internal class TurnTuner : OpMode() {
    override fun init() {
        follower!!.update()
        drawOnlyCurrent()
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("Turn your robot $ANGLE radians. Your turn ticks to inches will be shown on the telemetry.")
        telemetryA.update()

        drawOnlyCurrent()
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    override fun loop() {
        follower!!.update()

        telemetryA.addData("Total Angle", follower!!.totalHeading)
        telemetryA.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to $ANGLE radians.")
        telemetryA.addData(
            "Multiplier", (ANGLE / (follower!!.getTotalHeading() / follower!!.getPoseTracker().getLocalizer()
                .getTurningMultiplier()))
        )
        telemetryA.update()

        draw()
    }

    companion object {
        var ANGLE: Double = 2 * Math.PI
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Inject
internal class ForwardVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.")
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.")
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.")
        telemetryA.addLine("Press B on game pad 1 to stop.")
        telemetryA.addData("pose", follower!!.getPose())
        telemetryA.update()

        follower!!.update()
        drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        follower!!.startTeleopDrive(true)
        follower!!.update()
        end = false
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot()
            requestOpModeStop()
        }

        follower!!.update()
        draw()


        if (!end) {
            if (Math.abs(follower!!.getPose().getX()) > DISTANCE) {
                end = true
                stopRobot()
            } else {
                follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
                //double currentVelocity = Math.abs(follower!!.getVelocity().getXComponent());
                val currentVelocity: Double = Math.abs(follower!!.poseTracker.getLocalizer().getVelocity().getX())
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()
            telemetryA.addLine("Forward Velocity: " + average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Forward Velocity temporarily (while robot remains on).")

            for (i in velocities.indices) {
                telemetry.addData(i.toString(), velocities.get(i))
            }

            telemetryA.update()
            telemetry.update()

            if (gamepad1.aWasPressed()) {
                follower!!.setXVelocity(average)
                val message = "XMovement: " + average
                changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot right at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Inject
internal class LateralVelocityTuner : OpMode() {
    private val velocities = ArrayList<Double?>()

    private var end = false

    override fun init() {}

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * telemetryM.
     */
    override fun init_loop() {
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.")
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.")
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.")
        telemetryA.addLine("Press B on Gamepad 1 to stop.")
        telemetryA.update()

        follower!!.update()
        drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run right at full power.  */
    override fun start() {
        var i = 0
        while (i < RECORD_NUMBER) {
            velocities.add(0.0)
            i++
        }
        follower!!.startTeleopDrive(true)
        follower!!.update()
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot()
            requestOpModeStop()
        }

        follower!!.update()
        draw()

        if (!end) {
            if (Math.abs(follower!!.getPose().getY()) > DISTANCE) {
                end = true
                stopRobot()
            } else {
                follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
                val currentVelocity: Double = Math.abs(follower!!.getVelocity().dot(Vector(1.0, Math.PI / 2)))
                velocities.add(currentVelocity)
                velocities.removeAt(0)
            }
        } else {
            stopRobot()
            var average = 0.0
            for (velocity in velocities) {
                average += velocity!!
            }
            average /= velocities.size.toDouble()

            telemetryA.addLine("Strafe Velocity: " + average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Lateral Velocity temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower!!.setYVelocity(average)
                val message = "YMovement: " + average
                changes.add(message)
            }
        }
    }

    companion object {
        var DISTANCE: Double = 48.0
        var RECORD_NUMBER: Double = 10.0
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@Inject
internal class ForwardZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0

    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetryM.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run forward until it reaches " + VELOCITY + " inches per second.")
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.")
        telemetryA.addLine("Press B on Gamepad 1 to stop.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        follower!!.startTeleopDrive(false)
        follower!!.update()
        follower!!.setTeleOpDrive(1.0, 0.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot()
            requestOpModeStop()
        }

        follower!!.update()
        draw()

        val heading = Vector(1.0, follower!!.getPose().getHeading())
        if (!end) {
            if (!stopping) {
                if (follower!!.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = follower!!.getVelocity().dot(heading)
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = follower!!.getVelocity().dot(heading)
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < follower!!.getConstraints().getVelocityConstraint()) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            telemetryA.addLine("Forward Zero Power Acceleration (Deceleration): " + average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower!!.getConstants().setForwardZeroPowerAcceleration(average)
                val message = "Forward Zero Power Acceleration: " + average
                changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the right until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
@Inject
internal class LateralZeroPowerAccelerationTuner : OpMode() {
    private val accelerations = ArrayList<Double?>()
    private var previousVelocity = 0.0
    private var previousTimeNano: Long = 0
    private var stopping = false
    private var end = false

    override fun init() {}

    /** This initializes the drive motors as well as the Panels telemetry.  */
    override fun init_loop() {
        telemetryA.addLine("The robot will run to the right until it reaches " + VELOCITY + " inches per second.")
        telemetryA.addLine("Then, it will cut power from the drivetrain and roll to a stop.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.addLine("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.")
        telemetryA.addLine("Press B on game pad 1 to stop.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power.  */
    override fun start() {
        follower!!.startTeleopDrive(false)
        follower!!.update()
        follower!!.setTeleOpDrive(0.0, 1.0, 0.0, true)
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    override fun loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot()
            requestOpModeStop()
        }

        follower!!.update()
        draw()

        val heading = Vector(1.0, follower!!.getPose().getHeading() - Math.PI / 2)
        if (!end) {
            if (!stopping) {
                if (Math.abs(follower!!.getVelocity().dot(heading)) > VELOCITY) {
                    previousVelocity = Math.abs(follower!!.getVelocity().dot(heading))
                    previousTimeNano = System.nanoTime()
                    stopping = true
                    follower!!.setTeleOpDrive(0.0, 0.0, 0.0, true)
                }
            } else {
                val currentVelocity: Double = Math.abs(follower!!.getVelocity().dot(heading))
                accelerations.add(
                    (currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / 10.0.pow(
                        9.0
                    ))
                )
                previousVelocity = currentVelocity
                previousTimeNano = System.nanoTime()
                if (currentVelocity < follower!!.getConstraints().getVelocityConstraint()) {
                    end = true
                }
            }
        } else {
            var average = 0.0
            for (acceleration in accelerations) {
                average += acceleration!!
            }
            average /= accelerations.size.toDouble()

            telemetryA.addLine("Lateral Zero Power Acceleration (Deceleration): " + average)
            telemetryA.addLine("\n")
            telemetryA.addLine("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).")
            telemetryA.update()

            if (gamepad1.aWasPressed()) {
                follower!!.getConstants().setLateralZeroPowerAcceleration(average)
                val message = "Lateral Zero Power Acceleration: " + average
                changes.add(message)
            }
        }
    }

    companion object {
        var VELOCITY: Double = 30.0
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Inject
internal class TranslationalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        telemetryA.addLine("This will activate the translational PIDF(s)")
        telemetryA.addLine("The robot will try to stay in place while you push it laterally.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's translational PIDF(s).")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower!!.deactivateAllPIDFs()
        follower!!.activateTranslational()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        follower!!.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        follower!!.update()
        draw()

        if (!follower!!.isBusy()) {
            if (forward) {
                forward = false
                follower!!.followPath(backwards)
            } else {
                forward = true
                follower!!.followPath(forwards)
            }
        }

        telemetryA.addLine("Push the robot laterally to test the Translational PIDF(s).")
        telemetryA.update()
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class HeadingTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will activate the heading PIDF(s).")
        telemetryA.addLine("The robot will try to stay at a constant heading while you try to turn it.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's heading PIDF(s).")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower!!.deactivateAllPIDFs()
        follower!!.activateHeading()
        forwards = Path(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
        forwards!!.setConstantHeadingInterpolation(0.0)
        backwards = Path(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
        backwards!!.setConstantHeadingInterpolation(0.0)
        follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower!!.update()
        draw()

        if (!follower!!.isBusy()) {
            if (forward) {
                forward = false
                follower!!.followPath(backwards)
            } else {
                forward = true
                follower!!.followPath(forwards)
            }
        }

        telemetryA.addLine("Turn the robot manually to test the Heading PIDF(s).")
        telemetryA.update()
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class DriveTuner : OpMode() {
    private var forward = true

    private var forwards: PathChain? = null
    private var backwards: PathChain? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE + "inches forward.")
        telemetryA.addLine("The robot will go forward and backward continuously along the path.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower!!.deactivateAllPIDFs()
        follower!!.activateDrive()

        forwards = follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(0.0, 0.0), Pose(DISTANCE, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        backwards = follower!!.pathBuilder()
            .setGlobalDeceleration()
            .addPath(BezierLine(Pose(DISTANCE, 0.0), Pose(0.0, 0.0)))
            .setConstantHeadingInterpolation(0.0)
            .build()

        follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower!!.update()
        draw()

        if (!follower!!.isBusy()) {
            if (forward) {
                forward = false
                follower!!.followPath(backwards)
            } else {
                forward = true
                follower!!.followPath(forwards)
            }
        }

        telemetryA.addLine("Driving forward?: " + forward)
        telemetryA.update()
    }

    companion object {
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Line : OpMode() {
    private var forward = true

    private val forwards =
        Path(BezierLine(Pose(), Pose(DISTANCE, 0.0))).also { it.setConstantHeadingInterpolation(0.0) }
    private val backwards =
        Path(BezierLine(Pose(DISTANCE, 0.0), Pose())).also { it.setConstantHeadingInterpolation(0.0) }

    override fun init() {}

    /** This initializes the Follower and creates the forward and backward Paths.  */
    override fun init_loop() {
        telemetryA.addLine("This will activate all the PIDF(s)")
        telemetryA.addLine("The robot will go forward and backward continuously along the path while correcting.")
        telemetryA.addLine("You can adjust the PIDF values to tune the robot's drive PIDF(s).")
        telemetryA.update()
        follower.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower.activateAllPIDFs()
        follower.followPath(forwards)
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry  */
    override fun loop() {
        follower.update()
        draw()

        if (!follower.isBusy()) {
            if (forward) {
                forward = false
                follower.followPath(backwards)
            } else {
                forward = true
                follower.followPath(forwards)
            }
        }

        telemetryA.addLine("Driving Forward?: " + forward)
        telemetryA.update()
    }

    companion object {
        @JvmField
        var DISTANCE: Double = 40.0
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
internal class CentripetalTuner : OpMode() {
    private var forward = true

    private var forwards: Path? = null
    private var backwards: Path? = null

    override fun init() {}

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    override fun init_loop() {
        telemetryA.addLine("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.")
        telemetryA.addLine("The robot will go continuously along the path.")
        telemetryA.addLine("Make sure you have enough room.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    override fun start() {
        follower!!.activateAllPIDFs()
        forwards = Path(BezierCurve(Pose(), Pose(abs(DISTANCE), 0.0), Pose(abs(DISTANCE), DISTANCE)))
        backwards = Path(BezierCurve(Pose(abs(DISTANCE), DISTANCE), Pose(abs(DISTANCE), 0.0), Pose(0.0, 0.0)))

        backwards!!.setTangentHeadingInterpolation()
        backwards!!.reverseHeadingInterpolation()

        follower!!.followPath(forwards)
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower!!.update()
        draw()
        if (!follower!!.isBusy()) {
            if (forward) {
                forward = false
                follower!!.followPath(backwards)
            } else {
                forward = true
                follower!!.followPath(forwards)
            }
        }

        telemetryA.addLine("Driving away from the origin along the curve?: " + forward)
        telemetryA.update()
    }

    companion object {
        var DISTANCE: Double = 20.0
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
internal class Triangle : OpMode() {
    private val startPose = Pose(0.0, 0.0, Math.toRadians(0.0))
    private val interPose = Pose(24.0, -24.0, Math.toRadians(90.0))
    private val endPose = Pose(24.0, 24.0, Math.toRadians(45.0))

    private var triangle: PathChain? = null

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    override fun loop() {
        follower!!.update()
        draw()

        if (follower!!.atParametricEnd()) {
            follower!!.followPath(triangle, true)
        }
    }

    override fun init() {}

    override fun init_loop() {
        telemetryA.addLine("This will run in a roughly triangular shape, starting on the bottom-middle point.")
        telemetryA.addLine("So, make sure you have enough space to the left, front, and right to run the OpMode.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    /** Creates the PathChain for the "triangle". */
    override fun start() {
        follower!!.setStartingPose(startPose)

        triangle = follower!!.pathBuilder()
            .addPath(BezierLine(startPose, interPose))
            .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
            .addPath(BezierLine(interPose, endPose))
            .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
            .addPath(BezierLine(endPose, startPose))
            .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
            .build()

        follower!!.followPath(triangle)
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
internal class Circle : OpMode() {
    private var circle: PathChain? = null

    override fun start() {
        circle = follower!!.pathBuilder()
            .addPath(BezierCurve(Pose(0.0, 0.0), Pose(RADIUS, 0.0), Pose(RADIUS, RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(RADIUS, RADIUS), Pose(RADIUS, 2 * RADIUS), Pose(0.0, 2 * RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(0.0, 2 * RADIUS), Pose(-RADIUS, 2 * RADIUS), Pose(-RADIUS, RADIUS)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .addPath(BezierCurve(Pose(-RADIUS, RADIUS), Pose(-RADIUS, 0.0), Pose(0.0, 0.0)))
            .setHeadingInterpolation(HeadingInterpolator.facingPoint(0.0, RADIUS))
            .build()
        follower!!.followPath(circle)
    }

    override fun init_loop() {
        telemetryA.addLine("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ")
        telemetryA.addLine("So, make sure you have enough space to the left, front, and back to run the OpMode.")
        telemetryA.addLine("It will also continuously face the center of the circle to test your heading and centripetal correction.")
        telemetryA.update()
        follower!!.update()
        drawOnlyCurrent()
    }

    override fun init() {}

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        follower!!.update()
        draw()

        if (follower!!.atParametricEnd()) {
            follower!!.followPath(circle)
        }
    }

    companion object {
        var RADIUS: Double = 10.0
    }
}
