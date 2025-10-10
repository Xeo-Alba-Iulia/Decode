package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.cos
import kotlin.math.sin

@TeleOp
class BasicOpMode : OpMode() {
    companion object {
        const val IS_FIELD_CENTRIC = true
    }

    lateinit var drive: Drive
    lateinit var odometry: SensorOdometry

    override fun init() {
        drive = Drive(hardwareMap)
        odometry = SensorOdometry(hardwareMap)
    }

    override fun loop() {
        var axial = -gamepad1.left_stick_y.toDouble()
        var lateral = gamepad1.left_stick_x.toDouble()
        val yaw = gamepad1.right_stick_x.toDouble()

        odometry.update()
        val pose = odometry.pose
        telemetry.addData("Pose", pose)
        telemetry.update()

        if (IS_FIELD_CENTRIC) {
            val heading = pose.getHeading(AngleUnit.RADIANS)
            val temp = axial * cos(heading) - lateral * sin(heading)
            lateral = axial * sin(heading) + lateral * cos(heading)
            axial = temp
        }

        drive.drive(axial, lateral, yaw)
    }
}