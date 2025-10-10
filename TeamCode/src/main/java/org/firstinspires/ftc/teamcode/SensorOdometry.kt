package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

class SensorOdometry(hardwareMap: HardwareMap) {
    val sensor: GoBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

    init {
        sensor.apply {
            setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
            )
            setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            setOffsets(18.0, 0.0, DistanceUnit.CM)
            resetPosAndIMU()
        }
    }

    fun update() = sensor.update()

    var pose: Pose2D by sensor::position
    val heading: Double get() = sensor.getHeading(AngleUnit.DEGREES)
}