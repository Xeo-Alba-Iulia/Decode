package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class Drive(hardwareMap: HardwareMap) {
    val frontLeft = hardwareMap.dcMotor["frontLeft"]
    val frontRight = hardwareMap.dcMotor["frontRight"]
    val backLeft = hardwareMap.dcMotor["backLeft"].apply { direction = DcMotorSimple.Direction.REVERSE }
    val backRight = hardwareMap.dcMotor["backRight"]

    fun drive(axial: Double, lateral: Double, yaw: Double) {
        var frontLeftPower = axial + lateral + yaw
        var frontRightPower = axial - lateral - yaw
        var backLeftPower = axial - lateral + yaw
        var backRightPower = axial + lateral - yaw

        val maxPower = maxOf(
            abs(frontLeftPower),
            abs(frontRightPower),
            abs(backLeftPower),
            abs(backRightPower),
        )

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower
            frontRightPower /= maxPower
            backLeftPower /= maxPower
            backRightPower /= maxPower
        }

        frontLeft.power = frontLeftPower
        frontRight.power = frontRightPower
        backLeft.power = backLeftPower
        backRight.power = backRightPower
    }
}