package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap

class Pendul(hardwareMap: HardwareMap) {
    private val servoList = listOf(hardwareMap.servo["pendulServo1"], hardwareMap.servo["pendulServo2"])
    private var currentPosition = 0.0
    fun getPosition() = currentPosition

    fun setPosition(position: Double) {
        currentPosition = position.coerceIn(0.2, 0.8)
        for (servo in servoList) {
            servo.position = currentPosition
        }
    }
}