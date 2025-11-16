package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import dev.zacsweers.metro.Inject

@Inject
class FakeServo : Servo {
    private var direction = Servo.Direction.FORWARD
    private var position = 0.0

    override fun getController(): ServoController? = null
    override fun getPortNumber(): Int = 0

    override fun getPosition(): Double = position
    override fun setPosition(position: Double) {
        this.position = position
    }

    override fun getDirection(): Servo.Direction = direction
    override fun setDirection(direction: Servo.Direction) {
        this.direction = direction
    }

    override fun scaleRange(min: Double, max: Double) {}

    override fun getManufacturer(): HardwareDevice.Manufacturer = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName(): String = "FakeServo"
    override fun getConnectionInfo(): String = "FakeServoConnection"
    override fun getVersion(): Int = 1
    override fun resetDeviceConfigurationForOpMode() {
        direction = Servo.Direction.FORWARD
    }
    override fun close() {}
}