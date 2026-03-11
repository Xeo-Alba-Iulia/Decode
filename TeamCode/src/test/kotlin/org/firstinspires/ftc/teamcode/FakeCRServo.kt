package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import dev.zacsweers.metro.Inject

@Inject
class FakeCRServo : CRServo {
    private var direction = DcMotorSimple.Direction.FORWARD
    private var power = 0.0

    override fun getDirection() = direction
    override fun setDirection(direction: DcMotorSimple.Direction) {
        this.direction = direction
    }
    override fun getPower() = power
    override fun setPower(power: Double) {
        this.power = power
    }

    override fun getController() = null
    override fun getPortNumber() = 0
    override fun getManufacturer() = HardwareDevice.Manufacturer.Unknown
    override fun getDeviceName() = "FakeCRServo"
    override fun getConnectionInfo() = "FakeCRServo Connection"
    override fun getVersion() = 1
    override fun resetDeviceConfigurationForOpMode() {
        direction = DcMotorSimple.Direction.FORWARD
        power = 0.0
    }
    override fun close() {}
}