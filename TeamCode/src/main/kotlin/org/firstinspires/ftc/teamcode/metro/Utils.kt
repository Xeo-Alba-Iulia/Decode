package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap

inline fun <reified T : HardwareDevice> HardwareMap.getCast(name: String): T = get<T>(T::class.java, name)