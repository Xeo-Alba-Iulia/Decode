package org.firstinspires.ftc.teamcode

import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

class ConstrainedDouble(
    val range: ClosedFloatingPointRange<Double>,
    defaultValue: Double = 0.0
) : ReadWriteProperty<Any?, Double> {

    private var value = defaultValue.coerceIn(range)

    override fun getValue(thisRef: Any?, property: KProperty<*>) = value

    override fun setValue(thisRef: Any?, property: KProperty<*>, value: Double) {
        this.value = value.coerceIn(range)
    }
}