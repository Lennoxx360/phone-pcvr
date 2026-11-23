/* (C)2023 */
package io.github.lennoxx360.phonepcvr;

public interface BatteryLevelListener {
    void onBatteryLevelChanged(float batteryPercentage, boolean plugged);
}
