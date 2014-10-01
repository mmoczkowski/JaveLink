package eu.sathra.mavlink;

import eu.sathra.mavlink.MAV.Attitude;
import eu.sathra.mavlink.MAV.BatteryStatus;
import eu.sathra.mavlink.MAV.SensorStatus;

public interface MAVListener {
	void onAttitudeChanged(Attitude previous, Attitude current);

	void onSensorStatusChanged(SensorStatus previous, SensorStatus current);

	void onBatteryStatusChanged(BatteryStatus previous, BatteryStatus current);

	void onStabilizationModeChanged(boolean enabled);

	void onManualModeChanged(boolean enabled);

	void onGuidedModeChanged(boolean enabled);

	void onAutoModeChanged(boolean enabled);

	void onTestModeChanged(boolean enabled);

	void onSystemStatusChanged(int previous, int current);
}
