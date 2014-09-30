package eu.sathra.mavlink;

import java.util.List;

/*
 * 	private int type;
	private int autopilotType;
	private int customMode;
	private int protocolVersion;
 */

/*
 * Class representing Micro Air Vehicle
 */
public class MAV {

	public static class SensorStatus {
		public int id;
		public boolean isPresent;
		public boolean isEnabled;
		public boolean isOperational;
	}
	
	public static class Attitude {
		private float roll; 		// in radians
		private float pitch; 		// in radians
		private float yaw; 			// in radians
		private float rollSpeed;	// in rad/sec
		private float pitchSpeed;	// in rad/sec
		private float yawSpeed;		// in rad/sec
	}

	private static final int SENSORS_MAX_COUNT = 32;
	private static final int DATA_NOT_AVAILABLE = -1;

	private int mSystemStatus = MavLink.MAV_STATE_POWEROFF;
	private Attitude mAttitude;
	
	/*
	 * Battery
	 */
	private int mVoltageBattery = DATA_NOT_AVAILABLE;
	private int mCurrentBattery = DATA_NOT_AVAILABLE;
	private int mBatteryRemaining = DATA_NOT_AVAILABLE;
	
	/*
	 * Bitmasks
	 */
	private int mBaseMode;
	private int mOnboardControlSensorsPresent;
	private int mOnboardControlSensorsEnabled;
	private int mOnboardControlSensorsHealth;

	public SensorStatus[] getSensorsStatuses() {
		SensorStatus[] statutes = new SensorStatus[SENSORS_MAX_COUNT];

		for (int c = 0; c < SENSORS_MAX_COUNT; ++c) {
			statutes[c] = getSensorStatus(1 << (c + 1));
		}

		return statutes;
	}

	public SensorStatus getSensorStatus(int sensorId) {
		SensorStatus status = new SensorStatus();

		status.id = sensorId;
		status.isPresent = ((mOnboardControlSensorsPresent >> sensorId) & 1) == 1;
		status.isEnabled = ((mOnboardControlSensorsEnabled >> sensorId) & 1) == 1;
		status.isOperational = ((mOnboardControlSensorsHealth >> sensorId) & 1) == 1;
		
		return status;
	}

	public int getVoltageBattery() {
		return mVoltageBattery;
	}

	public void setVoltageBattery(int voltageBattery) {
		mVoltageBattery = voltageBattery;
	}

	public int getCurrentBattery() {
		return mCurrentBattery;
	}

	public void setCurrentBattery(int currentBattery) {
		mCurrentBattery = currentBattery;
	}

	public int getBatteryRemaining() {
		return mBatteryRemaining;
	}

	public void setBatteryRemaining(int batteryRemaining) {
		mBatteryRemaining = batteryRemaining;
	}
	
	public boolean isStabilizationModeEnabled() {
		return (MavLink.MAV_MODE_STABILIZE_ARMED & mBaseMode) == mBaseMode;
	}
	
	public boolean isManualModeEnabled() {
		return (MavLink.MAV_MODE_MANUAL_ARMED & mBaseMode) == mBaseMode;
	}
	
	public boolean isGuidedModeEnabled() {
		return (MavLink.MAV_MODE_GUIDED_ARMED & mBaseMode) == mBaseMode;
	}
	
	public boolean isAutoModeEnabled() {
		return (MavLink.MAV_MODE_AUTO_ARMED & mBaseMode) == mBaseMode;
	}
	
	public boolean isTestModeEnabled() {
		return (MavLink.MAV_MODE_TEST_ARMED & mBaseMode) == mBaseMode;
	}
	
	public int getSystemStatus() {
		return mSystemStatus;
	}
	
	public Attitude getAttitude() {
		return mAttitude;
	}
	
	
}
