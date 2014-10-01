package eu.sathra.mavlink;

import java.util.HashSet;
import java.util.Set;

import eu.sathra.mavlink.MavLink.MSG_ATTITUDE;
import eu.sathra.mavlink.MavLink.MSG_HEARTBEAT;
import eu.sathra.mavlink.MavLink.MSG_SYS_STATUS;
import eu.sathra.mavlink.MavLink.Message;

/*
 * Class representing Micro Air Vehicle
 */
public class MAV {

	public static class SensorStatus {
		public int id;
		public boolean isPresent;
		public boolean isEnabled;
		public boolean isOperational;

		@Override
		public boolean equals(Object other) {

			if (!(other instanceof SensorStatus))
				return false;

			SensorStatus otherStatus = (SensorStatus) other;

			return otherStatus.isEnabled == isEnabled
					&& otherStatus.isPresent == isPresent
					&& otherStatus.isOperational == isOperational;
		}
	}

	public static class Attitude {
		public float roll; // in radians
		public float pitch; // in radians
		public float yaw; // in radians
		public float rollSpeed; // in rad/sec
		public float pitchSpeed; // in rad/sec
		public float yawSpeed; // in rad/sec

		@Override
		public boolean equals(Object other) {

			if (!(other instanceof Attitude))
				return false;

			Attitude otherAttitude = (Attitude) other;

			return otherAttitude.roll == roll && otherAttitude.pitch == pitch
					&& otherAttitude.yaw == yaw
					&& otherAttitude.rollSpeed == rollSpeed
					&& otherAttitude.pitchSpeed == pitchSpeed
					&& otherAttitude.yawSpeed == yawSpeed;
		}
	}

	public static class BatteryStatus {
		public int voltageBattery = DATA_NOT_AVAILABLE;
		public int currentBattery = DATA_NOT_AVAILABLE;
		public int batteryRemaining = DATA_NOT_AVAILABLE;

		@Override
		public boolean equals(Object other) {

			if (!(other instanceof BatteryStatus))
				return false;

			BatteryStatus otherStatus = (BatteryStatus) other;

			return otherStatus.voltageBattery == voltageBattery
					&& otherStatus.currentBattery == currentBattery
					&& otherStatus.batteryRemaining == batteryRemaining;
		}
	}

	private static final int MAX_SEQUENCE_INDEX = 255;
	private static final int SENSORS_MAX_COUNT = 32;
	private static final int DATA_NOT_AVAILABLE = -1;

	private Set<MAVListener> mListeners = new HashSet<MAVListener>();

	private int mSystemStatus = MavLink.MAV_STATE_POWEROFF;
	private BatteryStatus mBatteryStatus = new BatteryStatus();
	private Attitude mAttitude = new Attitude();

	private int mPacketLossCount;
	private int mSequenceIndex = DATA_NOT_AVAILABLE;

	/*
	 * Bitmasks
	 */
	private int mBaseMode;
	private long mOnboardControlSensorsPresent;
	private long mOnboardControlSensorsEnabled;
	private long mOnboardControlSensorsHealth;

	public void MAV() {

	}

	public void addListener(MAVListener listener) {
		mListeners.add(listener);
	}

	public void removeListener(MAVListener listener) {
		mListeners.remove(listener);
	}

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

	public BatteryStatus getBatteryStatus() {
		return mBatteryStatus;
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

	protected void handleMessage(Message msg) {

		/*
		 * Check for packet loss
		 */
		short sequence = msg.getSequenceIndex();
		int lostPacketCount = sequence - (mSequenceIndex + 1)
				% MAX_SEQUENCE_INDEX;

		if (lostPacketCount > 0 && mSequenceIndex != DATA_NOT_AVAILABLE) {
			mPacketLossCount += lostPacketCount;
		}

		switch (msg.getMessageId()) {
		case MavLink.MSG_ID_ATTITUDE:
			onAttitudeMessage((MSG_ATTITUDE) msg);
			break;

		case MavLink.MSG_ID_SYS_STATUS:
			onSystemStatusMessage((MSG_SYS_STATUS) msg);
			break;

		case MavLink.MSG_ID_HEARTBEAT:
			onHeartbeatMessage((MSG_HEARTBEAT) msg);
			break;
		}

	}

	private void onAttitudeMessage(MavLink.MSG_ATTITUDE msg) {
		Attitude previous = mAttitude;

		mAttitude = new Attitude();
		mAttitude.roll = msg.getRoll();
		mAttitude.pitch = msg.getPitch();
		mAttitude.yaw = msg.getYaw();
		mAttitude.rollSpeed = msg.getRollspeed();
		mAttitude.pitchSpeed = msg.getPitchspeed();
		mAttitude.yawSpeed = msg.getYawspeed();

		if (!mAttitude.equals(previous)) {
			for (MAVListener listener : mListeners) {
				listener.onAttitudeChanged(previous, mAttitude);
			}
		}

	}

	private void onSystemStatusMessage(MavLink.MSG_SYS_STATUS msg) {
		/*
		 * Handle sensors
		 */
		for (int c = 0; c < SENSORS_MAX_COUNT; ++c) {
			int sensorId = 1 << (c + 1);
			SensorStatus previousSensorStatus = getSensorStatus(sensorId);

			mOnboardControlSensorsPresent = msg
					.getOnboard_control_sensors_present();
			mOnboardControlSensorsEnabled = msg
					.getOnboard_control_sensors_enabled();
			mOnboardControlSensorsHealth = msg
					.getOnboard_control_sensors_health();

			SensorStatus currentSensorStatus = getSensorStatus(sensorId);

			/*
			 * If status changed notify listener.
			 */
			if (!currentSensorStatus.equals(previousSensorStatus)) {

				for (MAVListener listener : mListeners) {
					listener.onSensorStatusChanged(previousSensorStatus,
							currentSensorStatus);
				}
			}
		}

		/*
		 * Handle battery
		 */
		BatteryStatus previousBatteryStatus = mBatteryStatus;

		mBatteryStatus = new BatteryStatus();
		mBatteryStatus.batteryRemaining = msg.getBattery_remaining();
		mBatteryStatus.currentBattery = msg.getCurrent_battery();
		mBatteryStatus.voltageBattery = msg.getVoltage_battery();

		if (!mBatteryStatus.equals(previousBatteryStatus)) {
			for (MAVListener listener : mListeners) {
				listener.onBatteryStatusChanged(previousBatteryStatus,
						mBatteryStatus);
			}
		}
	}

	private void onHeartbeatMessage(MavLink.MSG_HEARTBEAT msg) {
		boolean wasStabilizationEnabled = isStabilizationModeEnabled();
		boolean wasManualModeEnabled = isManualModeEnabled();
		boolean wasGuidedModeEnabled = isGuidedModeEnabled();
		boolean wasAutoModeEnabled = isAutoModeEnabled();
		boolean wasTestModeEnabled = isTestModeEnabled();

		mBaseMode = msg.getBase_mode();

		int previousStatus = mSystemStatus;
		mSystemStatus = msg.getSystem_status();

		for (MAVListener listener : mListeners) {
			if (wasStabilizationEnabled != isStabilizationModeEnabled()) {
				listener.onStabilizationModeChanged(isStabilizationModeEnabled());
			}

			if (wasManualModeEnabled != isManualModeEnabled()) {
				listener.onStabilizationModeChanged(isManualModeEnabled());
			}

			if (wasGuidedModeEnabled != isGuidedModeEnabled()) {
				listener.onStabilizationModeChanged(isGuidedModeEnabled());
			}

			if (wasAutoModeEnabled != isAutoModeEnabled()) {
				listener.onStabilizationModeChanged(isAutoModeEnabled());
			}

			if (wasTestModeEnabled != isTestModeEnabled()) {
				listener.onStabilizationModeChanged(isTestModeEnabled());
			}

			if (previousStatus != mSystemStatus) {
				listener.onSystemStatusChanged(previousStatus, mSystemStatus);
			}
		}
	}

	public void sendManualControl(short roll, short pitch, short yaw,
			short thrust) {
		MavLink.MSG_MANUAL_CONTROL message = new MavLink.MSG_MANUAL_CONTROL(
				(short) 0, (short) 0, pitch, roll, thrust, yaw, 0, 0);
	}
}
