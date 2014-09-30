package eu.sathra.mavlink;

import eu.sathra.mavlink.MAV.Attitude;

public interface MAVListener {
	void onAttitudeReceived(Attitude attitude); 
}
