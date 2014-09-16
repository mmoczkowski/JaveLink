package eu.sathra.mavlink.generator;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class MavEnum {

	public static class Value {
		
		private String mName;
		private String mDescription;
		private int mValue;
		
		public String getName() {
			return mName;
		}
		public void setName(String name) {
			this.mName = name;
		}
		public String getDescription() {
			return mDescription;
		}
		public void setDescription(String description) {
			this.mDescription = description;
		}
		public int getValue() {
			return mValue;
		}
		public void setValue(int value) {
			this.mValue = value;
		}
		
	}
	
	private String mName;
	private String mDescription;
	private List<Value> mValues = new ArrayList<Value>();
	
	public String getDescription() {
		return mDescription;
	}
	
	public void setDescription(String description) {
		this.mDescription = description;
	}
	
	public String getName() {
		return mName;
	}
	
	public void setName(String name) {
		this.mName = name;
	}
	
	public List<Value> getValues() {
		return mValues;
	}
	
	public void addValue(Value v) {
		mValues.add(v);
	}
	
}
