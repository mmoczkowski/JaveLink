package eu.sathra.mavlink.generator;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MavField implements Comparable<MavField> {
	
	public static final String TYPE_CHAR = "char";
	public static final String TYPE_INT8 = "int8_t";
	public static final String TYPE_UINT8_MAVLINK_VERSION = "uint8_t_mavlink_version";
	public static final String TYPE_INT16 = "int16_t";
	public static final String TYPE_INT32 = "int32_t";
	public static final String TYPE_INT64 = "int64_t";
	public static final String TYPE_UINT8 = "uint8_t";
	public static final String TYPE_UINT16 = "uint16_t";
	public static final String TYPE_UINT32 = "uint32_t";
	public static final String TYPE_UINT64 = "uint64_t";
	public static final String TYPE_FLOAT = "float";
	public static final String TYPE_DOUBLE = "double";
	
	private static final Pattern CHAR_SIZE_PATTERN = Pattern.compile("\\[(\\d+)\\]");
	
	private String mType;
	private String mName;
	private String mDescription; 
	private int mSize;
	private int mIndex;
	private int mLength = 1;
	private boolean mIsArray = false;

	public boolean isSigned() {
		return !mType.startsWith("u");
	}
	
	public int getLength() {
		return mLength;
	}

	public void setLength(int length) {
		this.mLength = length;
	}

	public boolean isArray() {
		return mIsArray;
	}

	public void setArray(boolean isArray) {
		this.mIsArray = isArray;
	}

	public int getIndex() {
		return mIndex;
	}

	public void setIndex(int index) {
		this.mIndex = index;
	}

	public void setDescription(String description) {
		mDescription = description;
	}
	
	public String getDescription() {
		return mDescription;
	}
	
	public String getType() {
		return mType;
	}
	
	public void setType(String type) {
		mType = type;

		switch(type.replaceAll("\\[\\d+\\]", "")) {
			case TYPE_CHAR:
			case TYPE_UINT8:
			case TYPE_INT8:
			case TYPE_UINT8_MAVLINK_VERSION:
				setSize(1);
			break;
			
			case TYPE_UINT16:
			case TYPE_INT16:
				setSize(2);
			break;
			
			case TYPE_FLOAT:
			case TYPE_INT32:
			case TYPE_UINT32:
				setSize(4);
			break;
				
			case TYPE_DOUBLE:
			case TYPE_INT64:
			case TYPE_UINT64:
				setSize(64);
			break;
		}
		
		if(type.contains("[")) {
			// extract array size
			Matcher myMatcher = CHAR_SIZE_PATTERN.matcher(type);
			myMatcher.find();
			setLength(Integer.parseInt(myMatcher.group(1)));
			setArray(true);
		}
	}
	
	public String getName() {
		return mName;
	}
	public void setName(String name) {
		this.mName = name;
	}
	
	@Override
	public int compareTo(MavField other) {		
		return Integer.compare(other.getSize(), getSize());
	}

	public int getSize() {
		return mSize;
	}

	public void setSize(int size) {
		mSize = size;
	}
	
}
