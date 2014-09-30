package eu.sathra.mavlink.generator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MavMessage {
	
	private int mId;
	private String mName;
	private String mDescription;
	private int mLength;
	private int mCRCExtra;
	
	public int getCRCExtra() {
		return mCRCExtra;
	}

	public void setCRCExtra(int cRCExtra) {
		mCRCExtra = cRCExtra;
	}

	private List<MavField> mFields = new ArrayList<MavField>();
	
	public List<MavField> getFields() {
		return mFields;
	}
	
	public void setFields(List<MavField> field) {
		mFields = field;
	}
	
	public void addField(MavField field) {
		mFields.add(field);
	}
	
	public String getDescription() {
		return mDescription;
	}
	public void setDescription(String description) {
		this.mDescription = description;
	}
	public int getId() {
		return mId;
	}
	public void setId(int id) {
		this.mId = id;
	}
	public String getName() {
		return mName;
	}
	public void setName(String name) {
		this.mName = name;
	}

	public void reorderFields() {
		Collections.sort(mFields);
		
		int currentIndex = 0;
		
		for(MavField field : mFields) {
			field.setIndex(currentIndex);
			currentIndex += field.getSize();
		}
		
		mLength = currentIndex;
	}
	
	public int getLength() {
		return mLength;
	}

}
