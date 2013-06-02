package com.mlearning.accelerometer;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class AccelerometerActivity extends Activity implements
		SensorEventListener {
	private SensorManager mSensorManager;
	private Sensor accel;
	private Sensor gyro;
	private float[] linear_acceleration = new float[3];
	private float[] gravity = new float[3];
	private String accelString;
	private static final int normal = SensorManager.SENSOR_DELAY_GAME;
	private static final float NS2S = 1.0f / 1000000000.0f;
	private static final float EPSILON = 0.0001f;;
	private boolean calibrateGyroFlag = false;
	private float[] currGyroValues;
	private long currGyroTimestamp;
	private float[] prevGyroValues;
	private long prevGyroTimestamp;
	private TextView tvaccel;
	private TextView tvgyro;
	private boolean calibrateAccelFlag = false;
	private float[] normInitGravity;
	private float currAccelAngle;
	private Button button;
	private boolean startedFlag = false;
	private float[] currRotationMatrix;
	private Button button2;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
		accel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		gyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		registerListeners();
		setContentView(R.layout.main);
		tvaccel = (TextView) this.findViewById(R.id.tv1);
		tvgyro = (TextView) this.findViewById(R.id.tv2);
		tvaccel.setText("Press Start to Restart");
		tvgyro.setText("Press Start to Restart");
		button = (Button) this.findViewById(R.id.button1);
		button2 = (Button) this.findViewById(R.id.button2);
		button.setText("Start");
		button2.setText("Calibrate");
		button.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub
				startedFlag = !startedFlag;	
				if (startedFlag==false){
					tvaccel.setText("Press Start to Restart");
					tvgyro.setText("Press Start to Restart");
					calibrateAccelFlag = false;
					calibrateGyroFlag = false;
				}
			}

		});

		button2.setOnClickListener(new OnClickListener() {

			@Override
			public void onClick(View v) {
				// TODO Auto-generated method stub
				if (startedFlag) { //Can also mean restarted
					calibrateAccelFlag = true;
					calibrateGyroFlag = true;
					
				}
			}

		});
	}

	public void registerListeners() {
		mSensorManager.registerListener(this, accel, normal);
		mSensorManager.registerListener(this, gyro, normal);
	}

	public void unregisterListeners() {
		mSensorManager.unregisterListener(this, accel);
		mSensorManager.unregisterListener(this, gyro);
	}

	@Override
	public void onPause() {
		super.onPause();
		unregisterListeners();
	}

	public void filterGravityEffects(SensorEvent event) {
		final float alpha = 0.8f;

		gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
		gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
		gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

		// gravity[0]=gravity[1]=gravity[2]=0f; //Comment to measure linear
		// acceleration
		for (int i = 0; i < 3; i++)
			linear_acceleration[i] = gravity[i];
		/*
		 * linear_acceleration[0] = event.values[0] - gravity[0];
		 * linear_acceleration[1] = event.values[1] - gravity[1];
		 * linear_acceleration[2] = event.values[2] - gravity[2];
		 */

	}

	@Override
	public void onResume() {
		super.onResume();
		registerListeners();
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub

	}

	private final float[] deltaRotationVector = new float[4];
	private double filteredAngle;
	private float currGyroAngle;

	@Override
	public void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub

		float velocity = calculateVelocity();
		calculateDistance(velocity);
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {

			Log.v("Gyroscp", "x:" + event.values[0] + "y:" + event.values[1]
					+ "z:" + event.values[2] + "timestamp" + event.timestamp);
			if (startedFlag) {
				// if the gyro is not yet calibrated then calibrate the gyro
				// (condition is calibrategyro will be true)
				if (this.calibrateGyroFlag) {
					if (currGyroValues == null) {
						currGyroValues = new float[3];
					}
					for (int i = 0; i < 3; i++) {
						currGyroValues[i] = event.values[i];
					}
					currGyroTimestamp = event.timestamp;
						currRotationMatrix = new float[] { 1, 0, 0, 0, 1, 0, 0,
								0, 1 };
					calibrateGyroFlag = false;
					currGyroAngle=0;
				} else {
					// Set prev values
					if (prevGyroValues == null) {
						prevGyroValues = new float[3];
					}
					for (int i = 0; i < 3; i++) {
						prevGyroValues[i] = currGyroValues[i];
					}
					prevGyroTimestamp = currGyroTimestamp;

					// set curr values
					for (int i = 0; i < 3; i++) {
						currGyroValues[i] = event.values[i];
					}
					currGyroTimestamp = event.timestamp;
					final float dT = (currGyroTimestamp - prevGyroTimestamp)
							* NS2S;

					// Calculate angle
					// Note:The x angular momentum is positive when you go up
					// and negative when you go down
					// If the angular momentum is greater than the initial
					// angular momentum means u are higher than the baseline
					// point
					// In this case, the angular displacement (angle) is
					// positive, so we need to do final - initial
					// Axis of the rotation sample, not normalized yet.
					float axisX = event.values[0];
					float axisY = event.values[1];
					float axisZ = event.values[2];

					// Calculate the angular speed of the sample
					float omegaMagnitude = (float) Math.sqrt(axisX * axisX
							+ axisY * axisY + axisZ * axisZ);

					// Normalize the rotation vector if it's big enough to get
					// the axis
					if (omegaMagnitude > EPSILON) {
						axisX /= omegaMagnitude;
						axisY /= omegaMagnitude;
						axisZ /= omegaMagnitude;
					}

					// Integrate around this axis with the angular speed by the
					// timestep
					// in order to get a delta rotation from this sample over
					// the timestep
					// We will convert this axis-angle representation of the
					// delta rotation
					// into a quaternion before turning it into the rotation
					// matrix.
					
					//Calculate the new gyroscope angle using the previous measurements and the accelerometer angle reading
					Log.v("timeperiod","dT:"+dT);
					currGyroAngle=(0.961f)*(this.currGyroAngle+event.values[0] * dT)+(0.039f)*currAccelAngle;
					/*
					float thetaOverTwo = omegaMagnitude * dT / 2.0f;
					//float thetaOverTwo = currGyroAngle / 2.0f;
					float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
					float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
					deltaRotationVector[0] = sinThetaOverTwo * axisX;
					deltaRotationVector[1] = sinThetaOverTwo * axisY;
					deltaRotationVector[2] = sinThetaOverTwo * axisZ;
					deltaRotationVector[3] = cosThetaOverTwo;

					float[] deltaRotationMatrix = new float[9];
					SensorManager.getRotationMatrixFromVector(
							deltaRotationMatrix, deltaRotationVector);

					// right matrix multiply since the vector will be right
					// multiplied too
					
					
					currRotationMatrix = matrixMultiply(currRotationMatrix,	deltaRotationMatrix);
					currGyroAngle= (float) Math.acos(currRotationMatrix[4]);
					*/
					//Put the filter somewhere here
					//calculate the new angle
					
					//Put a high pass filter here
					//Put a low pass
					
					this.tvgyro.setText("GyroAngle:" + currGyroAngle*(180/Math.PI) + "X Value:"+event.values[0]);
					
				}
			} else {
				calibrateGyroFlag = true;
			}
		} else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
			// filterGravityEffects(event);
			if (startedFlag) {

				// If the accelerometer is not calibrated => calibrateAccelFlag
				// is true
				if (this.calibrateAccelFlag) {
					/*
					 * if (currAccelValues==null){ currAccelValues = new
					 * float[3]; } for (int i = 0; i < 3; i++) {
					 * currAccelValues[i] = event.values[i]; }
					 * currAccelTimestamp=event.timestamp;
					 */
					// calibrateAccelFlag=true;
					currAccelAngle = 0.0f;
					normInitGravity = normalize(event.values);
					calibrateAccelFlag = false; // /since we have calibrated the
												// accelerometer, set the
												// calibrateflag to false
				} else {
					// normalizedPrevGravity=normalizedCurrGravity;
					float[] normCurrGravity = normalize(event.values);
					float costheta = dotProduct(normInitGravity,
							normCurrGravity);
					currAccelAngle = (float) (Math.acos(costheta));

				}
				// calculateDirectionVectorValue();
				Log.v("Acceleration", "x:" + linear_acceleration[0] + "y:"
						+ linear_acceleration[1] + "z:"
						+ linear_acceleration[2] + "timestamp"
						+ event.timestamp);
				
				accelString = "x:" + event.values[0] + "y:" + event.values[1]
						+ "z:" + event.values[2] + "timestamp"
						+ event.timestamp;
				
				tvaccel.setText("Accelerometer Angle: " + currAccelAngle * (180 / Math.PI) + ":"
						+ accelString);
			}
			else{
				calibrateAccelFlag=true;
			}
		} 
		else
			Log.v("NoSensor", "Paagga Janega");
	}

	private float[] matrixMultiply(float[] m1, float[] m2) {
		// TODO Auto-generated method stub
		float[] temp = new float[9];
		for (int i = 0; i < 9; i = i + 3)
			for (int j = 0; j < 3; j++)
				temp[i + j] = m1[i + 0] * m2[0 + j] + m1[i + 1] * m2[3 + j]
						+ m1[i + 2] * m2[6 + j];
		return temp;
	}

	private float dotProduct(float[] normInitGravity2, float[] normCurrGravity) {
		// TODO Auto-generated method stub
		float sum = 0;
		for (int i = 0; i < 3; i++) {
			sum = sum + normInitGravity2[i] * normCurrGravity[i];
		}
		return sum;
	}

	private float[] normalize(float[] values) {
		float[] temp = new float[3];
		float mag = 0.0f;
		for (int i = 0; i < 3; i++) {
			mag = mag + values[i] * values[i];
		}

		for (int i = 0; i < 3; i++) {
			temp[i] = values[i] / (float) Math.sqrt(mag);
		}
		return temp;
	}

	private float calculateDistance(float velocity) {
		// TODO Auto-generated method stub
		return 0;
	}

	private float calculateVelocity() {
		// TODO Auto-generated method stub
		return 0;
	}
}
