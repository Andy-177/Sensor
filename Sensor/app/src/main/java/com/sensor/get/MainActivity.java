package com.sensor.get;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Switch;
import android.widget.TextView;

public class MainActivity extends Activity implements SensorEventListener {

    // 日志标签
    private static final String TAG_ACC = "传感器-加速度计";
    private static final String TAG_GYRO = "传感器-陀螺仪";
    private static final String TAG_MAG = "传感器-磁力计";
    private static final String TAG_PRESS = "传感器-气压计";
    private static final String TAG_QUAT = "姿态-四元数";
    private static final String TAG_EULER = "姿态-欧拉角";

    // 控件声明
    private TextView tvAccX, tvAccY, tvAccZ;
    private TextView tvGyroX, tvGyroY, tvGyroZ;
    private TextView tvMagX, tvMagY, tvMagZ;
    private TextView tvPressure, tvAltitude;
    private TextView tvQw, tvQx, tvQy, tvQz;
    private TextView tvRoll, tvPitch, tvYaw;

    // 控制开关：【显示原始数据】开关（核心）+ 6个日志开关
    private Switch switchDecimal;
    private Switch switchAcc, switchGyro, switchMag, switchPress, switchQuat, switchEuler;

    // 传感器相关
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private Sensor gyroscope;
    private Sensor magnetometer;
    private Sensor pressure;

    // 姿态解算变量（加入磁力计）
    private float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    private long lastTime = 0;
    private static final float Kp = 0.98f;
    private static final float Ki = 0.02f;
    private float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

    // 磁力计相关（用于修正偏航角）
    private float mx, my, mz;
    private static final float KpMag = 0.05f;  // 磁力计修正系数

    // 临时变量
    private float ax, ay, az;
    private float gx, gy, gz;
    private float roll, pitch, yaw;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // 绑定数据显示控件
        tvAccX = findViewById(R.id.activitymainTextViewAccelerometerX);
        tvAccY = findViewById(R.id.activitymainTextViewAccelerometerY);
        tvAccZ = findViewById(R.id.activitymainTextViewAccelerometerZ);
        tvGyroX = findViewById(R.id.activitymainTextViewGyroX);
        tvGyroY = findViewById(R.id.activitymainTextViewGyroY);
        tvGyroZ = findViewById(R.id.activitymainTextViewGyroZ);
        tvMagX = findViewById(R.id.tvMagX);
        tvMagY = findViewById(R.id.tvMagY);
        tvMagZ = findViewById(R.id.tvMagZ);
        tvPressure = findViewById(R.id.tvPressure);
        tvAltitude = findViewById(R.id.tvAltitude);
        tvQw = findViewById(R.id.tvQuaternionW);
        tvQx = findViewById(R.id.tvQuaternionX);
        tvQy = findViewById(R.id.tvQuaternionY);
        tvQz = findViewById(R.id.tvQuaternionZ);
        tvRoll = findViewById(R.id.tvRoll);
        tvPitch = findViewById(R.id.tvPitch);
        tvYaw = findViewById(R.id.tvYaw);

        // 绑定控制开关
        switchDecimal = findViewById(R.id.switchDecimal);
        switchAcc = findViewById(R.id.switchAcc);
        switchGyro = findViewById(R.id.switchGyro);
        switchMag = findViewById(R.id.switchMag);
        switchPress = findViewById(R.id.switchPress);
        switchQuat = findViewById(R.id.switchQuat);
        switchEuler = findViewById(R.id.switchEuler);

        // 获取传感器
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        pressure = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (accelerometer != null) {
            sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (gyroscope != null) {
            sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (magnetometer != null) {
            sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (pressure != null) {
            sensorManager.registerListener(this, pressure, SensorManager.SENSOR_DELAY_NORMAL);
        }
        lastTime = System.nanoTime();
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            ax = event.values[0];
            ay = event.values[1];
            az = event.values[2];

            tvAccX.setText(formatNumber(ax));
            tvAccY.setText(formatNumber(ay));
            tvAccZ.setText(formatNumber(az));

            if (switchAcc.isChecked()) {
                Log.d(TAG_ACC, "X: " + formatNumber(ax) + " , Y: " + formatNumber(ay) + " , Z: " + formatNumber(az));
            }

        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            gx = event.values[0];
            gy = event.values[1];
            gz = event.values[2];

            tvGyroX.setText(formatNumber(gx));
            tvGyroY.setText(formatNumber(gy));
            tvGyroZ.setText(formatNumber(gz));

            if (switchGyro.isChecked()) {
                Log.d(TAG_GYRO, "X: " + formatNumber(gx) + " , Y: " + formatNumber(gy) + " , Z: " + formatNumber(gz));
            }

            // 姿态解算（陀螺仪数据到达时才更新）
            calculateAttitude();

        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            mx = event.values[0];
            my = event.values[1];
            mz = event.values[2];

            tvMagX.setText(formatNumber(mx));
            tvMagY.setText(formatNumber(my));
            tvMagZ.setText(formatNumber(mz));

            if (switchMag.isChecked()) {
                Log.d(TAG_MAG, "X: " + formatNumber(mx) + " , Y: " + formatNumber(my) + " , Z: " + formatNumber(mz));
            }

        } else if (event.sensor.getType() == Sensor.TYPE_PRESSURE) {
            float pressureValue = event.values[0];
            float altitudeValue = calculateAltitude(pressureValue);

            tvPressure.setText(formatNumber(pressureValue) + " hPa");
            tvAltitude.setText(formatNumber(altitudeValue) + " m");

            if (switchPress.isChecked()) {
                Log.d(TAG_PRESS, "Pressure: " + formatNumber(pressureValue) + " hPa, Altitude: " + formatNumber(altitudeValue) + " m");
            }
        }
    }

    // 计算海拔高度（基于标准大气压）
    private float calculateAltitude(float pressure) {
        // 标准大气压 1013.25 hPa，海拔公式
        return (float) (44330.0 * (1.0 - Math.pow(pressure / 1013.25, 0.1903)));
    }

    // 姿态解算方法（加入磁力计修正）
    private void calculateAttitude() {
        long now = System.nanoTime();
        float dt = (now - lastTime) / 1000000000.0f;
        if (dt > 0.1f) dt = 0.01f; // 限制最大时间间隔
        lastTime = now;

        // 归一化加速度计数据
        float norm = (float) Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0) return;
        float axn = ax / norm;
        float ayn = ay / norm;
        float azn = az / norm;

        // 加速度计计算误差
        float vx = 2 * (q1*q3 - q0*q2);
        float vy = 2 * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        float ex = (ayn * vz - azn * vy);
        float ey = (azn * vx - axn * vz);
        float ez = (axn * vy - ayn * vx);

        // 磁力计修正（如果有效数据）
        if (mx != 0 || my != 0 || mz != 0) {
            // 将磁力计数据旋转到地球坐标系
            float hx = 2 * (mx * (0.5f - q2*q2 - q3*q3) + my * (q1*q2 - q0*q3) + mz * (q1*q3 + q0*q2));
            float hy = 2 * (mx * (q1*q2 + q0*q3) + my * (0.5f - q1*q1 - q3*q3) + mz * (q2*q3 - q0*q1));
            float bx = (float) Math.sqrt(hx*hx + hy*hy);
            float bz = 2 * (mx * (q1*q3 - q0*q2) + my * (q2*q3 + q0*q1) + mz * (0.5f - q1*q1 - q2*q2));

            // 计算磁力计误差
            float wx = 2 * (bx * (0.5f - q2*q2 - q3*q3) + bz * (q1*q3 - q0*q2));
            float wy = 2 * (bx * (q1*q2 - q0*q3) + bz * (q0*q1 + q2*q3));
            float wz = 2 * (bx * (q0*q2 + q1*q3) + bz * (0.5f - q1*q1 - q2*q2));

            // 磁力计误差加入总误差
            ex += (my * wz - mz * wy) * KpMag;
            ey += (mz * wx - mx * wz) * KpMag;
            ez += (mx * wy - my * wx) * KpMag;
        }

        // PI控制器
        exInt += ex * Ki * dt;
        eyInt += ey * Ki * dt;
        ezInt += ez * Ki * dt;

        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;

        // 四元数微分方程
        float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
        float dq1 = 0.5f * (q0*gx + q2*gz - q3*gy) * dt;
        float dq2 = 0.5f * (q0*gy - q1*gz + q3*gx) * dt;
        float dq3 = 0.5f * (q0*gz + q1*gy - q2*gx) * dt;

        q0 += dq0;
        q1 += dq1;
        q2 += dq2;
        q3 += dq3;

        // 归一化四元数
        norm = (float) Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm != 0) {
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }

        // 计算欧拉角
        roll = (float) Math.toDegrees(Math.atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)));
        pitch = (float) Math.toDegrees(Math.asin(2 * (q0*q2 - q1*q3)));
        yaw = (float) Math.toDegrees(Math.atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3)));

        // 显示数据
        tvQw.setText(formatNumber(q0));
        tvQx.setText(formatNumber(q1));
        tvQy.setText(formatNumber(q2));
        tvQz.setText(formatNumber(q3));
        tvRoll.setText(formatNumber(roll));
        tvPitch.setText(formatNumber(pitch));
        tvYaw.setText(formatNumber(yaw));

        // 日志打印
        if (switchQuat.isChecked()) {
            Log.d(TAG_QUAT, "W: " + formatNumber(q0) + " , X: " + formatNumber(q1) + " , Y: " + formatNumber(q2) + " , Z: " + formatNumber(q3));
        }
        if (switchEuler.isChecked()) {
            Log.d(TAG_EULER, "Roll: " + formatNumber(roll) + "° , Pitch: " + formatNumber(pitch) + "° , Yaw: " + formatNumber(yaw) + "°");
        }
    }

    // 格式化显示
    private String formatNumber(float num) {
        if (switchDecimal.isChecked()) {
            return String.valueOf(num);
        } else {
            return String.format("%.2f", num);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
