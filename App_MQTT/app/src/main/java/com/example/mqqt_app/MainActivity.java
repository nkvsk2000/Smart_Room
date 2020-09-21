package com.example.mqqt_app;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.NotificationCompat;
import androidx.core.app.NotificationManagerCompat;
import androidx.core.content.ContextCompat;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;


import com.cepheuen.elegantnumberbutton.view.ElegantNumberButton;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import java.io.UnsupportedEncodingException;

public class MainActivity extends AppCompatActivity {
    mqtthelper mqttHelper;
    mqtthelper_ultrasonic mqttHelper_ultra;
    mqtthelper_light mqttHelper_light;
    mqtthelper_air mqttHelper_air;
    mqtthelper_notif mqttHelper_notif;
    mqtthelper_webcam mqttHelper_webcam;
    mqtthelper_read mqttHelper_read;

    TextView temp;
    TextView ultra;
    TextView light;
    TextView air;
    TextView webcam;

    Switch label_switch_fan;
    Switch label_switch_light;
    Switch label_switch_ac;

    Button btn_fan;
    Button btn_light;
    Button btn_ac;

    MqttAndroidClient client;

    private NotificationManagerCompat notificationManager;

    public static final String CHANNEL_1_ID = "channel1";
    String message = "";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        temp = findViewById(R.id.txt_temp);
        ultra = findViewById(R.id.txt_ultra);
        light = findViewById(R.id.txt_light);
        air = findViewById(R.id.txt_air);
        webcam = findViewById(R.id.txt_webcam);

        btn_fan = findViewById(R.id.push_button);
        btn_ac = findViewById(R.id.push_button_ac);
        btn_light = findViewById(R.id.push_button_light);

        label_switch_fan = findViewById(R.id.switch_fan);
        label_switch_light = findViewById(R.id.switch_light);
        label_switch_ac = findViewById(R.id.switch_ac);

        btn_fan.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
        btn_light.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
        btn_ac.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
        connect_to_publish();


        label_switch_fan.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {

                String topic = "topic/switch";
                String payload;
                if (b) {
                    payload = "10";
                } else {
                    payload = "00";
                }

                byte[] encodedPayload = new byte[0];
                try {
                    encodedPayload = payload.getBytes("UTF-8");
                    MqttMessage message = new MqttMessage(encodedPayload);
                    client.publish(topic, message);
                } catch (UnsupportedEncodingException | MqttException e) {
                    e.printStackTrace();
                }

            }
        });

        label_switch_light.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                String topic = "topic/switch";
                String payload;
                if (b) {
                    payload = "11";
                } else {
                    payload = "01";
                }

                byte[] encodedPayload = new byte[0];
                try {
                    encodedPayload = payload.getBytes("UTF-8");
                    MqttMessage message = new MqttMessage(encodedPayload);
                    client.publish(topic, message);
                } catch (UnsupportedEncodingException | MqttException e) {
                    e.printStackTrace();
                }
            }
        });

        label_switch_ac.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                String topic = "topic/switch";
                String payload;
                if (b) {
                    //label_switch_ac.setTrackTintList(@col);
                    payload = "12";
                } else {
                    payload = "02";
                }

                byte[] encodedPayload = new byte[0];
                try {
                    encodedPayload = payload.getBytes("UTF-8");
                    MqttMessage message = new MqttMessage(encodedPayload);
                    client.publish(topic, message);
                } catch (UnsupportedEncodingException | MqttException e) {
                    e.printStackTrace();
                }
            }
        });


        startMqtt_notif();
        createNotificationChannels();

        notificationManager = NotificationManagerCompat.from(this);

        startMqtt_air();
        startMqtt_temp();
        startMqtt_ultra();
        startMqtt_light();
        startMqtt_webcam();
        startMqtt_read();

    }




    private void connect_to_publish() {

        String clientId = MqttClient.generateClientId();
        client = new MqttAndroidClient(this.getApplicationContext(), "tcp://172.16.116.133:1883",
                        clientId);

        try {
            IMqttToken token = client.connect();
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    // We are connected
                    Log.d("Success", "onSuccess");
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    // Something went wrong e.g. connection timeout or firewall problems
                    Log.d("Failure", "onFailure");

                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }

    }

    private void createNotificationChannels() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationChannel channel1 = new NotificationChannel(
                    CHANNEL_1_ID,
                    "Channel 1",
                    NotificationManager.IMPORTANCE_HIGH
            );
            channel1.setDescription("This is Channel 1");

            NotificationManager manager = getSystemService(NotificationManager.class);
            manager.createNotificationChannel(channel1);
        }
    }

    private void notification_Call() {

        String title = "NOTIFICATION";
//        message = "ncsjkdnvlsdk";
        Notification notification = new NotificationCompat.Builder(this, CHANNEL_1_ID)
                .setSmallIcon(R.drawable.ic_launcher_background)
                .setContentTitle(title)
                .setContentText(message)
                .setPriority(NotificationCompat.PRIORITY_HIGH)
                .setCategory(NotificationCompat.CATEGORY_MESSAGE)
                .build();

        notificationManager.notify(1, notification);
    }





    private void startMqtt_read() {
        mqttHelper_read = new mqtthelper_read(getApplicationContext());
        mqttHelper_read.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String temp_complete = mqttMessage.toString();
                if(temp_complete.charAt(0) == '1'){
                    btn_fan.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimaryDark));
                }else{
                    btn_fan.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
                }

                if(temp_complete.charAt(1) == '1'){
                    btn_light.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimaryDark));
                }else{
                    btn_light.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
                }

                if(temp_complete.charAt(2) == '1'){
                    btn_ac.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimaryDark));
                }else{
                    btn_ac.setBackgroundTintList(ContextCompat.getColorStateList(getApplicationContext(), R.color.colorPrimary));
                }

            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

    private void startMqtt_webcam() {
        mqttHelper_webcam = new mqtthelper_webcam(getApplicationContext());
        mqttHelper_webcam.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String temp_complete = mqttMessage.toString();
                webcam.setText("Number of People : " + temp_complete);
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });

    }

    private void startMqtt_notif() {

        mqttHelper_notif = new mqtthelper_notif(getApplicationContext());
        mqttHelper_notif.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String temp_complete = mqttMessage.toString();
                message = temp_complete;

                notification_Call();
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

    private void startMqtt_air() {
        mqttHelper_air = new mqtthelper_air(getApplicationContext());
        mqttHelper_air.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String temp_complete = mqttMessage.toString();
                String[] parts = temp_complete.split(",");
                String part2 = parts[1];

                String a = part2.substring(0, part2.length() - 1);
                air.setText(a);
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

    private void startMqtt_temp() {
        mqttHelper = new mqtthelper(getApplicationContext());
        mqttHelper.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {

                String temp_complete = mqttMessage.toString();
                //temp.setText(temp_complete);

                String[] parts = temp_complete.split(",");
                String part2 = parts[1]; // humidity
                String part3 = parts[2]; //temperature

                String humidity = part2.substring(9, part2.length());
                String temperature = part3.substring(13, part3.length() - 1);


                temp.setText("Temp: " + temperature + "\u2103 Hum:" + humidity);
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

    private void startMqtt_ultra() {
        mqttHelper_ultra = new mqtthelper_ultrasonic(getApplicationContext());
        mqttHelper_ultra.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String ultra_complete = mqttMessage.toString();
                String a = ultra_complete.substring(20, ultra_complete.length() - 3);

                Double distance_door = Double.parseDouble(a);

                if (distance_door > 30)
                    ultra.setText("Door Status : Open");
                else
                    ultra.setText("Door Status : Closed");
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

    private void startMqtt_light() {
        mqttHelper_light = new mqtthelper_light(getApplicationContext());
        mqttHelper_light.mqttAndroidClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean b, String s) {
                Log.w("Debug", "Connected");
            }

            @Override
            public void connectionLost(Throwable throwable) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage mqttMessage) throws Exception {
                String temp_complete = mqttMessage.toString();
                String[] parts = temp_complete.split("V");
                String part1 = parts[0]; // Lux

                String lux = part1.substring(5, part1.length() - 2);


                light.setText("Brightness : " + lux);
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {

            }
        });
    }

}
