package com.example.remote_control_app;

import android.util.Log;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;

public class MainActivity extends AppCompatActivity implements Joystick.JoystickListener {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Joystick joystick = new Joystick(this);
        setContentView(R.layout.activity_main);
    }

    @Override
    public void onJoystickMoved(float xPercent, float yPercent, int id) {
        switch (id)
        {
            case R.id.joystickRight:
                Log.d("Right Joystick", "X percent: " + xPercent + " Y percent: " + yPercent);
                break;
            case R.id.joystickLeft:
                Log.d("Left Joystick", "X percent: " + xPercent + " Y percent: " + yPercent);
                break;
        }
    }

}
