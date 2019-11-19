package com.example.gdwsapp;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {

    private EditText mMobileNumber;
    Button mButtonMobile;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get layout from activity_main.xml
        setContentView(R.layout.activity_main);

        // Get UI elements
        mMobileNumber = findViewById(R.id.textMobile);
        mButtonMobile = findViewById(R.id.mobileButton);

        mButtonMobile.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                goToDataDisplay();
            }
        });
    }

    // Method to view data display page
    protected void goToDataDisplay() {
        Intent intent = new Intent(this, DataDisplay.class);
        startActivity(intent);
    }
}
