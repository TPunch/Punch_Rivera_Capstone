package com.example.gdwsapp;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity {
    public static final String EXTRA_MOBILE = "com.example.gdwsapp.EXTRA_MOBILE";
    EditText mobileNumber;
    Button mButtonMobile;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get layout from activity_main.xml
        setContentView(R.layout.activity_main);

        // Get UI elements

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
        mobileNumber = findViewById(R.id.textMobile);
        String phoneNumber = mobileNumber.getText().toString();
        Intent viewData = new Intent(this, DataDisplay.class);
        viewData.putExtra(EXTRA_MOBILE, phoneNumber);
        startActivity(viewData);
    }
}
