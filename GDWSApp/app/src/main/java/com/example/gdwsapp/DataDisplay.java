package com.example.gdwsapp;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.telephony.SmsManager;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

// Firebase stuff
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.math.BigDecimal;
import java.math.MathContext;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class DataDisplay extends AppCompatActivity {

    final int SEND_SMS_PERMISSION_REQUEST_CODE = 1;

    // Add elements as properties
    TextView mTimeTextView;
    TextView mTempTextView;
    TextView mXTextView;
    TextView mYTextView;
    TextView mZTextView;
    TextView mGarageStateTextView;
    Button mPreviousButton;

    // Gets reference to the root of Firebase JSON tree
    DatabaseReference mRootRef = FirebaseDatabase.getInstance().getReference();

    // Gets references for accelerometer data
    DatabaseReference mTimeRef = mRootRef.child("ADXL362").child("Time");
    DatabaseReference mTempRef = mRootRef.child("ADXL362").child("Temp");
    DatabaseReference mXRef = mRootRef.child("ADXL362").child("XAng");
    DatabaseReference mYRef = mRootRef.child("ADXL362").child("YAng");
    DatabaseReference mZRef = mRootRef.child("ADXL362").child("ZAng");

    // Get garage state reference
    DatabaseReference mGarageStateRef = mRootRef.child("ADXL362").child("GarageState");

    String mobileNumber;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Get layout from activity_main.xml
        setContentView(R.layout.activity_data_display);

        // Get UI elements
        mTimeTextView = findViewById(R.id.textViewTime);
        mTempTextView = findViewById(R.id.textViewTemp);
        mXTextView = findViewById(R.id.textViewX);
        mYTextView = findViewById(R.id.textViewY);
        mZTextView = findViewById(R.id.textViewZ);
        mGarageStateTextView = findViewById(R.id.textViewGarageState);
        mPreviousButton = findViewById(R.id.previousButton);

        // Get mobile phone number from main activity
        Intent getMobile = getIntent();
        mobileNumber = getMobile.getStringExtra(MainActivity.EXTRA_MOBILE);

        // Check to see if SMS permissions are set
        if(!checkPermission(Manifest.permission.SEND_SMS)) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.SEND_SMS}, SEND_SMS_PERMISSION_REQUEST_CODE);
        }

        // Check to see if user wants to return to main (pressed button)
        mPreviousButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                goToMainActivity();
            }
        });

    }

    @Override
    protected void onStart() {
        super.onStart();

        /* Get epoch time value from Firebase and update display */
        mTimeRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double timeVal = dataSnapshot.getValue(double.class);
                // Create date object for time in milliseconds since Jan. 1, 1970 00:00:00 GMT.
                // Also known as epoch time
                Date date = new Date((long)timeVal);
                // Create format for the date and time
                DateFormat human_readable = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
                // Format the timestamp in human-readable time
                String readable_time = human_readable.format(date);
                mTimeTextView.setText(readable_time); // Update display
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        /* Get temperature value from Firebase and update display */
        mTempRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double tempVal = dataSnapshot.getValue(double.class);
                double tempAbs = Math.abs(tempVal); // Get temperature magnitude
                BigDecimal tempRound = new BigDecimal(tempVal); // Use big decimal to round
                // Round temperature to two decimal places
                if(tempAbs > 9.999) tempRound = tempRound.round(new MathContext(4));
                if(tempAbs < 10.0 && tempAbs > 0.0) tempRound = tempRound.round(new MathContext(3));
                if(tempAbs < 1.0 && tempAbs > 0.0) tempRound = tempRound.round(new MathContext(2));
                String temperature = tempRound + " \u2109"; // Add Units (degrees Fahrenheit)
                mTempTextView.setText(temperature); // Update display
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        /* Get X axis angle from Firebase and update display */
        mXRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double xVal = dataSnapshot.getValue(double.class);
                double xAbs = Math.abs(xVal); // Get angle magnitude
                BigDecimal xRound = new BigDecimal(xVal); // Use Big Decimal to round
                // Round angle value to two decimal places
                if(xAbs > 9.999) xRound = xRound.round(new MathContext(4));
                if(xAbs < 10.0 && xAbs > 0.0) xRound = xRound.round(new MathContext(3));
                if(xAbs < 1.0 && xAbs > 0.0) xRound = xRound.round(new MathContext(2));
                String X = xRound + " \u00B0"; // Add units (degree symbol)
                mXTextView.setText(X); // Update text field
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        /* Get Y axis angle from Firebase and update display */
        mYRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double yVal = dataSnapshot.getValue(double.class);
                double yAbs = Math.abs(yVal); // Get angle magnitude
                BigDecimal yRound = new BigDecimal(yVal); // Use Big Decimal to round
                // Round angle value to two decimal places
                if(yAbs > 9.999) yRound = yRound.round(new MathContext(4));
                if(yAbs < 10.0 && yAbs > 0.0) yRound = yRound.round(new MathContext(3));
                if(yAbs < 1.0 && yAbs > 0.0) yRound = yRound.round(new MathContext(2));
                String Y = yRound + " \u00B0"; // Add units (degree symbol)
                mYTextView.setText(Y); // Update text field
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        /* Get Z axis angle from Firebase and update display */
        mZRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double zVal = dataSnapshot.getValue(double.class);
                double zAbs = Math.abs(zVal); // Get angle magnitude
                BigDecimal zRound = new BigDecimal(zVal); // Use Big Decimal to round
                // Round angle value to two decimal places
                if(zAbs > 9.999) zRound = zRound.round(new MathContext(4));
                if(zAbs < 10.0 && zAbs > 0.0) zRound = zRound.round(new MathContext(3));
                if(zAbs < 1.0 && zAbs > 0.0) zRound = zRound.round(new MathContext(2));
                String Z = zRound + " \u00B0"; // Add units (degree symbol)
                mZTextView.setText(Z); // Update text field
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        /* Get Garage State from Firebase and update display */
        mGarageStateRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double state = dataSnapshot.getValue(double.class);
                String GarageState;
                // Display appropriate message depending on Garage State
                if(state == 0.0) GarageState = mobileNumber;
                else {
                    GarageState = "Garage Door is Open!";
                    SmsManager smgr = SmsManager.getDefault();
                    smgr.sendTextMessage(mobileNumber, null, GarageState,
                            null, null);

                }
                mGarageStateTextView.setText(GarageState); // Update display
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
    }

    // Add button to return to main page to re-enter mobile number
    protected void goToMainActivity() {
        Intent view_main = new Intent(this, MainActivity.class);
        startActivity(view_main);
    }

    // Check SMS permissions
    public boolean checkPermission(String permission) {
        int check = ContextCompat.checkSelfPermission(this, permission);
        return (check == PackageManager.PERMISSION_GRANTED);
    }

    // Start timer for notification
    public void startTimer() {

    }

    // Stop timer for notification
    public void stopTimer() {
        
    }
}