package com.example.gdwsapp;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.TextView;

import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.*;
import java.text.*;

import static java.lang.Math.abs;

public class MainActivity extends AppCompatActivity {

    TextView mTimeTextView;
    TextView mTempTextView;
    TextView mXTextView;
    TextView mYTextView;
    TextView mZTextView;
    TextView mGarageStateTextView;

    // Gets reference to the root of Firebase JSON tree
    DatabaseReference mRootRef = FirebaseDatabase.getInstance().getReference();
    DatabaseReference mTimeRef = mRootRef.child("ADXL362").child("Time");
    DatabaseReference mTempRef = mRootRef.child("ADXL362").child("Temp");
    DatabaseReference mXRef = mRootRef.child("ADXL362").child("XAng");
    DatabaseReference mYRef = mRootRef.child("ADXL362").child("YAng");
    DatabaseReference mZRef = mRootRef.child("ADXL362").child("ZAng");
    DatabaseReference mGarageStateRef = mRootRef.child("ADXL362").child("GarageState");

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Get UI elements
        mTimeTextView = findViewById(R.id.textViewTime);
        mTempTextView = findViewById(R.id.textViewTemp);
        mXTextView = findViewById(R.id.textViewX);
        mYTextView = findViewById(R.id.textViewY);
        mZTextView = findViewById(R.id.textViewZ);
        mGarageStateTextView = findViewById(R.id.textViewGarageState);

    }

    @Override
    protected void onStart() {
        super.onStart();

        mTimeRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double timeVal = dataSnapshot.getValue(double.class);
                Date date = new Date((long)timeVal);
                DateFormat format = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
                String formatted = format.format(date);
                mTimeTextView.setText(formatted);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });

        mTempRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double tempVal = dataSnapshot.getValue(double.class);
                String temperature = tempVal + " \u2109";
                mTempTextView.setText(temperature);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mXRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double xVal = dataSnapshot.getValue(double.class);
                String X = xVal + " \u00B0";
                mXTextView.setText(X);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mYRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double yVal = dataSnapshot.getValue(double.class);
                double yAbs = Math.abs(yVal);
                BigDecimal yRound = new BigDecimal(yVal);
                if(yAbs > 9.999) yRound = yRound.round(new MathContext(4));
                if(yAbs < 10.0 && yAbs > 0.0) yRound = yRound.round(new MathContext(3));
                if(yAbs < 1.0 && yAbs > 0.0) yRound = yRound.round(new MathContext(2));
                String Y = yRound + " \u00B0";
                mYTextView.setText(Y);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mZRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double zVal = dataSnapshot.getValue(double.class);
                BigDecimal zRound = new BigDecimal(zVal);
                zRound = zRound.round(new MathContext(4));
                String Z = zRound + " \u00B0";
                mZTextView.setText(Z);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mGarageStateRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double state = dataSnapshot.getValue(double.class);
                String GarageState;
                if(state == 0.0) GarageState = "Garage Door is Closed!";
                else GarageState = "Garage Door is Open!";
                mGarageStateTextView.setText(GarageState);
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
    }
}
