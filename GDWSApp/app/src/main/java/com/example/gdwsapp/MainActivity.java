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
import java.util.*;
import java.text.*;

public class MainActivity extends AppCompatActivity {

    TextView mTimeTextView;
    TextView mTempTextView;
    TextView mXTextView;
    TextView mYTextView;
    TextView mZTextView;

    // Gets reference to the root of Firebase JSON tree
    DatabaseReference mRootRef = FirebaseDatabase.getInstance().getReference();
    DatabaseReference mTimeRef = mRootRef.child("ADXL362").child("Time");
    DatabaseReference mTempRef = mRootRef.child("ADXL362").child("Temp");
    DatabaseReference mXRef = mRootRef.child("ADXL362").child("XAng");
    DatabaseReference mYRef = mRootRef.child("ADXL362").child("YAng");
    DatabaseReference mZRef = mRootRef.child("ADXL362").child("ZAng");

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
                mTempTextView.setText(String.valueOf(tempVal));
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mXRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double xVal = dataSnapshot.getValue(double.class);
                mXTextView.setText(String.valueOf(xVal));
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mYRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double yVal = dataSnapshot.getValue(double.class);
                mYTextView.setText(String.valueOf(yVal));
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
        mZRef.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                double zVal = dataSnapshot.getValue(double.class);
                mZTextView.setText(String.valueOf(zVal));
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
    }
}
