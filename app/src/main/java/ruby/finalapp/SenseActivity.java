package ruby.finalapp;

import android.app.ActivityManager;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.support.wearable.activity.WearableActivity;
import android.support.wearable.view.BoxInsetLayout;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class SenseActivity extends WearableActivity {
    private static final String TAG = "SenseActivity";
    private Button mBtnView;
    private Button mBtnView2;
    private Button mBtnView3;
    private TextView stepCount;
    private BoxInsetLayout mContainerView;




    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.sense_layout);
        setAmbientEnabled();


        mContainerView = (BoxInsetLayout) findViewById(R.id.container);
        mBtnView = (Button) findViewById(R.id.btn);
        mBtnView2 = (Button) findViewById(R.id.btn2);
        mBtnView3 = (Button) findViewById(R.id.btn3);
        mBtnView3.setVisibility(View.INVISIBLE);
        mBtnView2.setVisibility(View.INVISIBLE);
        stepCount = (TextView) findViewById(R.id.sc);
        stepCount.setVisibility(View.INVISIBLE);

        registerReceiver(receiver, filter);


    }

    IntentFilter filter = new IntentFilter("ruby.finalapp.testIntent");
    BroadcastReceiver receiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            int steps =  intent.getExtras().getInt("count");
            stepCount.setText(String.valueOf(steps));
            stepCount.setVisibility(View.VISIBLE);
            mBtnView3.setVisibility(View.VISIBLE);
        }
    };

    public void onStartClick(View view) {

        mBtnView2.setVisibility(View.VISIBLE);
        ActivityManager manager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
        for (ActivityManager.RunningServiceInfo service : manager.getRunningServices(Integer.MAX_VALUE)) {
            if ("ruby.finalapp.WearableService"
                    .equals(service.service.getClassName())) {
                Log.i(TAG, "Service already running!");
            }
        }



        //Send the intent with the user chosen values
        Intent toservice = new Intent(this, WearableService.class);
        this.startService(toservice);
        //Make Start button invisible
        mBtnView.setVisibility(View.INVISIBLE);

    }

    public void onStopClick(View view) {
        Intent intent = new Intent("ruby.finalapp.testIntent");
        intent.putExtra("value", 1);
        sendBroadcast(intent);
        mBtnView2.setVisibility(View.INVISIBLE);
    }

    //End the application
    public void onFinishClick(View view) {
        this.finishAffinity();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    public void onDestroy() {
        Log.d(TAG, "onDestroy");
        this.unregisterReceiver(receiver);
        super.onDestroy();
    }

}
