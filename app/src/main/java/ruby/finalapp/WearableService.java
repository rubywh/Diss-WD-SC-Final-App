package ruby.finalapp;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Binder;
import android.os.Environment;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;
import android.widget.Toast;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import static java.lang.Math.abs;

/**
 * Created by ruby__000 on 13/04/2017.
 */

public class WearableService extends Service implements SensorEventListener {

    private static final String TAG = "WearableService";
    private final IBinder mBinder = new LocalBinder();
    Sensor senAccelerometer;
    ArrayList<String> accArrayList;
    Thread aConsThread;
    private SensorManager senSensorManager;
    private FileWriter accWriter;
    private BufferedWriter aBufferedWriter;
    Intent intent;
    private PowerManager.WakeLock mWakeLock;
    int lastTime = 0;
    int moving = 0;
    BroadcastReceiver receiver;
    Double lastDist = 0.0;
    List<ScanResult> results;
    int size = 0;

    WifiManager wifi;

    @Override
    public void onCreate() {

        super.onCreate();
        // sharedQueue = new LinkedBlockingQueue<>();
        accArrayList = new ArrayList<>();

        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //get accelerometer
        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);


   /*     wifi = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        if (wifi.isWifiEnabled() == false) {
            wifi.setWifiEnabled(true);
        }
        registerReceiver(new BroadcastReceiver() {
            @Override
            public void onReceive(Context c, Intent intent) {
                results = wifi.getScanResults();
                size = results.size();
            }
        }, new IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION));
*/



        IntentFilter rssiFilter = new IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION);
        this.registerReceiver(myRssiChangeReceiver, rssiFilter);

        WifiManager wifiMan=(WifiManager)getSystemService(Context.WIFI_SERVICE);
        wifiMan.startScan();



        //senSensorManager.registerListener(this, senGyro, SensorManager.SENSOR_DELAY_FASTEST);
        Log.d(TAG, "Finished Creation");


       // initializeWiFiListener();

        IntentFilter filter = new IntentFilter("ruby.finalapp.testIntent");
        receiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                Log.i(TAG, "Stop clicked");
                if (intent.getExtras().getInt("value") == 1) {
                    try {
                        stopSensing();
                    } catch (IOException e) {
                        e.printStackTrace();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        registerReceiver(receiver, filter);
    }


    private BroadcastReceiver myRssiChangeReceiver = new BroadcastReceiver(){
        @Override
        public void onReceive(Context arg0, Intent arg1) {
            int thisTime = (int) System.currentTimeMillis() / 1000;

            WifiManager wifiMan=(WifiManager) getSystemService(Context.WIFI_SERVICE);
            wifiMan.startScan();
            int newRssi = wifiMan.getConnectionInfo().getRssi();
            double freq = wifiMan.getConnectionInfo().getFrequency();
            double thisDist = calculateDistance(newRssi, freq)/10;


            if (lastTime == 0.0) {
                lastTime = thisTime;
                lastDist = Double.POSITIVE_INFINITY;
            } else {
                if ((thisTime - lastTime) > 2) {

                    lastTime = thisTime;

                    Log.d(TAG, String.valueOf(thisDist));

                    Log.d(TAG, String.valueOf(lastDist));
                    if ((abs(thisDist - lastDist) > 0 )&& (lastDist!=Double.POSITIVE_INFINITY)) {

                        Log.d(TAG, "2s Elapsed");
                        moving = 1;
                        Log.d(TAG, "Moving");

                        resumeSensing();

                    } else {
                        if (moving == 1) {

                            Log.d(TAG, "stopped moving");
                            pauseSensing();
                            moving = 0;
                        }
                    }
                    lastDist = thisDist;
                }
            }
        }};


    public void stopSensing() throws IOException, InterruptedException {
        if (senSensorManager != null) {
            senSensorManager.unregisterListener(this);
        }
        if (accArrayList.size()!=0) {
            aConsThread = new Thread(new accelerometerConsumer(accArrayList, WearableService.this));
            aConsThread.start();
            Log.d(TAG, "start cons thread called");
            aConsThread.join();
            //sendBroadcast(new Intent("End"));
        } else{
            Intent intent = new Intent("ruby.finalapp.testIntent");
            intent.putExtra("count",0);
            sendBroadcast(intent);
        }
        stopSelf();
        unregisterReceiver(receiver);
        this.unregisterReceiver(myRssiChangeReceiver);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        mWakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "My Tag");
        mWakeLock.acquire();
        Toast.makeText(this, "service starting", Toast.LENGTH_SHORT).show();

//        try {
//            makeFile();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }


        Log.d(TAG, "onStartCommand Finished");

        return Service.START_NOT_STICKY;
    }

/*

    private void initializeWiFiListener() {


        Log.d(TAG, "Initialise Wifi Listener");

        String connectivity_context = Context.WIFI_SERVICE;
        final WifiManager wifi = (WifiManager) getSystemService(connectivity_context);
        if (!wifi.isWifiEnabled()) {
            if (wifi.getWifiState() != WifiManager.WIFI_STATE_ENABLING) {
                wifi.setWifiEnabled(true);
            }
        }
        wifibr = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {

                Log.d(TAG, "Wifi Update");
                int thisTime = (int) System.currentTimeMillis() / 1000;
                WifiInfo info = wifi.getConnectionInfo();
                double currentWifiSignalStrengthDbm = info.getRssi();
                    double freq = info.getFrequency();
                    double thisDist = calculateDistance(currentWifiSignalStrengthDbm, freq);
                if (lastTime == 0) {
                    lastTime = thisTime;
                    lastDist = (int) Double.POSITIVE_INFINITY;
                } else {
                    if ((thisTime - lastTime) > 2) {
                        if ((thisDist - lastDist) > 1) {
                            moving = 1;
                            resumeSensing();
                            lastTime = thisTime;
                        }
                    } else {
                        if (moving == 1) {
                            pauseSensing();
                            moving = 0;
                        }
                    }
                }
            }
        };
          registerReceiver(wifibr, new IntentFilter(WifiManager.RSSI_CHANGED_ACTION));
    }
*/



    public void resumeSensing() {
        Log.d(TAG, "Resume Sensing");

            //register the sensor, use context, name and rate at which sensor events are delivered to us.
            senSensorManager.registerListener(this, senAccelerometer, 20000);


    }

    public void pauseSensing() {
        Log.d(TAG, "Sensing paused");
        if (senSensorManager != null) {
            senSensorManager.unregisterListener(this);
        }
    }


    public double calculateDistance(double signalLevelInDb, double freqInMHz) {
        Log.d(TAG, "Calculate Distance");

        double exp = (27.55 - (20 * Math.log10(freqInMHz)) + abs(signalLevelInDb)) / 20.0;
        return Math.pow(10.0, exp);
    }


    /*When a new sensor event received, execute a new AsyncTask for accelerometer and gyro*/
    @Override
    public void onSensorChanged(SensorEvent event) {

        Log.d(TAG, "osc");
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

            accArrayList.add(event.timestamp + ";" + event.values[0] + ";" + event.values[1] + ";" + event.values[2] + "\n");
        }
    }

    @Override
    public void onDestroy() {
        mWakeLock.release();
        Log.d(TAG, "onDestroy");
        super.onDestroy();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

//    public void makeFile() throws IOException {
//
//        Date now = new Date();
//        String stringDate = new SimpleDateFormat("ddMMyyyyHHmm").format(now);
////        accWriter = new FileWriter(Environment.getExternalStorageDirectory().toString() + stringDate + "_accelerometer.dat");
//
//        aBufferedWriter = new BufferedWriter(accWriter);
//
//    }


    public class LocalBinder extends Binder {

        public WearableService getService() {
            // Return this instance of LocalService so clients can call public methods.
            return WearableService.this;
        }
    }


}


class accelerometerConsumer implements Runnable {
    private static final String TAG = "ConsumerThread";
    private final ArrayList<String> arrayList;
    Context context;
    float currentNascResult = 0;
    float lwv = Float.POSITIVE_INFINITY;
    int reachedEnd = 0;

    public accelerometerConsumer(ArrayList<String> arrayList, Context context) {
        this.arrayList = arrayList;
        Log.d(TAG, "Consumer Created");
        this.context=context;

    }

//Perform step counting
    @Override
        public void run () {
            Log.d(TAG, "run");
            int stepsTaken = 0;

            long nascWindowSize = 2000000000L;
            double sdWalkingThresh = 0.6;
            int thresh = 7;
            long tmin = 700000000L;
            long tmax = 1500000000L;
            int start1 = 0;
            int windowEnd = 0;
            double Rthresh = 0.7;
            int clen = arrayList.size();

            long[] timestamps = new long[clen - 1];
            float[] values = new float[clen - 1];

            for (int j = 0; j < arrayList.size()-1; j++) {
                String[] strs = arrayList.get(j).split(";");
                timestamps[j] = (long) Long.parseLong(strs[0]);
                values[j] = (float) Math.sqrt(Float.parseFloat(strs[1]) * Float.parseFloat(strs[1]) +
                        Float.parseFloat(strs[2]) * Float.parseFloat(strs[2]) + Float.parseFloat(strs[3]) * Float.parseFloat(strs[3]));
            }


            long firstTimeVal = timestamps[0];
            Long timeElapsed = timestamps[clen-2] - firstTimeVal;
            int numWindows = (int) Math.ceil(timeElapsed / nascWindowSize);
            long lastWindow = firstTimeVal + nascWindowSize * (numWindows - 1);
            int lastTime = 0;
            lastTime = find(timestamps, lastWindow);


            for (long p = firstTimeVal; p <= timestamps[lastTime]; p = p + nascWindowSize) {
                float[] windowVals;
                long[] windowTimes;
                if (p == timestamps[lastTime]) {
                    start1 = lastTime;
                    windowEnd = clen;
                    reachedEnd = 1;
                } else {

                    int mIdx = find(timestamps, p);
                    if (p == firstTimeVal) {
                        start1 = 0;
                    } else {
                        start1 = mIdx;
                    }


                    long timeLimit = p + nascWindowSize;
                    if (timeLimit >= timestamps[clen - 2]) {
                        timeLimit = timestamps[clen - 3];
                    }

                    int nIdx = find(timestamps, timeLimit)+1;
                    windowEnd = nIdx - 1;

                     windowVals = Arrays.copyOfRange(values, start1, windowEnd);
                    //windowTimes = Arrays.copyOfRange(timestamps, start1, windowEnd);


                    if (sd(windowVals) < sdWalkingThresh) {
                        continue;
                    }
                }

                stepsTaken = stepsTaken + myWpd(timestamps,values, start1, windowEnd,  510000000L ,thresh, lwv, reachedEnd);
                //long topt = nascv2(timestamps, values, start1, tmax, tmin);
               /* if ((currentNascResult > Rthresh) && (topt!=0)) {
                    stepsTaken = stepsTaken + myWpd(timestamps, values, start1, windowEnd, topt / 2, thresh, lwv, reachedEnd);
                }*/
            }
            Intent intent = new Intent("ruby.finalapp.testIntent");
            intent.putExtra("count",stepsTaken);
            context.sendBroadcast(intent);
        }

    //Normalised Autocorrelation, extracts 't' that best represents time taken between strides
    public long nascv2(long[] timestamps, float[] values, int startIdx, long tmax, long tmin) {
        Log.d(TAG, "nascv2");
        currentNascResult = 0;
        long resTime = 0L;
        long maxTime = 0L;
        float maxInWindowSoFar = Float.NEGATIVE_INFINITY;
        float maxSoFar = Float.NEGATIVE_INFINITY;
        int mEnd = find(timestamps, (timestamps[startIdx] + tmax)) - 1;
        int timeEnd = timestamps.length-1;
        int clen = timestamps.length;
        for (int m = startIdx; m <= mEnd; m++) {
            if (mEnd == 0) {
                break;
            }

            for (long testT = tmin; testT <=tmax; testT = testT + 10000000) {
                long timeLimit = timestamps[m] + testT;
                if (timeLimit > timestamps[timeEnd]) {
                    break;
                }
                int tMinusOneIndex = find(timestamps, timeLimit) - 2;

                float[] numWithinTminusOneArray = Arrays.copyOfRange(values, m, m+tMinusOneIndex);
                int numWithinTminusOne = numWithinTminusOneArray.length;

                int numWithinT = numWithinTminusOne + 1;

                if ((m + numWithinT + numWithinT - 1) > clen-2) {
                    break;
                }

                float[] f1 = Arrays.copyOfRange(values, m, m+ tMinusOneIndex);
                float[] f2 = Arrays.copyOfRange(values, (m + numWithinTminusOne), (m + numWithinT + numWithinTminusOne));

                float[] topBottom =Arrays.copyOfRange(values, (m + numWithinT), (m + numWithinT + numWithinTminusOne));

                float firstMean = (float) mean(f1);
                float secondMean = (float) mean(f2);
                float firstStd = (float) sd(f1);
                float secondStd = (float) sd(f2);

                float[]top1=null;


                top1 = Arrays.copyOfRange(values, m, m+ tMinusOneIndex);

                float[]top2=new float[top1.length];

                for (int j = 0; j <= top1.length - 2; j++) {
                    top2[j] = top1[j] - firstMean;
                }

                for (int k = 0; k <= topBottom.length - 2; k++) {
                    topBottom[k] = topBottom[k] - secondMean;
                }

                float totalSum = 0;

                for (int z = 0; z <= top2.length - 2; z++) {
                    totalSum = totalSum + (top2[z] * topBottom[z]);
                }

                float bottom = (numWithinT) * firstStd * secondStd;
                float thisNascResult = totalSum / bottom;


                if (thisNascResult >= maxSoFar) {
                    maxSoFar = thisNascResult;
                    maxTime = testT;
                    resTime=maxTime;
                }

                if (maxSoFar >= maxInWindowSoFar) {
                    resTime = maxTime;
                    currentNascResult = maxSoFar;
                    maxInWindowSoFar = maxSoFar;
                }

            }
        }

        return resTime;
    }

    public static double mean(float[] m) {
        double sum = 0;
        for (int i = 0; i < m.length; i++) {
            sum += m[i];
        }
        return sum / m.length;
    }

    public static double sd(float[] m) {
        double myMean = mean(m);
        double temp = 0;
        int size = m.length;
        for (int i = 0; i < m.length; i++) {
            double sqDifftoMean = Math.pow(m[i] - myMean, 2);
            temp += sqDifftoMean;
        }
        double meanOfDiffs = (double) temp / (double) (size);

        return Math.sqrt(meanOfDiffs);
    }

    public int find(long[] values, long toFind) {
        int index = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] > toFind) {
                index = i;
                break;
            }
        }
        return index;
    }

    public int findeq(float[] values, float toFind) {
        int index = 0;
        for (int i = 0; i < values.length; i++) {
            if (values[i] == toFind) {
                index = i;
                break;
            }
        }
        return index;
    }

//Windowed Peak detection
    public int myWpd(long[] timestamps, float[] values, int wpdStart, int wpdEnd, Long wpdWinSize, int thresh, float lwvin, int reachedEndIn) {
        Log.d(TAG, "wpd");

        int totalSteps = 0;

        Long timeLimit;
        int mIdx;
        int nIdx;
        long[] windowTimes;
        float[] windowVals;
        int flag = 0;
        float prevMax;
        float firstOfNextWindow;
        long[] wpdInput1;
        float[] wpdInput2;
        int start1;
        int windowEnd;

        if (wpdStart != 0 && wpdEnd != 0) {
            wpdInput1 = Arrays.copyOfRange(timestamps, wpdStart, wpdEnd);
            wpdInput2 = Arrays.copyOfRange(values, wpdStart, wpdEnd);
        } else {
            wpdInput1 = timestamps;
            wpdInput2 = values;
        }

        if (reachedEndIn == 0) {
            firstOfNextWindow = values[wpdEnd + 1];
        } else {
            firstOfNextWindow = Float.POSITIVE_INFINITY;
        }

        if (lwvin == 0) {
            lwv = (int) Double.POSITIVE_INFINITY;
        }


        int clen = wpdInput1.length;
        Long timeElapsed = wpdInput1[clen - 1] - wpdInput1[0];
        int numWindows = (int) Math.ceil(timeElapsed / wpdWinSize);
        long lastWindow = (wpdInput1[0] + (wpdWinSize * (numWindows - 1)));
        int lastTime = find(wpdInput1, lastWindow);
        long firstTimeVal = wpdInput1[0];


        for (long i = wpdInput1[0]; i < lastWindow; i = i + wpdWinSize) {
            if (i == lastWindow) {
                start1 = lastTime;
                windowEnd = clen - 1;
                flag = 1;
            } else {

                mIdx = find(wpdInput1, i);
                if (i == firstTimeVal) {
                    start1 = 1;
                } else {
                    start1 = mIdx;
                }

                timeLimit = i + wpdWinSize;

                if (timeLimit >= wpdInput1[clen - 1]) {
                    timeLimit = wpdInput1[clen - 2];
                }

                nIdx = find(wpdInput1, timeLimit);
                windowEnd = nIdx - 1;
            }
            //windowTimes = Arrays.copyOfRange(wpdInput1, start1, windowEnd);
            windowVals = Arrays.copyOfRange(wpdInput2, start1, windowEnd);

            float maxVal = Float.NEGATIVE_INFINITY;
            //int maxIdx = (int) Double.NEGATIVE_INFINITY;

            for (int p = 0; p < windowEnd - start1-1; p++) {
                if (p == 0) {
                    if ((windowEnd - start1) == 1) {
                        continue;
                    }
                    if ((windowVals[p] > lwv) && (windowVals[p] > windowVals[p + 1])) {
                        maxVal = windowVals[p];
                    }
                    continue;

                }
                if (p == windowEnd - start1) {
                    if (flag != 1) {
                        if ((windowVals[p] > windowVals[p - 1]) && (windowVals[p] > wpdInput2[windowEnd + 1]) && (windowVals[p] > maxVal)) {
                            maxVal = windowVals[p];
                        } else {
                            if ((windowVals[p] > windowVals[p - 1]) && (windowVals[p] > firstOfNextWindow) && (windowVals[p] > maxVal)) {
                                maxVal = windowVals[p];
                            }
                        }
                    }
                    lwv = windowVals[p];
                    continue;
                }
                float prev = windowVals[p - 1];
                float next = windowVals[p + 1];

                if ((windowVals[p] > prev) && (windowVals[p] > next)) {
                    float thisMax = windowVals[p];
                    if (thisMax >= maxVal) {
                        maxVal = thisMax;
                        prevMax = thisMax;
                    }
                }
            }

                //maxIdx = findeq(windowVals, maxVal);
                if (maxVal > thresh) {
                    totalSteps++;
                }

        }
        return totalSteps;
    }
}