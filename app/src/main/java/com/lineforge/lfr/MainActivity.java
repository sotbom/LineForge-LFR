package com.lineforge.lfr;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothSocket;
import android.content.ActivityNotFoundException;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * LineForge LFR: a no-third-party-dependencies Bluetooth Classic SPP controller.
 * Protocol: two-byte binary commands; compatible with ESP32S3_LFR_BT_NVS.
 * The accompanying firmware extension adds [9,0] status query and [10,1] heartbeat.
 */
public final class MainActivity extends Activity {
    private static final int BG = Color.rgb(10, 16, 32);
    private static final int PANEL = Color.rgb(19, 29, 49);
    private static final int PANEL_ALT = Color.rgb(25, 38, 61);
    private static final int BORDER = Color.rgb(43, 59, 83);
    private static final int TEXT = Color.rgb(240, 246, 255);
    private static final int MUTED = Color.rgb(151, 168, 191);
    private static final int MINT = Color.rgb(54, 226, 198);
    private static final int BLUE = Color.rgb(96, 164, 255);
    private static final int PURPLE = Color.rgb(177, 136, 255);
    private static final int RED = Color.rgb(255, 98, 111);
    private static final int ORANGE = Color.rgb(255, 196, 100);
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int REQ_CONNECT = 151;
    private static final String PREFS_NAME = "lineforge_ui";

    private final Handler ui = new Handler(Looper.getMainLooper());
    private final ExecutorService writer = Executors.newSingleThreadExecutor();
    private final AtomicInteger session = new AtomicInteger();
    private final Object outputLock = new Object();
    private volatile BluetoothSocket socket;
    private volatile OutputStream socketOut;
    private volatile boolean connected = false;
    private boolean running = false;
    private boolean liveTune = false;
    private boolean calibrationValid = false;
    private boolean queryReceived = false;
    private SharedPreferences preferences;
    private TextView connectionLabel, selectedDevice, heroStatus, eventLog, sendCountLabel;
    private View connectionDot;
    private Button connectButton, startButton, stopButton;
    private Switch liveSwitch;
    private PidControl kp, ki, kd;
    private int sentBytes = 0, receivedBytes = 0;
    private String lastEvent = "Ready to pair";
    private final Runnable heartbeatTask = new Runnable() {
        @Override public void run() {
            if (connected && running) sendBytes(new byte[]{10, 1}, false);
            ui.postDelayed(this, 600);
        }
    };

    @Override public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setStatusBarColor(BG);
        getWindow().setNavigationBarColor(BG);
        preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        createInterface();
        ui.postDelayed(heartbeatTask, 600);
        addEvent("Pair HC-06 in Android Bluetooth settings, then select it here.");
    }

    @Override protected void onStop() {
        // Treat the app leaving the foreground as an intentional STOP.
        if (running && connected) {
            running = false;
            sendBytes(new byte[]{7, 0}, true);
            updateRunUI();
        }
        super.onStop();
    }

    @Override protected void onDestroy() {
        ui.removeCallbacks(heartbeatTask);
        disconnectSocket();
        writer.shutdownNow();
        super.onDestroy();
    }

    private int dp(float value) {
        return Math.round(getResources().getDisplayMetrics().density * value);
    }

    private GradientDrawable rounded(int color, int radius, int outline) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(color);
        d.setCornerRadius(dp(radius));
        if (outline != 0) d.setStroke(dp(1), outline);
        return d;
    }

    private TextView txt(String value, int sp, int color, boolean heavy) {
        TextView t = new TextView(this);
        t.setText(value);
        t.setTextSize(sp);
        t.setTextColor(color);
        t.setGravity(Gravity.CENTER_VERTICAL);
        if (heavy) t.setTypeface(Typeface.create("sans-serif", Typeface.BOLD));
        return t;
    }

    private LinearLayout col() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.VERTICAL);
        return l;
    }

    private LinearLayout row() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.HORIZONTAL);
        l.setGravity(Gravity.CENTER_VERTICAL);
        return l;
    }

    private LinearLayout.LayoutParams wrap() {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams full() {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams weighted() {
        return new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f);
    }

    private View gap(int height) {
        View v = new View(this);
        v.setLayoutParams(new LinearLayout.LayoutParams(1, dp(height)));
        return v;
    }

    private LinearLayout card() {
        LinearLayout c = col();
        c.setPadding(dp(18), dp(17), dp(18), dp(18));
        c.setBackground(rounded(PANEL, 20, BORDER));
        return c;
    }

    private Button button(String label, int fill, int foreground) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextSize(14);
        b.setTypeface(Typeface.create("sans-serif-medium", Typeface.BOLD));
        b.setTextColor(foreground);
        b.setMinHeight(0);
        b.setMinimumHeight(0);
        b.setPadding(dp(13), dp(13), dp(13), dp(13));
        b.setBackground(rounded(fill, 14, fill == PANEL_ALT ? BORDER : 0));
        return b;
    }

    private void createInterface() {
        LinearLayout screen = col();
        screen.setBackgroundColor(BG);
        setContentView(screen);

        ScrollView scroll = new ScrollView(this);
        scroll.setClipToPadding(false);
        scroll.setFillViewport(true);
        scroll.setVerticalScrollBarEnabled(false);
        screen.addView(scroll, new LinearLayout.LayoutParams(-1, 0, 1f));
        LinearLayout content = col();
        content.setPadding(dp(18), dp(24), dp(18), dp(20));
        scroll.addView(content);

        LinearLayout heading = row();
        TextView mark = txt("LF", 21, BG, true);
        mark.setGravity(Gravity.CENTER);
        mark.setBackground(rounded(MINT, 15, 0));
        heading.addView(mark, new LinearLayout.LayoutParams(dp(47), dp(47)));
        LinearLayout headingText = col();
        headingText.setPadding(dp(12), 0, 0, 0);
        headingText.addView(txt("LINEFORGE", 22, TEXT, true));
        headingText.addView(txt("ESP32-S3  /  BLUETOOTH PID STUDIO", 10, MUTED, false));
        heading.addView(headingText);
        content.addView(heading);
        content.addView(gap(22));

        LinearLayout deviceCard = card();
        LinearLayout deviceHeader = row();
        connectionDot = new View(this);
        connectionDot.setBackground(rounded(RED, 100, 0));
        deviceHeader.addView(connectionDot, new LinearLayout.LayoutParams(dp(11), dp(11)));
        connectionLabel = txt("OFFLINE", 12, RED, true);
        connectionLabel.setPadding(dp(9), 0, 0, 0);
        deviceHeader.addView(connectionLabel, weighted());
        TextView protocol = txt("BT CLASSIC  •  SPP", 10, MUTED, true);
        deviceHeader.addView(protocol);
        deviceCard.addView(deviceHeader);
        deviceCard.addView(gap(13));
        selectedDevice = txt("No Bluetooth module connected", 16, TEXT, true);
        deviceCard.addView(selectedDevice);
        deviceCard.addView(gap(14));
        connectButton = button("Select paired HC-06", MINT, BG);
        connectButton.setOnClickListener(v -> {
            if (connected) disconnectSocket();
            else ensureBluetoothPermission();
        });
        deviceCard.addView(connectButton, full());
        content.addView(deviceCard, full());
        content.addView(gap(18));

        LinearLayout titleRow = row();
        titleRow.addView(txt("PID TUNING", 13, TEXT, true), weighted());
        TextView resolution = txt("0–255  /  10^divider", 11, MUTED, false);
        titleRow.addView(resolution);
        content.addView(titleRow, full());
        content.addView(gap(10));

        kp = new PidControl("KP", "PROPORTIONAL", MINT, 1, 40, 0);
        ki = new PidControl("KI", "INTEGRAL", BLUE, 3, 0, 0);
        kd = new PidControl("KD", "DERIVATIVE", PURPLE, 5, 1, 0);
        content.addView(kp.view, full()); content.addView(gap(11));
        content.addView(ki.view, full()); content.addView(gap(11));
        content.addView(kd.view, full()); content.addView(gap(13));

        LinearLayout autoRow = row();
        LinearLayout autoText = col();
        autoText.addView(txt("Live tuning", 14, TEXT, true));
        autoText.addView(txt("Send changes on slider release", 11, MUTED, false));
        autoRow.addView(autoText, weighted());
        liveSwitch = new Switch(this);
        liveSwitch.setThumbTintList(ColorStateList.valueOf(MINT));
        liveSwitch.setTrackTintList(ColorStateList.valueOf(BORDER));
        liveSwitch.setChecked(preferences.getBoolean("live", false));
        liveTune = liveSwitch.isChecked();
        liveSwitch.setOnCheckedChangeListener((CompoundButton b, boolean checked) -> {
            liveTune = checked;
            preferences.edit().putBoolean("live", checked).apply();
        });
        autoRow.addView(liveSwitch);
        content.addView(autoRow, full());
        content.addView(gap(12));

        LinearLayout uploadButtons = row();
        Button apply = button("APPLY ALL  ↗", MINT, BG);
        apply.setOnClickListener(v -> uploadAll());
        uploadButtons.addView(apply, new LinearLayout.LayoutParams(0, dp(51), 1f));
        View split = new View(this);
        uploadButtons.addView(split, new LinearLayout.LayoutParams(dp(9), 1));
        Button read = button("READ FROM ROBOT", PANEL_ALT, TEXT);
        read.setOnClickListener(v -> requestRobotState());
        uploadButtons.addView(read, new LinearLayout.LayoutParams(0, dp(51), 1f));
        content.addView(uploadButtons);
        content.addView(gap(20));

        LinearLayout calCard = card();
        calCard.addView(txt("SENSOR CALIBRATION", 13, TEXT, true));
        calCard.addView(gap(7));
        calCard.addView(txt("Saved to ESP32 NVS • 8-channel QTR-8A", 12, MUTED, false));
        calCard.addView(gap(14));
        Button recalibrate = button("RECALIBRATE SENSORS", PANEL_ALT, TEXT);
        recalibrate.setOnClickListener(v -> confirmCalibration());
        calCard.addView(recalibrate, full());
        content.addView(calCard, full());
        content.addView(gap(17));

        LinearLayout console = card();
        LinearLayout consoleHead = row();
        consoleHead.addView(txt("LINK CONSOLE", 12, TEXT, true), weighted());
        sendCountLabel = txt("TX 0 B  /  RX 0 B", 10, MUTED, false);
        consoleHead.addView(sendCountLabel);
        console.addView(consoleHead);
        console.addView(gap(12));
        eventLog = txt("Waiting for HC-06…", 12, MUTED, false);
        eventLog.setLineSpacing(dp(5), 1.0f);
        console.addView(eventLog);
        content.addView(console, full());
        content.addView(gap(14));
        TextView foot = txt("Protocol v2  •  9600 baud  •  Binary two-byte commands", 11, MUTED, false);
        foot.setGravity(Gravity.CENTER);
        content.addView(foot);

        // Sticky run controls, always accessible even while scrolling.
        LinearLayout actionArea = col();
        actionArea.setPadding(dp(18), dp(12), dp(18), dp(14));
        actionArea.setBackground(rounded(PANEL, 18, BORDER));
        heroStatus = txt("MOTORS DISARMED", 11, MUTED, true);
        heroStatus.setGravity(Gravity.CENTER);
        actionArea.addView(heroStatus);
        actionArea.addView(gap(10));
        LinearLayout actionRow = row();
        startButton = button("▶  START RUN", MINT, BG);
        stopButton = button("■  STOP NOW", RED, Color.WHITE);
        actionRow.addView(startButton, new LinearLayout.LayoutParams(0, dp(58), 1f));
        actionRow.addView(new View(this), new LinearLayout.LayoutParams(dp(10), 1));
        actionRow.addView(stopButton, new LinearLayout.LayoutParams(0, dp(58), 1f));
        startButton.setOnClickListener(v -> startRobot());
        stopButton.setOnClickListener(v -> stopRobot());
        actionArea.addView(actionRow);
        screen.addView(actionArea, full());

        kp.restore(); ki.restore(); kd.restore();
        updateRunUI();
    }

    private final class PidControl {
        final String key, description;
        final int color, command;
        final LinearLayout view;
        final TextView largeValue;
        final SeekBar slider;
        final EditText number;
        final Spinner divider;
        boolean binding = false;

        PidControl(String key, String description, int color, int command, int initialNumber, int initialDivider) {
            this.key = key; this.description = description; this.color = color; this.command = command;
            view = card();
            LinearLayout top = row();
            LinearLayout lhs = col();
            lhs.addView(txt(description, 10, MUTED, true));
            lhs.addView(txt(key, 19, color, true));
            top.addView(lhs, weighted());
            largeValue = txt("0.0000", 29, TEXT, true);
            largeValue.setGravity(Gravity.RIGHT | Gravity.CENTER_VERTICAL);
            top.addView(largeValue);
            view.addView(top);
            view.addView(gap(14));
            slider = new SeekBar(MainActivity.this);
            slider.setMax(255);
            slider.setProgressTintList(ColorStateList.valueOf(color));
            slider.setThumbTintList(ColorStateList.valueOf(color));
            slider.setProgressBackgroundTintList(ColorStateList.valueOf(BORDER));
            view.addView(slider, full());
            view.addView(gap(12));
            LinearLayout bottom = row();
            bottom.addView(txt("VALUE", 10, MUTED, true));
            number = new EditText(MainActivity.this);
            number.setTextColor(TEXT);
            number.setTextSize(14);
            number.setSingleLine(true);
            number.setInputType(InputType.TYPE_CLASS_NUMBER);
            number.setSelectAllOnFocus(true);
            number.setGravity(Gravity.CENTER);
            number.setBackground(rounded(PANEL_ALT, 9, BORDER));
            number.setPadding(dp(8), dp(7), dp(8), dp(7));
            LinearLayout.LayoutParams nlp = new LinearLayout.LayoutParams(dp(66), dp(38));
            nlp.setMargins(dp(9), 0, dp(14), 0);
            bottom.addView(number, nlp);
            bottom.addView(txt("DIVIDE BY", 10, MUTED, true));
            divider = new Spinner(MainActivity.this);
            final String[] options = {"1", "10", "100", "1k", "10k"};
            ArrayAdapter<String> adapter = new ArrayAdapter<String>(MainActivity.this,
                    android.R.layout.simple_spinner_item, options) {
                @Override public View getView(int position, View recycled, ViewGroup parent) {
                    TextView t = txt(options[position], 14, TEXT, true);
                    t.setText(options[position] + "  ▾");
                    t.setPadding(dp(10), dp(7), 0, dp(7));
                    return t;
                }
                @Override public View getDropDownView(int position, View recycled, ViewGroup parent) {
                    TextView t = txt(options[position], 15, TEXT, true);
                    t.setBackgroundColor(PANEL_ALT);
                    t.setPadding(dp(18), dp(13), dp(18), dp(13));
                    return t;
                }
            };
            divider.setAdapter(adapter);
            divider.setBackground(rounded(PANEL_ALT, 9, BORDER));
            LinearLayout.LayoutParams dlp = new LinearLayout.LayoutParams(0, dp(38), 1f);
            dlp.setMargins(dp(9), 0, 0, 0);
            bottom.addView(divider, dlp);
            view.addView(bottom);
            slider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                @Override public void onStartTrackingTouch(SeekBar sb) { }
                @Override public void onProgressChanged(SeekBar sb, int v, boolean fromUser) {
                    if (!fromUser || binding) return;
                    binding = true;
                    number.setText(String.valueOf(v));
                    number.setSelection(number.length());
                    binding = false;
                    updateDisplay();
                }
                @Override public void onStopTrackingTouch(SeekBar sb) {
                    saveLocal();
                    if (liveTune) sendThisGain();
                }
            });
            number.addTextChangedListener(new TextWatcher() {
                @Override public void beforeTextChanged(CharSequence s, int start, int count, int after) { }
                @Override public void onTextChanged(CharSequence s, int start, int before, int count) {
                    if (binding) return;
                    binding = true;
                    slider.setProgress(value());
                    binding = false;
                    updateDisplay();
                }
                @Override public void afterTextChanged(Editable s) { }
            });
            number.setOnFocusChangeListener((v, focused) -> {
                if (!focused) {
                    setData(value(), divisor());
                    saveLocal();
                    if (liveTune) sendThisGain();
                }
            });
            divider.setOnItemSelectedListener(new android.widget.AdapterView.OnItemSelectedListener() {
                @Override public void onNothingSelected(android.widget.AdapterView<?> p) { }
                @Override public void onItemSelected(android.widget.AdapterView<?> p, View v, int position, long id) {
                    updateDisplay();
                    if (binding) return;
                    saveLocal();
                    // Never auto-send divider on initial spinner binding.
                    if (liveTune && divider.getTag() != null) sendThisGain();
                    divider.setTag("bound");
                }
            });
            setData(initialNumber, initialDivider);
        }

        int value() {
            try { return Math.min(255, Math.max(0, Integer.parseInt(number.getText().toString()))); }
            catch (Exception e) { return 0; }
        }
        int divisor() { return Math.min(4, Math.max(0, divider.getSelectedItemPosition())); }
        void setData(int numerator, int divisor) {
            binding = true;
            int v = Math.min(255, Math.max(0, numerator));
            int d = Math.min(4, Math.max(0, divisor));
            number.setText(String.valueOf(v));
            slider.setProgress(v);
            divider.setSelection(d);
            binding = false;
            updateDisplay();
        }
        void updateDisplay() {
            double value = value() / Math.pow(10.0, divisor());
            largeValue.setText(String.format(Locale.US, "%.4f", value));
        }
        void restore() {
            int defNum = command == 1 ? 40 : command == 5 ? 1 : 0;
            setData(preferences.getInt(key + "_v", defNum), preferences.getInt(key + "_d", 0));
        }
        void saveLocal() {
            preferences.edit().putInt(key + "_v", value()).putInt(key + "_d", divisor()).apply();
        }
        void sendThisGain() {
            if (!connected) return;
            sendBytes(new byte[]{(byte)command, (byte)value(), (byte)(command + 1), (byte)divisor()}, true);
            addEvent(key + " updated: " + largeValue.getText());
        }
    }

    private void uploadAll() {
        if (!connected) { showToast("Connect your HC-06 first"); return; }
        blurInputs();
        byte[] values = new byte[]{
                1, (byte)kp.value(), 2, (byte)kp.divisor(),
                3, (byte)ki.value(), 4, (byte)ki.divisor(),
                5, (byte)kd.value(), 6, (byte)kd.divisor()};
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal();
        sendBytes(values, true);
        addEvent("Sent six PID settings (12 bytes). NVS will save after 2 s.");
    }

    private void blurInputs() {
        View focus = getCurrentFocus();
        if (focus != null) {
            focus.clearFocus();
            InputMethodManager input = (InputMethodManager)getSystemService(INPUT_METHOD_SERVICE);
            if (input != null) input.hideSoftInputFromWindow(focus.getWindowToken(), 0);
        }
    }

    private void startRobot() {
        if (!connected) { showToast("Connect the HC-06 first"); return; }
        // The ESP32 only accepts START when calibration is valid.
        if (queryReceived && !calibrationValid) {
            showToast("Calibrate the QTR-8A before starting");
            return;
        }
        sendBytes(new byte[]{7, 1}, true);
        running = true;
        updateRunUI();
        addEvent("START requested; watchdog heartbeat active.");
    }

    private void stopRobot() {
        running = false;
        updateRunUI();
        if (connected) sendBytes(new byte[]{7, 0}, true);
        addEvent("STOP requested.");
    }

    private void updateRunUI() {
        if (heroStatus == null) return;
        heroStatus.setText(running ? "RUNNING  •  HEARTBEAT ACTIVE" : "MOTORS DISARMED");
        heroStatus.setTextColor(running ? MINT : MUTED);
        if (startButton != null) startButton.setAlpha(connected && !running ? 1f : 0.55f);
        if (stopButton != null) stopButton.setAlpha(connected ? 1f : 0.7f);
        if (running) getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        else getWindow().clearFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    private void confirmCalibration() {
        if (!connected) { showToast("Connect the HC-06 first"); return; }
        new AlertDialog.Builder(this)
                .setTitle("Recalibrate QTR-8A?")
                .setMessage("Place the robot on WHITE before proceeding. Once started, leave it still for 2 seconds, then sweep all 8 sensors across the black line for 5 seconds. The motors will stay stopped. Successful values are saved to NVS.")
                .setNegativeButton("Cancel", null)
                .setPositiveButton("Start calibration", (dialog, which) -> {
                    running = false;
                    updateRunUI();
                    sendBytes(new byte[]{8, 1}, true);
                    addEvent("Calibrating: 2 s white + 5 s sweep. Awaiting result…");
                }).show();
    }

    private void requestRobotState() {
        if (!connected) { showToast("Connect the HC-06 first"); return; }
        queryReceived = false;
        sendBytes(new byte[]{9, 0}, true);
        addEvent("Requesting actual PID values from the ESP32…");
        ui.postDelayed(() -> {
            if (connected && !queryReceived)
                addEvent("No status reply. Flash the included firmware update to enable readback.");
        }, 2300);
    }

    private boolean hasConnectPermission() {
        return Build.VERSION.SDK_INT < 31 ||
                checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
    }

    private void ensureBluetoothPermission() {
        if (!hasConnectPermission()) {
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT}, REQ_CONNECT);
            return;
        }
        choosePairedDevice();
    }

    @Override public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        if (requestCode == REQ_CONNECT) {
            if (results.length > 0 && results[0] == PackageManager.PERMISSION_GRANTED) choosePairedDevice();
            else addEvent("Nearby devices permission is required to connect to HC-06.");
        }
    }

    @SuppressLint("MissingPermission")
    private void choosePairedDevice() {
        if (!hasConnectPermission()) return;
        BluetoothManager manager = (BluetoothManager)getSystemService(Context.BLUETOOTH_SERVICE);
        BluetoothAdapter adapter = manager == null ? null : manager.getAdapter();
        if (adapter == null) { showToast("This phone does not support Bluetooth"); return; }
        if (!adapter.isEnabled()) {
            new AlertDialog.Builder(this).setTitle("Bluetooth is off")
                    .setMessage("Turn on Bluetooth and pair the HC-06 in your phone settings first.")
                    .setNegativeButton("Cancel", null)
                    .setPositiveButton("Bluetooth settings", (d, w) -> openBluetoothSettings())
                    .show();
            return;
        }
        Set<BluetoothDevice> bonded = adapter.getBondedDevices();
        if (bonded == null || bonded.isEmpty()) {
            new AlertDialog.Builder(this).setTitle("No paired devices")
                    .setMessage("Open Android Bluetooth settings and pair with HC-06 (commonly PIN 1234 or 0000). Then return and tap Connect.")
                    .setNegativeButton("Cancel", null)
                    .setPositiveButton("Open settings", (d, w) -> openBluetoothSettings()).show();
            return;
        }
        List<BluetoothDevice> devices = new ArrayList<>(bonded);
        Collections.sort(devices, (a, b) -> {
            String x = a.getName() == null ? "" : a.getName();
            String y = b.getName() == null ? "" : b.getName();
            boolean ax = x.toLowerCase(Locale.ROOT).contains("hc-");
            boolean by = y.toLowerCase(Locale.ROOT).contains("hc-");
            if (ax != by) return ax ? -1 : 1;
            return x.compareToIgnoreCase(y);
        });
        String[] labels = new String[devices.size()];
        for (int i = 0; i < devices.size(); i++) {
            BluetoothDevice d = devices.get(i);
            labels[i] = (d.getName() == null ? "Unnamed Bluetooth device" : d.getName()) + "\n" + d.getAddress();
        }
        new AlertDialog.Builder(this)
                .setTitle("Choose your paired HC-06")
                .setItems(labels, (dialog, which) -> connectDevice(devices.get(which)))
                .setNeutralButton("Pair new device", (dialog, which) -> openBluetoothSettings())
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void openBluetoothSettings() {
        try { startActivity(new Intent(Settings.ACTION_BLUETOOTH_SETTINGS)); }
        catch (ActivityNotFoundException e) { showToast("Open Bluetooth in Android settings"); }
    }

    @SuppressLint("MissingPermission")
    private void connectDevice(BluetoothDevice device) {
        if (!hasConnectPermission()) { ensureBluetoothPermission(); return; }
        disconnectSocket();
        int epoch = session.get();
        String name = device.getName() == null ? device.getAddress() : device.getName();
        connectionLabel.setText("CONNECTING…");
        connectionLabel.setTextColor(ORANGE);
        connectionDot.setBackground(rounded(ORANGE, 100, 0));
        selectedDevice.setText(name);
        connectButton.setText("Connecting…");
        connectButton.setEnabled(false);
        new Thread(() -> {
            BluetoothSocket s = null;
            String failure = "Unknown connection error";
            try {
                s = device.createRfcommSocketToServiceRecord(SPP_UUID);
                s.connect();
            } catch (IOException first) {
                failure = first.getMessage();
                closeQuietly(s);
                s = null;
                // Older HC-05/HC-06 modules sometimes need the insecure RFCOMM variant.
                try {
                    s = device.createInsecureRfcommSocketToServiceRecord(SPP_UUID);
                    s.connect();
                } catch (IOException second) {
                    failure = second.getMessage();
                    closeQuietly(s);
                    final String err = failure;
                    ui.post(() -> {
                        if (session.get() == epoch) {
                            updateConnectionUI(false, null);
                            addEvent("Connection failed: " + err);
                        }
                    });
                    return;
                }
            }
            if (session.get() != epoch) { closeQuietly(s); return; }
            try {
                socket = s;
                socketOut = s.getOutputStream();
                InputStream input = s.getInputStream();
                connected = true;
                queryReceived = false;
                BluetoothSocket finalSocket = s;
                ui.post(() -> {
                    if (session.get() != epoch) return;
                    updateConnectionUI(true, name);
                    addEvent("Connected to " + name + " using serial port profile.");
                    requestRobotState();
                });
                readSocket(input, epoch);
                closeQuietly(finalSocket);
                if (session.get() == epoch) ui.post(() -> {
                    if (session.get() != epoch) return;
                    disconnectSocket();
                    addEvent("Bluetooth connection closed; robot will stop via heartbeat timeout.");
                });
            } catch (IOException e) {
                closeQuietly(s);
                if (session.get() == epoch) ui.post(() -> {
                    if (session.get() == epoch) {
                        disconnectSocket();
                        addEvent("Bluetooth read error: " + e.getMessage());
                    }
                });
            }
        }, "HC06-Connection").start();
    }

    private void readSocket(InputStream input, int epoch) throws IOException {
        int remaining = 0;
        byte[] frameBody = new byte[8];
        int at = 0;
        while (session.get() == epoch) {
            int b = input.read();
            if (b < 0) break;
            int nextTotal = ++receivedBytes;
            ui.post(() -> {
                if (session.get() == epoch) sendCountLabel.setText("TX " + sentBytes + " B  /  RX " + nextTotal + " B");
            });
            if (remaining > 0) {
                frameBody[at++] = (byte)b;
                remaining--;
                if (remaining == 0) {
                    byte[] copy = frameBody.clone();
                    ui.post(() -> {
                        if (session.get() == epoch) receiveStateFrame(copy);
                    });
                    at = 0;
                }
                continue;
            }
            if (b == 0xD0) { remaining = 8; at = 0; continue; }
            final int reply = b;
            ui.post(() -> {
                if (session.get() == epoch) handleReply(reply);
            });
        }
    }

    private void handleReply(int b) {
        if (b == 0x06) return;  // ACK - intentionally don't fill the console
        if (b == 0x15) {
            if (running && !calibrationValid) { running = false; updateRunUI(); }
            addEvent("ESP32 rejected a command (NAK).");
            return;
        }
        if (b == 0xC1) { calibrationValid = true; addEvent("Calibration successful; values saved to NVS."); return; }
        if (b == 0xC0) { calibrationValid = false; running = false; updateRunUI(); addEvent("Calibration failed. Motors are disabled."); return; }
        if (b == 0xC2) { calibrationValid = true; addEvent("Saved sensor calibration loaded from NVS."); return; }
        if (b == 0xD1) { running = false; updateRunUI(); addEvent("ESP32 motor safety timeout: heartbeat lost."); return; }
        addEvent(String.format(Locale.US, "Unknown reply byte 0x%02X", b));
    }

    private void receiveStateFrame(byte[] data) {
        int checksum = 0;
        for (int i = 0; i < 7; ++i) checksum ^= (data[i] & 0xff);
        if (checksum != (data[7] & 0xff)) { addEvent("Status frame checksum error."); return; }
        int kpd = data[1] & 0xff, kid = data[3] & 0xff, kdd = data[5] & 0xff;
        if (kpd > 4 || kid > 4 || kdd > 4) { addEvent("Invalid PID data in status frame."); return; }
        kp.setData(data[0] & 0xff, kpd);
        ki.setData(data[2] & 0xff, kid);
        kd.setData(data[4] & 0xff, kdd);
        kp.saveLocal(); ki.saveLocal(); kd.saveLocal();
        int flags = data[6] & 0xff;
        calibrationValid = (flags & 1) != 0;
        running = (flags & 2) != 0;
        updateRunUI();
        queryReceived = true;
        addEvent("PID synchronized from ESP32 NVS/RAM. Calibration: " + (calibrationValid ? "READY" : "NOT READY"));
    }

    private void sendBytes(byte[] payload, boolean reportErrors) {
        if (!connected) {
            if (reportErrors) showToast("HC-06 is disconnected");
            return;
        }
        final int expectedSession = session.get();
        writer.execute(() -> {
            try {
                synchronized (outputLock) {
                    if (expectedSession != session.get()) return;
                    OutputStream out = socketOut;
                    if (out == null || !connected) return;
                    out.write(payload);
                    out.flush();
                }
                sentBytes += payload.length;
                ui.post(() -> sendCountLabel.setText("TX " + sentBytes + " B  /  RX " + receivedBytes + " B"));
            } catch (IOException e) {
                if (reportErrors) ui.post(() -> addEvent("Send failed: " + e.getMessage()));
                ui.post(() -> disconnectSocket());
            }
        });
    }

    private void updateConnectionUI(boolean active, String name) {
        connectionLabel.setText(active ? "CONNECTED" : "OFFLINE");
        connectionLabel.setTextColor(active ? MINT : RED);
        connectionDot.setBackground(rounded(active ? MINT : RED, 100, 0));
        if (name != null) selectedDevice.setText(name);
        else if (!active) selectedDevice.setText("No Bluetooth module connected");
        connectButton.setEnabled(true);
        connectButton.setText(active ? "Disconnect HC-06" : "Select paired HC-06");
        updateRunUI();
    }

    private void disconnectSocket() {
        session.incrementAndGet();
        running = false;
        connected = false;
        socketOut = null;
        BluetoothSocket old = socket;
        socket = null;
        closeQuietly(old);
        if (Looper.getMainLooper() == Looper.myLooper()) updateConnectionUI(false, null);
        else ui.post(() -> updateConnectionUI(false, null));
    }

    private void closeQuietly(BluetoothSocket s) {
        try { if (s != null) s.close(); } catch (Exception ignored) { }
    }

    private void addEvent(String s) {
        lastEvent = s;
        if (eventLog != null) eventLog.setText(s);
    }

    private void showToast(String s) { Toast.makeText(this, s, Toast.LENGTH_SHORT).show(); }
}
