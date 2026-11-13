package com.lineforge.lfr;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.SystemClock;
import android.view.View;

/** Animated eight-sensor strip: white = background, mint bars = black line reflectance. */
public final class QtrSensorView extends View {
    private final Paint brush = new Paint(3);
    private final float[] shown = new float[8];
    private final float[] target = new float[8];
    private float currentPosition = 3.5f;
    private float targetPosition = 3.5f;
    private boolean lineSeen = false;
    private boolean calibration = false;
    private boolean online = false;
    private long lastFrameMs = 0;
    private final int fg = Color.rgb(235, 247, 255);
    private final int dim = Color.rgb(138, 159, 187);
    private final int mint = Color.rgb(54, 226, 198);
    private final int panel = Color.rgb(25, 38, 61);
    public QtrSensorView(Context context) {
        super(context);
        setLayerType(View.LAYER_TYPE_SOFTWARE, null);
    }
    private float dp(float px) { return getResources().getDisplayMetrics().density * px; }
    @Override protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        int h = (int)dp(215);
        setMeasuredDimension(resolveSize((int)dp(270), widthMeasureSpec), resolveSize(h, heightMeasureSpec));
    }
    public void setConnected(boolean b) { online = b; invalidate(); }
    public void update(int[] normalized, int position1000, boolean seesLine, boolean calibrating) {
        for (int i = 0; i < 8; i++) target[i] = Math.min(1f, Math.max(0f, normalized[i]/255f));
        if (position1000 >= 0 && position1000 <= 7000) targetPosition = position1000/1000f;
        lineSeen = seesLine;
        calibration = calibrating;
        online = true;
        lastFrameMs = SystemClock.uptimeMillis();
        invalidate();
    }
    private void rounded(Canvas c, float a, float b, float d, float e, float r, int color) {
        brush.setColor(color);
        brush.setStyle(Paint.Style.FILL);
        brush.clearShadowLayer();
        c.drawRoundRect(new RectF(a,b,d,e), r,r,brush);
    }
    @Override protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        final float w = getWidth();
        final long age = SystemClock.uptimeMillis() - lastFrameMs;
        final boolean fresh = online && age < 1000;
        brush.clearShadowLayer();
        brush.setTypeface(android.graphics.Typeface.create("sans-serif-medium", 1));
        brush.setTextSize(dp(12));
        brush.setColor(fresh ? (calibration ? Color.rgb(255,196,100) : lineSeen ? mint : Color.rgb(255,98,111)) : dim);
        canvas.drawText(!fresh ? "NO LIVE DATA" : calibration ? "CALIBRATING • RAW SIGNAL" : lineSeen ? "LINE DETECTED" : "LINE LOST", dp(2), dp(18), brush);
        brush.setTypeface(android.graphics.Typeface.create("sans-serif", 0));
        brush.setColor(dim); brush.setTextSize(dp(10));
        String label = fresh ? "8 Hz telemetry" : "Waiting for valid packets";
        canvas.drawText(label, w - brush.measureText(label) - dp(2), dp(18), brush);
        final float left = dp(2), right = w - dp(2), gap = dp(7);
        final float cell = (right-left-gap*7)/8;
        final float chartTop = dp(42), chartBot = dp(165), chartHeight = chartBot-chartTop;
        for (int i=0;i<8;i++) {
            float x=left+i*(cell+gap);
            rounded(canvas, x, chartTop, x+cell, chartBot, dp(9), panel);
            float diff=target[i]-shown[i];
            shown[i] += diff*0.24f;
            float top=chartBot-chartHeight*shown[i];
            if (shown[i]>0.01f) {
                int color = (shown[i] >= 0.25f) ? mint : Color.rgb(74,113,134);
                rounded(canvas,x,top,x+cell,chartBot,dp(9),color);
            }
            brush.setTextSize(dp(11));
            brush.setTypeface(android.graphics.Typeface.create("sans-serif-medium",1));
            brush.setColor(fg);
            String n="S"+(i+1);
            canvas.drawText(n, x + (cell-brush.measureText(n))/2,dp(186),brush);
            brush.setTypeface(android.graphics.Typeface.create("sans-serif",0));
            brush.setColor(dim); brush.setTextSize(dp(9));
            String level=Integer.toString(Math.round(target[i]*100));
            canvas.drawText(level,x+(cell-brush.measureText(level))/2,dp(200),brush);
        }
        currentPosition += (targetPosition-currentPosition)*0.24f;
        if (fresh && lineSeen && !calibration) {
            float markerX=left+cell/2+currentPosition*(cell+gap);
            brush.setColor(mint);
            brush.setShadowLayer(dp(8),0,0,mint);
            canvas.drawCircle(markerX,dp(32),dp(4),brush);
            brush.clearShadowLayer();
            rounded(canvas,markerX-dp(1),dp(39),markerX+dp(1),chartTop,dp(1),mint);
        }
        if (online && age < 1000) postInvalidateDelayed(32);
    }
}
