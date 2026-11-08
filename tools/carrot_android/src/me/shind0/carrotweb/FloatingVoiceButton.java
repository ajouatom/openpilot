package me.shind0.carrotweb;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.RippleDrawable;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewConfiguration;
import android.widget.ImageButton;

/** Position is stored as a fraction of the usable area so rotation keeps the button reachable. */
final class FloatingVoiceButton extends ImageButton {
  private final SharedPreferences preferences;
  private final int margin, slop;
  private float xFraction, yFraction, downX, downY, initialX, initialY;
  private boolean dragging, cancelled;

  FloatingVoiceButton(Context context, SharedPreferences preferences) {
    super(context);
    this.preferences = preferences;
    margin = dp(8);
    slop = ViewConfiguration.get(context).getScaledTouchSlop();
    xFraction = preferences.getFloat("voice_button_x", 1f);
    yFraction = preferences.getFloat("voice_button_y", 0.18f);
    GradientDrawable circle = new GradientDrawable();
    circle.setShape(GradientDrawable.OVAL);
    circle.setColor(Color.rgb(255, 177, 100));
    setBackground(new RippleDrawable(ColorStateList.valueOf(0x44000000), circle, null));
    setImageResource(getResources().getIdentifier("voice_mic", "drawable", context.getPackageName()));
    setScaleType(ScaleType.FIT_CENTER);
    int pad = dp(15);
    setPadding(pad, pad, pad, pad);
    setElevation(dp(5));
    setFocusable(true);
    setContentDescription(UiText.get("음성비서 열기. 끌어서 이동", "Open voice assistant. Drag to move"));
  }

  private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }
  private float span(int parentSize, int buttonSize) { return Math.max(0, parentSize - buttonSize - 2 * margin); }
  private static float clamp(float value, float max) { return Math.max(0, Math.min(max, value)); }

  void place() {
    View parent = (View)getParent();
    if (parent == null || getWidth() == 0) return;
    setX(margin + clamp(xFraction, 1) * span(parent.getWidth(), getWidth()));
    setY(margin + clamp(yFraction, 1) * span(parent.getHeight(), getHeight()));
  }

  private void move(float dx, float dy) {
    View parent = (View)getParent();
    setX(margin + clamp(initialX + dx - margin, span(parent.getWidth(), getWidth())));
    setY(margin + clamp(initialY + dy - margin, span(parent.getHeight(), getHeight())));
  }

  private void savePosition() {
    View parent = (View)getParent();
    float width = span(parent.getWidth(), getWidth()), height = span(parent.getHeight(), getHeight());
    if (width > 0) xFraction = clamp((getX() - margin) / width, 1);
    if (height > 0) yFraction = clamp((getY() - margin) / height, 1);
    preferences.edit().putFloat("voice_button_x", xFraction).putFloat("voice_button_y", yFraction).apply();
  }

  @Override public boolean onTouchEvent(MotionEvent event) {
    float dx = event.getRawX() - downX, dy = event.getRawY() - downY;
    switch (event.getActionMasked()) {
      case MotionEvent.ACTION_DOWN:
        downX = event.getRawX(); downY = event.getRawY();
        initialX = getX(); initialY = getY(); dragging = false; cancelled = false;
        setPressed(true);
        getParent().requestDisallowInterceptTouchEvent(true);
        return true;
      case MotionEvent.ACTION_POINTER_DOWN:
      case MotionEvent.ACTION_CANCEL:
        cancelled = true; setPressed(false); place(); return true;
      case MotionEvent.ACTION_MOVE:
      case MotionEvent.ACTION_UP:
        if (cancelled) return true;
        if (!dragging && Math.hypot(dx, dy) > slop) { dragging = true; setPressed(false); }
        if (dragging) move(dx, dy);
        if (event.getActionMasked() == MotionEvent.ACTION_UP) {
          setPressed(false);
          if (dragging) savePosition(); else performClick();
        }
        return true;
      default: return true;
    }
  }

  @Override public boolean performClick() { return super.performClick(); }
}
