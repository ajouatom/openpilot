#pragma once

#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/onroad/alerts.h"

typedef struct Rect1 {
	int x, y, w, h;
	int centerX() const { return x + w / 2; }
	int centerY() const { return y + h / 2; }
	int right() const { return x + w; }
	int bottom() const { return y + h; }
	bool ptInRect(int px, int py) const {
		return px >= x && px < (x + w) && py >= y && py < (y + h);
	}
} Rect1;

void ui_draw(UIState *s, int w, int h);
void ui_update_alert(const OnroadAlerts::Alert& a);
void ui_draw_alert(UIState* s);
void ui_draw_image(const UIState *s, const Rect1 &r, const char *name, float alpha);
void ui_draw_rect(NVGcontext *vg, const Rect1 &r, NVGcolor color, int width, float radius = 0);
void ui_fill_rect(NVGcontext *vg, const Rect1 &r, const NVGpaint &paint, float radius = 0);
void ui_fill_rect(NVGcontext *vg, const Rect1 &r, const NVGcolor &color, float radius = 0);
void ui_nvg_init(UIState *s);
void ui_resize(UIState *s, int width, int height);
void ui_update_params(UIState *s);


