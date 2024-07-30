#include "selfdrive/ui/paint.h"

#include <cassert>
#include <cmath>

//#define __TEST

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
//#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_RED nvgRGBA(255, 0, 0, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 203, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 153, 0, x)
#define COLOR_BLUE nvgRGBA(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(0, 0, 255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)

#define BOLD "KaiGenGothicKR-Bold"//"Inter-Bold"//"sans-bold"

int g_fps= 0;

#if 0
static void ui_print(UIState *s, int x, int y,  const char* fmt, ... )
{
  char* msg_buf = NULL;
  va_list args;
  va_start(args, fmt);
  vasprintf( &msg_buf, fmt, args);
  va_end(args);
  nvgText(s->vg, x, y, msg_buf, NULL);
}
#endif
static void ui_draw_text(const UIState* s, float x, float y, const char* string, float size, NVGcolor color, const char* font_name, float borderWidth=3.0, float shadowOffset=0.0, NVGcolor borderColor=COLOR_BLACK, NVGcolor shadowColor=COLOR_BLACK) {
    y += 6;
    nvgFontFace(s->vg, font_name);
    nvgFontSize(s->vg, size);
    if (borderWidth > 0.0) {
        //NVGcolor borderColor = COLOR_BLACK;
        nvgFillColor(s->vg, borderColor);
        for (int i = 0; i < 360; i += 45) {
            float angle = i * NVG_PI / 180.0f;
            float offsetX = borderWidth * cos(angle);
            float offsetY = borderWidth * sin(angle);
            nvgText(s->vg, x + offsetX, y + offsetY, string, NULL);
        }
    }
    if (shadowOffset != 0.0) {
        //NVGcolor shadowColor = COLOR_BLACK;
        nvgFillColor(s->vg, shadowColor);
        nvgText(s->vg, x + shadowOffset, y + shadowOffset, string, NULL);
    }
    nvgFillColor(s->vg, color);
    nvgText(s->vg, x, y, string, NULL);
}

float a_x = 0.;
float a_y = 0.;
float a_size = 0.;
const int a_max = 100;
char a_font[128]="";
int a_time = -1;
char a_string[256] = "";
NVGcolor a_color = COLOR_WHITE;
static void ui_draw_text_a2(const UIState* s) {
    if (a_time <= 0) return;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    a_time-=10;
    int a_time1 = a_time;
    if (a_time1 > 100) a_time1 = 100;
    int x = (s->fb_w / 2 * a_time1 + a_x * (a_max - a_time1)) / a_max;
    int y = ((s->fb_h - 400) * a_time1 + a_y * (a_max - a_time1)) / a_max;
    int size = (350 * a_time1 + a_size * (a_max - a_time1)) / a_max;
    if(a_time>=100) ui_draw_text(s, x, y, a_string, size, a_color, a_font, 9.0, 8.0, COLOR_BLACK, COLOR_BLACK);
    else ui_draw_text(s, x, y, a_string, size, a_color, a_font);
}
static void ui_draw_text_a(const UIState* s, float x, float y, const char* string, float size, NVGcolor color, const char* font_name) {
    a_x = x;
    a_y = y;
    strcpy(a_string, string);
    a_size = size;
    a_color = color;
    strcpy(a_font, font_name);
    a_time = 130;
}

static void ui_draw_line(const UIState* s, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint, float stroke=0.0, NVGcolor strokeColor=COLOR_WHITE) {
    if (vd.size() == 0) return;

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, vd.at(0).x(), vd.at(0).y());
    for (int i = 1; i < vd.size(); i++) {
        nvgLineTo(s->vg, vd.at(i).x(), vd.at(i).y());
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);
    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, strokeColor);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }
}
static void ui_draw_line2(const UIState* s, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke=0.0, NVGcolor strokeColor=COLOR_WHITE) {

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, x[0], y[0]);
    for (int i = 1; i < size; i++) {
        nvgLineTo(s->vg, x[i], y[i]);
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);

    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, strokeColor);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }
}

static void ui_draw_bsd(const UIState* s, const QPolygonF& vd, NVGcolor* color, bool right) {
    int index = vd.length();

    float x[4], y[4];
    for (int i = 0; i < index/2 - 2; i += 2) {

        if (right) {
            x[0] = vd[i + 0].x();
            y[0] = vd[i + 0].y();
            x[1] = vd[i + 1].x();
            y[1] = vd[i + 1].y();
            x[2] = vd[index - i - 3].x();
            y[2] = vd[index - i - 3].y();
            x[3] = vd[index - i - 2].x();
            y[3] = vd[index - i - 2].y();
        }
        else {
            x[0] = vd[i + 0].x();
            y[0] = vd[i + 0].y();
            x[1] = vd[i + 1].x();
            y[1] = vd[i + 1].y();
            x[2] = vd[index - i - 3].x();
            y[2] = vd[index - i - 3].y();
            x[3] = vd[index - i - 2].x();
            y[3] = vd[index - i - 2].y();
        }
        ui_draw_line2(s, x, y, 4, color, nullptr, 3.0f);
    }

}
template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
    std::vector<T> xData(x_list);
    std::vector<T> yData(y_list);
    int size = xData.size();

    int i = 0;
    if (x >= xData[size - 2]) {
        i = size - 2;
    }
    else {
        while (x > xData[i + 1]) i++;
    }
    T xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
    if (!extrapolate) {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }

    T dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
}
#if 0
template <class T>
float interp(float x, const T* x_list, const T* y_list, size_t size, bool extrapolate)
{
    int i = 0;
    if (x >= x_list[size - 2]) {
        i = size - 2;
    }
    else {
        while (x > x_list[i + 1]) i++;
    }
    T xL = x_list[i], yL = y_list[i], xR = x_list[i + 1], yR = y_list[i + 1];
    if (!extrapolate) {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }

    T dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
}
static void ui_draw_path(const UIState* s) {
    const UIScene& scene = s->scene;
    SubMaster& sm = *(s->sm);
    auto model_position = sm["modelV2"].getModelV2().getPosition();
    float max_distance = std::clamp(model_position.getX()[TRAJECTORY_SIZE - 1],
        MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    auto lead_one = sm["radarState"].getRadarState().getLeadOne();
    if (lead_one.getStatus()) {
        const float lead_d = lead_one.getDRel() * 2.;
        max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
    }
    auto    car_state = sm["carState"].getCarState();
    float   accel = car_state.getAEgo();
    float   v_ego_kph = getVEgo() * MS_TO_KPH;
    const auto line_x = model_position.getX(), line_y = model_position.getY(), line_z = model_position.getZ();
    float idxs[33], line_xs[33], line_ys[33], line_zs[33];
    for (int i = 0; i < 33; i++) {
        idxs[i] = (float)i;
        line_xs[i] = line_x[i];
        line_ys[i] = line_y[i];
        line_zs[i] = line_z[i];
    }

    static bool forward = true;

    if (accel > 0.5) forward = true;
    if (accel < -0.5) forward = false;

    float start_dist = 10.0;
    float dt = 0.01;
    float max_t = sqrt((dist - start_dist) / (getVEgo() * dt));
    static float pos_t = 2;   // dist = pos_t[] * pos_t[] * v_ego * 0.01 + 3.0,  pos_t = sqrt((dist-3.0) / (getVEgo()*0.01))
    for (int i = 0; i < 5; i++) {
        dist = (pos_t+i) * (pos_t+i) * getVEgo() * dt + start_dist;
        if (dist > max_distance) {
            pos_t[i] = 0;
            dist = start_dist;
        }

        if (forward) pos_t[i] = (pos_t[i] + 1) % (int)max_t;
        else if (--pos_t[i] < 0) pos_t[i] = (int)max_t - 1;
    }
    // 0~150M까지 display해준다... 
    // 높이시작은 0.8 ~ s->show_z_offset(max_distance)

}
#endif

void ui_draw_image(const UIState* s, const Rect1& r, const char* name, float alpha) {
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
    nvgRect(s->vg, r.x, r.y, r.w, r.h);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
}
static void ui_draw_circle_image_rotation(const UIState* s, int center_x, int center_y, int radius, const char* image, NVGcolor color, float img_alpha, float angleSteers = 0) {
    const int img_size = radius * 2.0;
    float img_rotation = angleSteers / 180 * 3.141592;
    int ct_pos = -radius;

    nvgBeginPath(s->vg);
    nvgCircle(s->vg, center_x, center_y, radius);
    nvgFillColor(s->vg, color);
    nvgFill(s->vg);
    //ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);

    nvgSave(s->vg);
    nvgTranslate(s->vg, center_x, center_y);
    nvgRotate(s->vg, -img_rotation);

    ui_draw_image(s, { ct_pos, ct_pos, img_size, img_size }, image, img_alpha);
    nvgRestore(s->vg);
}
void ui_draw_rect(NVGcontext* vg, const Rect1& r, NVGcolor color, int width, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
}
static inline void fill_rect(NVGcontext* vg, const Rect1& r, const NVGcolor* color, const NVGpaint* paint, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    if (color) nvgFillColor(vg, *color);
    if (paint) nvgFillPaint(vg, *paint);
    nvgFill(vg);
}
void ui_fill_rect(NVGcontext* vg, const Rect1& r, const NVGcolor& color, float radius) {
    fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext* vg, const Rect1& r, const NVGpaint& paint, float radius) {
    fill_rect(vg, r, nullptr, &paint, radius);
}

static NVGcolor get_tpms_color(float tpms) {
    if (tpms < 5 || tpms > 60) // N/A
        return nvgRGBA(255, 255, 255, 220);
    if (tpms < 31)
        return nvgRGBA(255, 90, 90, 220);
    return nvgRGBA(255, 255, 255, 220);
}

static const char *get_tpms_text(float tpms) {
    if (tpms < 5 || tpms > 60)
        return "  -";


    static char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return str;
}
#ifdef __TEST
int test_seq = 0;
#endif

void DrawApilot::drawLaneLines(const UIState* s) {

    static float pathDrawSeq = 0;


    const UIScene& scene = s->scene;
    SubMaster& sm = *(s->sm);
    NVGcolor color;
    auto    car_state = sm["carState"].getCarState();
    //auto    controls_state = sm["controlsState"].getControlsState();
    bool brake_valid = car_state.getBrakeLights();

    bool left_blindspot = sm["carState"].getCarState().getLeftBlindspot();
    bool right_blindspot = sm["carState"].getCarState().getRightBlindspot();

    // show_lane_info: -1 : all off, 0: path only, 1: path+lane, 2: path+lane+edge
    // lanelines
    if (s->show_lane_info > 0) {
        for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
            //color = nvgRGBAf(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i] * 2.0, 0.5, 1.0));
            color = nvgRGBAf(1.0, 1.0, 1.0, (scene.lane_line_probs[i]>0.3)?1.0:0.0);
            ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
        }
    }
    if (s->show_blind_spot) {
#ifdef __TEST
        left_blindspot = right_blindspot  = true;
#endif
        color = nvgRGBA(255, 215, 0, 150);
        NVGcolor color2 = nvgRGBA(0, 204, 0, 150);
        auto lead_left = (*s->sm)["radarState"].getRadarState().getLeadLeft();
        auto lead_right = (*s->sm)["radarState"].getRadarState().getLeadRight();
        //auto controls_state = (*s->sm)["controlsState"].getControlsState();
        //int leftBlinkerExt = controls_state.getLeftBlinkerExt();
        //int rightBlinkerExt = controls_state.getRightBlinkerExt();
        auto meta = (*s->sm)["modelV2"].getModelV2().getMeta();
        auto laneChangeState = meta.getLaneChangeState();
        auto laneChangeDirection = meta.getLaneChangeDirection();
        bool rightLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) && (laneChangeDirection == cereal::LaneChangeDirection::RIGHT);
        bool leftLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) && (laneChangeDirection == cereal::LaneChangeDirection::LEFT);

        if (left_blindspot) ui_draw_bsd(s, scene.lane_barrier_vertices[0], &color, false);
        //else if (lead_left.getStatus() && lead_left.getDRel() < getVEgo() * 3.0 && leftBlinkerExt)  ui_draw_bsd(s, scene.lane_barrier_vertices[0], &color2, false);
        else if (lead_left.getStatus() && lead_left.getDRel() < getVEgo() * 3.0 && leftLaneChange)  ui_draw_bsd(s, scene.lane_barrier_vertices[0], &color2, false);
        if (right_blindspot) ui_draw_bsd(s, scene.lane_barrier_vertices[1], &color, true);
        //else if (lead_right.getStatus() && lead_right.getDRel() < getVEgo() * 3.0 && rightBlinkerExt >= 10000) ui_draw_bsd(s, scene.lane_barrier_vertices[1], &color2, true);
        else if (lead_right.getStatus() && lead_right.getDRel() < getVEgo() * 3.0 && rightLaneChange) ui_draw_bsd(s, scene.lane_barrier_vertices[1], &color2, true);
    }

    // road edges
    if (s->show_lane_info > 1) {
        for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
            //color = nvgRGBAf(1.0, 0.0, 1.0, std::clamp<float>(3.0 - scene.road_edge_stds[i], 0.0, 1.0));
            //color = nvgRGBAf(1.0, 0.0, 1.0, (scene.road_edge_stds[i] < 2.0) ? 1.0 : 0.0);
            float temp_f = std::clamp<float>(scene.road_edge_stds[i] / 2., 0.0, 1.0);
            color = nvgRGBAf(1.0 - temp_f, 0.0, temp_f, 1.0);
            ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
        }
    }
    // paint path
    static bool forward = true;
    int alpha = 120;
    NVGcolor colors[10] = {
         COLOR_RED_ALPHA(alpha),
         nvgRGBA(255,153,0, alpha),
         COLOR_YELLOW_ALPHA(alpha),
         COLOR_GREEN_ALPHA(alpha),
         COLOR_BLUE_ALPHA(alpha),
         nvgRGBA(0,0,128, alpha),
         nvgRGBA(0x8b,0,0xff, alpha),
         COLOR_OCHRE_ALPHA(alpha),
         COLOR_WHITE_ALPHA(alpha),
         COLOR_BLACK_ALPHA(alpha),
    };
    if (s->show_lane_info > -1) {
        int show_path_color = (s->use_lane_lines) ? s->show_path_color_lane : s->show_path_color;
        int show_path_mode = (s->use_lane_lines) ? s->show_path_mode_lane : s->show_path_mode;

        if (!isLongActive()) {
            show_path_color = s->show_path_color_cruise_off;
            show_path_mode = s->show_path_mode_cruise_off;
        }
        if (show_path_mode == 0) {
            ui_draw_line(s, scene.track_vertices, &colors[show_path_color % 10], nullptr,(show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid)?COLOR_RED:COLOR_WHITE);
        }
        else if (show_path_mode >= 13 && show_path_mode <= 15) {
            int     track_vertices_len = scene.track_vertices.length();
            float xp[3][128], yp[3][128];
            float   g = 0.05, gc=0.4;
            switch (show_path_mode) {
            case 13: g = 0.2; gc = 0.10; break;
            case 14: g = 0.45; gc = 0.05; break;
            case 15: 
            default:
                g = 0.05; gc = 0.05; break;
            }
            if (show_path_mode == 15) gc = g;
            int glen = track_vertices_len / 2 - 1;
            for (int i = 0; i < glen; i ++) {
                int e = track_vertices_len - i - 1;
                int ge = glen*2 - 1 - i;
                float x1 = scene.track_vertices[i].x();
                float y1 = scene.track_vertices[i].y();
                float x2 = scene.track_vertices[e].x();
                float y2 = scene.track_vertices[e].y();
                xp[0][i] = x1;  yp[0][i] = y1;
                xp[0][ge] = x1 + (x2 - x1) * g; yp[0][ge] = y1 + (y2 - y1) * g;
                xp[1][i] = x1 + (x2 - x1) * (0.5 - gc); yp[1][i] = y1 + (y2 - y1) * (0.5 - gc);
                xp[1][ge] = x1 + (x2 - x1) * (0.5 + gc); yp[1][ge] = y1 + (y2 - y1) * (0.5 + gc);
                xp[2][i] = x1 + (x2 - x1) * (1.0 - g); yp[2][i] = y1 + (y2 - y1) * (1.0 - g);
                xp[2][ge] = x2; yp[2][ge] = y2;
            }
            if(show_path_mode==13 || show_path_mode == 14)
                ui_draw_line2(s, xp[0], yp[0], glen*2, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE);
            if(show_path_mode==13 || show_path_mode == 15)
                ui_draw_line2(s, xp[1], yp[1], glen * 2, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE);
            if (show_path_mode == 13 || show_path_mode == 14)
                ui_draw_line2(s, xp[2], yp[2], glen * 2, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE);
        }
        else if (show_path_mode >= 9) {
            int     track_vertices_len = scene.track_vertices.length();
            //printf("len = %d\n", track_vertices_len);
            float   x[6], y[6];
            int     color_n = 0;
            //for (int i = 0; i < track_vertices_len / 2; i++)
            //    printf("(%.1f,%.1f)(%.1f,%.1f)\n", scene.track_vertices[i].x(), scene.track_vertices[i].y(), scene.track_vertices[track_vertices_len-i - 1].x(), scene.track_vertices[track_vertices_len-i - 1].y());
            for (int i = 0; i < track_vertices_len / 2 - 1; i += 3) {
                int e = track_vertices_len - i - 1;
                x[0] = scene.track_vertices[i].x();
                y[0] = scene.track_vertices[i].y();
                x[1] = scene.track_vertices[i + 1].x();
                y[1] = scene.track_vertices[i + 1].y();
                x[2] = (scene.track_vertices[i + 2].x() + scene.track_vertices[e - 2].x()) / 2;
                y[2] = (scene.track_vertices[i + 2].y() + scene.track_vertices[e - 2].y()) / 2;
                x[3] = scene.track_vertices[e - 1].x();
                y[3] = scene.track_vertices[e - 1].y();
                x[4] = scene.track_vertices[e].x();
                y[4] = scene.track_vertices[e].y();
                x[5] = (x[1] + x[3]) / 2;
                y[5] = (y[1] + y[3]) / 2;
                //ui_draw_line2(s, x, y, 6, &colors[color_n], nullptr, (show_path_color >= 10) ? 2.0 : 0.0);
                ui_draw_line2(s, x, y, 6, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE);

                if (++color_n > 6) color_n = 0;
            }
        }
        else {

            int     track_vertices_len = scene.track_vertices.length();
            //color = nvgRGBA(0, 150, 0, 30);
            float   accel = car_state.getAEgo();
            float   v_ego_kph = getVEgo() * MS_TO_KPH;
#ifdef __TEST
            v_ego_kph = 20.0;
            accel = -1.2;
#endif
            float   seq = (1.0 * v_ego_kph / 100.0);
            if (seq < 0.3) seq = 0.3;
            if (accel < -1.0) forward = false;
            if (accel > -0.5) forward = true;
            int max_seq = track_vertices_len / 4 + 3;
#ifdef __TEST
            if(max_seq> 16) max_seq = 16;
#endif
            static int pathDrawSeq2 = -1;

            if (forward) {
                pathDrawSeq += seq;
                if (pathDrawSeq > max_seq) {
                    pathDrawSeq = (pathDrawSeq2>=0) ? pathDrawSeq2: 0;
                }
            }
            else {
                pathDrawSeq -= seq;
                if (pathDrawSeq < 0) {
                    pathDrawSeq = (pathDrawSeq2>=0)? pathDrawSeq2: max_seq;
                }
            }
            if (max_seq > 15) pathDrawSeq2 = ((int)pathDrawSeq - max_seq/2 + max_seq) % max_seq;
            else pathDrawSeq2 = -5;

            float   x[6], y[6];

            switch (show_path_mode) {
            case 1: case 2: case 5: case 6:
                for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
                    x[0] = scene.track_vertices[i].x();
                    y[0] = scene.track_vertices[i].y();
                    x[1] = scene.track_vertices[i + 2].x();
                    y[1] = scene.track_vertices[i + 2].y();
                    x[2] = scene.track_vertices[track_vertices_len - i - 3].x();
                    y[2] = scene.track_vertices[track_vertices_len - i - 3].y();
                    x[3] = scene.track_vertices[track_vertices_len - i - 1].x();
                    y[3] = scene.track_vertices[track_vertices_len - i - 1].y();

                    int draw = false;
                    if ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2) draw = true;
                    if (pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2) draw = true;
                    //if ((int)(pathDrawSeq + 0.5) * 2 == i || (((int)(pathDrawSeq + 0.5) - 2) * 2 == i))  draw = true;
                    if (track_vertices_len / 2 < 8) draw = true;
                    if (show_path_mode == 5 || show_path_mode == 6) draw = true;

                    if (draw) {
                        switch (show_path_mode) {
                        case 2: case 6: ui_draw_line2(s, x, y, 4, &colors[color_n], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE); break;
                        default:        ui_draw_line2(s, x, y, 4, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE); break;
                        }
                    }

                    if (i > 1) color_n++;
                    if (color_n > 6) color_n = 0;
                }
                break;
            case 3: case 4: case 7: case 8:
                for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
                    x[0] = scene.track_vertices[i].x();
                    y[0] = scene.track_vertices[i].y();
                    x[1] = scene.track_vertices[i + 2].x();
                    y[1] = scene.track_vertices[i + 2].y();
                    x[2] = (scene.track_vertices[i + 4].x() + scene.track_vertices[track_vertices_len - i - 5].x()) / 2;
                    y[2] = (scene.track_vertices[i + 4].y() + scene.track_vertices[track_vertices_len - i - 5].y()) / 2;
                    x[3] = scene.track_vertices[track_vertices_len - i - 3].x();
                    y[3] = scene.track_vertices[track_vertices_len - i - 3].y();
                    x[4] = scene.track_vertices[track_vertices_len - i - 1].x();
                    y[4] = scene.track_vertices[track_vertices_len - i - 1].y();
                    x[5] = (x[1] + x[3]) / 2;
                    y[5] = (y[1] + y[3]) / 2;

                    int draw = false;
                    if ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2) draw = true;
                    if (pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2) draw = true;
                    //if ((int)(pathDrawSeq + 0.5) * 2 == i || (((int)(pathDrawSeq + 0.5) - 2) * 2 == i))  draw = true;
                    if (track_vertices_len / 2 < 8) draw = true;
                    if (show_path_mode == 7 || show_path_mode == 8) draw = true;

                    if (draw) {
                        switch (show_path_mode) {
                        case 4: case 8:     ui_draw_line2(s, x, y, 6, &colors[color_n], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE); break;
                        default:            ui_draw_line2(s, x, y, 6, &colors[show_path_color % 10], nullptr, (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0, (brake_valid) ? COLOR_RED : COLOR_WHITE); break;

                        }
                    }
                    if (i > 1) color_n++;
                    if (color_n > 6) color_n = 0;
                }
                break;
            }
        }
    }
}

void DrawPlot::drawPlotting(const UIState* s, int index, int start, float x, float y[], int size, NVGcolor* color, float stroke) {

    nvgBeginPath(s->vg);
    plotRatio = (plotMax - plotMin) < 1.0 ? plotHeight : plotHeight / (plotMax - plotMin);
    float dx = 2.0;
    char str[128];

    for (int i = 0; i < size; i++) {
        float data = y[(start - i + PLOT_MAX) % PLOT_MAX];
        float plot_y = plotY + plotHeight - (data - plotMin) * plotRatio;
        if (i == 0) {
            nvgMoveTo(s->vg, x + (size - i)*dx, plot_y);
            sprintf(str, "%.2f", data);
            ui_draw_text(s, x + (size - i)*dx + 50, plot_y + ((index>0)?40:0), str, 40, *color, BOLD);
        }
        else nvgLineTo(s->vg, x + (size - i)*dx, plot_y);
    }
    //nvgClosePath(s->vg);

    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, *color);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }
}

// 에잉 시간텀... ㅠㅠ
void DrawPlot::makePlotData(const UIState* s, float& data1, float& data2, char *str) {

    SubMaster& sm = *(s->sm);
    auto    car_state = sm["carState"].getCarState();
    float   a_ego = car_state.getAEgo();
    float   v_ego = car_state.getVEgo();
    auto    car_control = sm["carControl"].getCarControl();
    float   accel_out = car_control.getActuators().getAccel();
    //auto    live_parameters = sm["liveParameters"].getLiveParameters();
    //float   roll = live_parameters.getRoll();
    auto    controls_state = sm["controlsState"].getControlsState();
    auto    torque_state = controls_state.getLateralControlState().getTorqueState();
    float   curvature = torque_state.getActualLateralAccel();
    //float   desired_curvature = controls_state.getDesiredCurvature();
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    float   accel = lp.getAccels()[0];
    float   speeds_0 = lp.getSpeeds()[0];
    //const auto lat_plan = sm["lateralPlan"].getLateralPlan();
    //float   curvatures_0 = lat_plan.getCurvatures()[0];
    float   curvatures_0 = torque_state.getDesiredLateralAccel();

    const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
    const auto position = model.getPosition();
    const auto velocity = model.getVelocity();

    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();    

    switch (s->show_plot_mode) {
    case 0:
    case 1:
        data1 = a_ego;
        data2 = accel;
        sprintf(str, "1.Accel (G:desired, Y:actual)");
        break;
    case 2:
        // curvature * v * v : 원심가속도
        data1 = curvature;
        //data2 = (desired_curvature * v_ego * v_ego) - (roll * 9.81);
        data2 = curvatures_0;
        sprintf(str, "2.Lateral Accel(G:desired, Y:actual)");
        break;
    case 3:
        data1 = v_ego;
        data2 = speeds_0;
        sprintf(str, "3.Speed/Accel(G:speed, Y:accel)");
        break;
    case 4:
        data1 = position.getX()[32];
        data2 = velocity.getX()[32];
        sprintf(str, "4.Model data(G:velocity, Y:position");
        break;
    case 5:
        data1 = lead_radar.getVLeadK();
        data2 = lead_radar.getALeadK();
        sprintf(str, "5.Detected radar(G:aLeadK, Y:vLeadK)");
        break;
    case 6:
        data1 = accel_out; // 노
        data2 = lead_radar.getALeadK(); // 녹
        sprintf(str, "6.Detected radar(G:aLeadK, Y:accel)");
        break;
    case 7:
        data1 = lp.getTFollow();//  //노
        data2 = lead_radar.getVLat(); // 녹
        sprintf(str, "7.Detected radar(G:VLat, Y:TF)");
        break;
    case 8:
        data1 = lead_radar.getALeadTau();//  //노
        data2 = lead_radar.getALeadK(); // 녹
        sprintf(str, "8.Detected radar(G:aLeadK, Y:aLeadTau)");
        break;
    case 9:
        data1 = lead_radar.getALeadK();
        data2 = lead_radar.getALead(); // getDRel();
        sprintf(str, "9.Detected radar(G:aLead, Y:aLeadK)");
        break;
        data1 = lead_radar.getVLeadK();
        data2 = lead_radar.getVLead(); // getDRel();
        sprintf(str, "9.Detected radar(G:vLead, Y:vLeadK)");
        break;
    case 10:
        data1 = a_ego; // 노
        data2 = accel_out;  // 녹
        sprintf(str, "10.Accel (G:accel output, Y:a_ego)");
        break;
    default:
        data1 = data2 = 0;
        sprintf(str, "no data");
        break;
    }
    if (s->show_plot_mode != show_plot_mode_prev) {
        plotSize = 0;
        plotIndex = 0;
        plotMin = 0.;
        plotMax = 0.;
        show_plot_mode_prev = s->show_plot_mode;
    }
}

void DrawPlot::draw(const UIState* s) {

    if (s->show_plot_mode == 0) return;

    float _data = 0.;
    float _data1 = 0.;
    char title[128] = "";

    makePlotData(s, _data, _data1, title);

#ifdef __TEST
    static float _data_s = 0.0;
    _data = _data_s;
    _data += 0.1;
    if (_data > 2.0) _data = -2.0;
    _data1 = 3. - _data;

    _data_s = _data;
#endif
    if (plotMin > _data) plotMin = _data;
    if (plotMax < _data) plotMax = _data;
    if (plotMin > _data1) plotMin = _data1;
    if (plotMax < _data1) plotMax = _data1;

    if (plotMin > -2.) plotMin = -2.0;
    if (plotMax < 2.0) plotMax = 2.0;
    plotIndex = (plotIndex + 1) % PLOT_MAX;
    plotQueue[0][plotIndex] = _data;
    plotQueue[1][plotIndex] = _data1;
    if (plotSize < PLOT_MAX - 1) plotSize++;

    if (s->fb_w < 1200) return;

    NVGcolor color[2] = { COLOR_YELLOW, COLOR_GREEN };
    for (int i = 0; i < 2; i++) {
        //drawPlotting(s, i, plotX, plotQueue[i], plotSize, &color[i], nullptr);
        drawPlotting(s, i, plotIndex, plotX, plotQueue[i], plotSize, &color[i], 3.0f);
    }
    //ui_draw_rect(s->vg, { (int)plotX, (int)plotY, (int)plotWidth, (int)plotHeight }, COLOR_WHITE, 2, 0);
    ui_draw_text(s, s->fb_w/2, 20, title, 25, COLOR_WHITE, BOLD);
}
void DrawApilot::drawRadarInfo(const UIState* s) {

    char str[128];

    if (s->show_radar_info) {
        int wStr = 40;
        for (auto const& vrd : s->scene.lead_vertices_side) {
            auto [rx, ry, rd, rv, ry_rel, v_lat, radar] = vrd;

            if (rv < -1.0 || rv > 1.0) {
                sprintf(str, "%.0f", rv * 3.6);
                wStr = 35 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(rx - wStr / 2), (int)(ry - 35), wStr, 42 }, (!radar)?COLOR_BLUE:(rv>0.)?COLOR_GREEN:COLOR_RED, 15);
                ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
                if (s->show_radar_info >= 2) {
                    sprintf(str, "%.1f", ry_rel);
                    ui_draw_text(s, rx, ry - 40, str, 30, COLOR_WHITE, BOLD);
                    sprintf(str, "%.2f", v_lat);
                    //sprintf(str, "%.1f", rd);
                    ui_draw_text(s, rx, ry + 30, str, 30, COLOR_WHITE, BOLD);
                }
            }
#if 0
            else if (v_lat < -1.0 || v_lat > 1.0) {
                sprintf(str, "%.0f", (rv + v_lat) * 3.6);
                wStr = 35 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(rx - wStr / 2), (int)(ry - 35), wStr, 42 }, COLOR_ORANGE, 15);
                ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
                if (s->show_radar_info >= 2) {
                    sprintf(str, "%.1f", ry_rel);
                    ui_draw_text(s, rx, ry - 40, str, 30, COLOR_WHITE, BOLD);
                }
            }
#endif
            else if (s->show_radar_info >= 3) {
                //sprintf(str, "%.1f", ry_rel);
                //ui_draw_text(s, rx, ry - 40, str, 30, COLOR_WHITE, BOLD);
                strcpy(str, "*");
                ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
            }
        }
    }
}
void DrawApilot::makeLeadData(const UIState* s) {
    SubMaster& sm = *(s->sm);
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    auto lp_source = lp.getLongitudinalPlanSource();
    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();

    switch (lp_source) {
    case cereal::LongitudinalPlan::LongitudinalPlanSource::CRUISE:
        m_lpSource = 0;
        break;
    case cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD0:
        m_lpSource = 1;
        break;
    case cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD1:
        lead_radar = sm["radarState"].getRadarState().getLeadTwo();
        m_lpSource = 2;
        break;
    case cereal::LongitudinalPlan::LongitudinalPlanSource::LEAD2:
        m_lpSource = 3;
        break;
    case cereal::LongitudinalPlan::LongitudinalPlanSource::E2E:
        m_lpSource = 4;
        break;
    default:
        m_lpSource = -1;
        break;
    }
    auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];
    bool radar_detected = lead_radar.getStatus() && lead_radar.getRadar();
    m_fLeadDistRadar = radar_detected ? lead_radar.getDRel() : 0;
    m_fLeadDistVision = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.52) : 0;      // RADAR_TO_CAMERA: 1.52

    //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    m_fStopDist = 0;// lp.getXStop();

    m_bLeadStatus = (lead_radar.getStatus() == 1);
    m_bLeadSCC = lead_radar.getRadarTrackId() == 0;
}
void DrawApilot::drawBackground(const UIState* s) {
    if (s->show_mode == 2) {
        NVGpaint track_bg;
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h - 50, s->fb_w, s->fb_h - 350,
            nvgRGBA(0, 0, 0, 250), nvgRGBA(0, 0, 0, 0));
        float x[4], y[4];
        x[0] = 0.0;
        y[0] = s->fb_h;
        x[1] = 0.0;
        y[1] = s->fb_h - 500;
        x[2] = s->fb_w;
        y[2] = s->fb_h - 500;
        x[3] = s->fb_w;
        y[3] = s->fb_h;
        ui_draw_line2(s, x, y, 4, nullptr, &track_bg, 0.0);
    }
    else if (s->show_mode == 3) {
        NVGpaint track_bg;
        track_bg = nvgLinearGradient(s->vg, 0, 0, 0, 150,
            nvgRGBA(0, 0, 0, 250), nvgRGBA(0, 0, 0, 0));
        float x[4], y[4];
        x[0] = 0.0;
        y[0] = 0;
        x[1] = 0.0;
        y[1] = 500;
        x[2] = s->fb_w;
        y[2] = 500;
        x[3] = s->fb_w;
        y[3] = 0;
        ui_draw_line2(s, x, y, 4, nullptr, &track_bg, 0.0);
    }
}
void DrawApilot::drawTPMS(const UIState* s) {
    // Tpms...
    SubMaster& sm = *(s->sm);
    auto car_state = sm["carState"].getCarState();
    if (s->show_tpms) {
        int bx = (192 - 24) / 2 + UI_BORDER_SIZE + 10;
        int by = s->fb_h - 280 / 2 + 10;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
#ifdef __TEST
        fl = 25;
        fr = 28;
        rl = 32;
        rr = 44;
        //s->show_dm_info = 0;
#endif
        if (s->show_dm_info < 1) {
            int sx = 100;
            int sy = 220;
            bx = 120;
            by = s->fb_h - 100;
            ui_draw_image(s, { bx - sx / 2, by - sy / 2, sx, sy }, "ic_tire", 1.0f);
            //QColor tpmsColor = get_tpms_color(fl);
            //drawTextWithColor(painter, bx - 80, by - 50, get_tpms_text(fl), tpmsColor);
            //tpmsColor = get_tpms_color(fr);
            //drawTextWithColor(painter, bx + 80, by - 50, get_tpms_text(fr), tpmsColor);
            //tpmsColor = get_tpms_color(rl);
            //drawTextWithColor(painter, bx - 80, by + 75, get_tpms_text(rl), tpmsColor);
            //tpmsColor = get_tpms_color(rr);
            //drawTextWithColor(painter, bx + 80, by + 75, get_tpms_text(rr), tpmsColor);
            ui_draw_text(s, bx - 80, by - 50, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
            ui_draw_text(s, bx + 80, by - 50, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
            ui_draw_text(s, bx - 80, by + 75, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
            ui_draw_text(s, bx + 80, by + 75, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
        }
        else {
            //int center_x = bx - 30;
            //int center_y = by - 0;
            //int marginX = (int)(rcFont.width() * 3.2f);
            //int marginY = (int)((footer_h / 2 - rcFont.height()) * 0.6f);
            //drawText2(painter, center_x - marginX, center_y - marginY - rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
            //drawText2(painter, center_x + marginX, center_y - marginY - rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
            //drawText2(painter, center_x - marginX, center_y + marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
            //drawText2(painter, center_x + marginX, center_y + marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
            ui_draw_text(s, bx - 90, by - 55, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
            ui_draw_text(s, bx + 90, by - 55, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
            ui_draw_text(s, bx - 90, by + 80, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
            ui_draw_text(s, bx + 90, by + 80, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
        }

    }
}
void DrawApilot::drawDateTime(const UIState* s) {
    char str[128];
    // 시간표시
    if (s->show_datetime) {
        time_t now = time(nullptr);
        struct tm* local = localtime(&now);

        int y = 190;
        int nav_y = y + 50;

        if (s->show_datetime == 1 || s->show_datetime == 2) {
            strftime(str, sizeof(str), "%H:%M", local);
            ui_draw_text(s, 170, y, str, 100, COLOR_WHITE, BOLD, 3.0f, 8.0f);

        }
        if (s->show_datetime == 1 || s->show_datetime == 3) {
            strftime(str, sizeof(str), "%m-%d-%a", local);
            ui_draw_text(s, 170, y + 70, str, 60, COLOR_WHITE, BOLD, 3.0f, 8.0f);
            nav_y += 70;
        }
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        ui_draw_text(s, 50, nav_y, s->m_navText.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    }
}

void DrawApilot::drawAccel(const UIState* s, int x, int y) {
    // Accel표시
    SubMaster& sm = *(s->sm);
    auto car_state = sm["carState"].getCarState();
    float accel = car_state.getAEgo();
    int dx = 128 + 10;
#ifdef __TEST
    static float accel1 = 0.0;
    accel1 += 0.2;
    if (accel1 > 2.5) accel1 = -2.5;
    accel = accel1;
#endif
    if (s->show_accel > 0) {
        ui_draw_rect(s->vg, { x + dx, y + 5, 40, 128 }, COLOR_WHITE, 4, 0);
        ui_fill_rect(s->vg, { x + dx + 2, y + 64 + 5, 36, -(int)(std::clamp((float)accel, -2.0f, 2.0f) / 2. * 64) }, (accel >= 0.0) ? COLOR_YELLOW : COLOR_RED, 0);
        ui_draw_text(s, x + dx + 20, y + 160, "ACC", 25, COLOR_WHITE, BOLD);
    }
}
void DrawApilot::drawRpm(const UIState* s, int x, int y) 
{
    // RPM표시
    SubMaster& sm = *(s->sm);
    auto car_state = sm["carState"].getCarState();
    float engineRpm = car_state.getEngineRpm();
    float motorRpm = car_state.getMotorRpm();
    int dx = 128 + 10;

#ifdef __TEST
    static float engineRpm1 = 0.0;
    engineRpm1 += 100.0;
    if (engineRpm1 > 4000.0) engineRpm1 = 0.0;
    motorRpm = engineRpm1;
#endif
    if (s->show_accel > 1) {
        dx += 60;
        //str.sprintf("%s: %.0f CHARGE: %.0f%%", (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
        //drawTextWithColor(p, width() - 350, 80, str, textColor);
        //painter.setPen(Qt::NoPen);
        ui_draw_rect(s->vg, { x + dx, y + 5, 40, 128 }, COLOR_WHITE, 4, 0);
        ui_fill_rect(s->vg, { x + dx + 2, y + 128 + 5, 36, -(int)(std::clamp((float)engineRpm > 0.0 ? engineRpm : motorRpm, 0.0f, 4000.0f) / 4000. * 128.0) }, (engineRpm > 0.0) ? COLOR_BLUE : COLOR_GREEN, 0);
        ui_draw_text(s, x + dx + 20, y + 160, "RPM", 25, COLOR_WHITE, BOLD);
    }

}
void DrawApilot::drawConnInfo(const UIState* s) {
    SubMaster& sm = *(s->sm);
    auto car_state = sm["carState"].getCarState();
    //const auto car_params = sm["carParams"].getCarParams();
    const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
    int radar_tracks = params.getInt("EnableRadarTracks");
    int sccBus = params.getInt("SccConnectedBus2");
    int activeAPM = road_limit_speed.getActive();
    const auto naviData = sm["naviData"].getNaviData();
    int activeNDA = naviData.getActive();

    static int activeOSM = 0;
    //auto navInstruction = sm["navInstruction"].getNavInstruction();
    //if (navInstruction.getSpeedLimit() > 0 && activeAPM < 200) activeOSM = 100;
    if (false) { //sm.updated("liveMapData")) {
        activeOSM = 100;
    }
    else if (activeOSM > 0) activeOSM--;

    if (s->show_conn_info) {
        //ui_draw_text(s, strlen(str) / 2 * 35 / 2 + 50,40, str, 35, COLOR_WHITE, BOLD);
        int hda_speedLimit = car_state.getSpeedLimit();
        int hda_speedLimitDistance = car_state.getSpeedLimitDistance();
        int naviCluster = 0;// HW: (int)car_params.getNaviCluster();
        int y = 10;
        int x = 30;
        if (sccBus) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_scc2", 1.0f);
            x += 135;
        }
        if (activeAPM >= 200) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apn", 1.0f);
            x += 135;
        }
        else if (hda_speedLimit > 0 && hda_speedLimitDistance > 0) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
            x += 135;
        }
        else if (activeAPM >= 100) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apm", 1.0f);
            x += 135;
        }
        else if (naviCluster > 0) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
            x += 135;
        }
        if (radar_tracks) {
            ui_draw_image(s, { x, y, 240, 54 }, "ic_radartracks", 1.0f);
            x += (120 + 135);
        }
        if (activeOSM > 0) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_osm", 1.0f);
            x += 135;
        }
        if (activeNDA > 0) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_nda", 1.0f);
            x += 135;
        }
    }
}
void DrawApilot::drawGapInfo(const UIState* s, int x, int y) {
    char    str[128];
    float gap = params.getInt("LongitudinalPersonality") + 1;// lp.getCruiseGap();
    int gap1 = gap;// controls_state.getLongCruiseGap(); // car_state.getCruiseGap();
#ifdef __TEST
    drivingMode = 3;
#endif
    char strDrivingMode[128];
    int drivingMode = params.getInt("AccelerationProfile"); //HW: controls_state.getMyDrivingMode();
    if (drivingMode == 0) { // apilot driving mode
        int myDrivingMode = params.getInt("MyDrivingMode");
        switch (myDrivingMode) {
        case 1: strcpy(strDrivingMode, tr("ECO").toStdString().c_str()); break;
        case 2: strcpy(strDrivingMode, tr("SAFE").toStdString().c_str()); break;
        case 3: strcpy(strDrivingMode, tr("NORM").toStdString().c_str()); break;
        case 4: strcpy(strDrivingMode, tr("HIGH").toStdString().c_str()); break;
        default: strcpy(strDrivingMode, tr("ERRM").toStdString().c_str()); break;
        }
    }
    else { // frogpilot mode..
        switch (drivingMode)
        {
        case 1: strcpy(strDrivingMode, "ECO"); break;
        case 2: strcpy(strDrivingMode, "NOR"); break;
        case 3: strcpy(strDrivingMode, "SPT"); break;
        default:
            sprintf(strDrivingMode, "ERR%d", drivingMode); break;
            //strcpy(strDrivingMode, "ERR"); break;
        }
    }

    int dxGap = -128 - 10 - 40;
    if (s->show_gap_info >= 0) {
        //sprintf(str, "%.2f", tFollow);
        //ui_draw_text(s, x + dxGap + 15 - 60, y + 120.0, str, 30, COLOR_WHITE, BOLD);
        //sprintf(str, "%.0fM", tFollow * getVEgo() + 6.0);
        //ui_draw_text(s, x + dxGap + 15 - 60, y + 155.0, str, 30, COLOR_WHITE, BOLD);

        ui_draw_text(s, x + dxGap + 15, y + 120.0, strDrivingMode, 30, COLOR_WHITE, BOLD);
    }
    //static char _strDrivingMode[128] = "";
    //if (strcmp(strDrivingMode, _strDrivingMode)) ui_draw_text_a(s, x + dxGap + 15, y + 120, strDrivingMode, 30, COLOR_WHITE, BOLD);
    //strcpy(_strDrivingMode, strDrivingMode);

    //char strLatControlMode[128] = "";
    //int useLaneLineSpeed = Params().getInt("UseLaneLineSpeed");
    //strcpy(strLatControlMode, (useLaneLineSpeed > 0) ? tr("Lane Follow").toStdString().c_str() : tr("Laneless").toStdString().c_str());
    //static char _strLatControlMode[128] = "";
    //if (strcmp(strLatControlMode, _strLatControlMode)) ui_draw_text_a(s, x + dxGap + 15, y + 120, strLatControlMode, 30, COLOR_WHITE, BOLD);
    //strcpy(_strLatControlMode, strLatControlMode);

    dxGap -= 60;
    if (s->show_gap_info > 0) {
#ifdef __TEST
        static int _gap = 0;
        _gap += 10;
        if (_gap > 400) _gap = 0;
        else if (_gap < 100) gap = 1;
        else if (_gap < 200) gap = 2;
        else if (_gap < 300) gap = 3;
        else gap = 4;

        static int _preGap = 0;
        if (_preGap != gap) {
            sprintf(str, "%d", (int)gap);
            ui_draw_text_a(s, x + dxGap + 15 + 60, y + 60, str, 50, COLOR_WHITE, BOLD);
        }
        _preGap = gap;
#endif
        ui_fill_rect(s->vg, { x + dxGap - 2, (int)(y + 5 + 64), 40, -(int)(std::clamp((float)gap, 0.0f, 4.0f) / 4. * 64) }, COLOR_GREEN, 0);
        ui_draw_rect(s->vg, { x + dxGap, y + 5, 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 1 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 2 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 3 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_text(s, x + dxGap + 20, y + 90, "GAP", 25, COLOR_WHITE, BOLD);
    }
    // 갭정보표시 중앙위
    sprintf(str, "%d", gap1);
    if (s->show_gap_info >= 0) {
        ui_draw_text(s, x + dxGap + 15 + 60, y + 60, str, 50, COLOR_WHITE, BOLD);
    }
    //static int _gap1 = 0;
    //if (_gap1 != gap1) ui_draw_text_a(s, x + dxGap + 15 + 60, y + 60, str, 50, COLOR_WHITE, BOLD);
    //_gap1 = gap1;
}
void DrawApilot::drawGapInfo2(const UIState* s, int x, int y) {
    char    str[128];
    int     gap = params.getInt("LongitudinalPersonality") + 1;// lp.getCruiseGap();
    char    strDrivingMode[128];
    int drivingMode = params.getInt("AccelerationProfile"); //HW: controls_state.getMyDrivingMode();
    if (drivingMode == 0) { // apilot driving mode
        int myDrivingMode = params.getInt("MyDrivingMode");
        switch (myDrivingMode) {
        case 1: strcpy(strDrivingMode, tr("ECO").toStdString().c_str()); break;
        case 2: strcpy(strDrivingMode, tr("SAFE").toStdString().c_str()); break;
        case 3: strcpy(strDrivingMode, tr("NORM").toStdString().c_str()); break;
        case 4: strcpy(strDrivingMode, tr("HIGH").toStdString().c_str()); break;
        default: strcpy(strDrivingMode, tr("ERRM").toStdString().c_str()); break;
        }
    }
    else { // frogpilot mode..
        switch (drivingMode)
        {
        case 1: strcpy(strDrivingMode, "ECO"); break;
        case 2: strcpy(strDrivingMode, "NOR"); break;
        case 3: strcpy(strDrivingMode, "SPT"); break;
        default:
            sprintf(strDrivingMode, "ERR%d", drivingMode); break;
            //strcpy(strDrivingMode, "ERR"); break;
        }
    }

    int dx = x + 50;
    int dy = y + 110;
    ui_draw_text(s, dx, dy, strDrivingMode, 30, COLOR_WHITE, BOLD);
    static char _strDrivingMode[128] = "";
    if (strcmp(strDrivingMode, _strDrivingMode)) ui_draw_text_a(s, dx, dy, strDrivingMode, 30, COLOR_WHITE, BOLD);
    strcpy(_strDrivingMode, strDrivingMode);

    char strLatControlMode[128] = "";
    int useLaneLineSpeedApply = params.getInt("UseLaneLineSpeedApply");
    strcpy(strLatControlMode, (useLaneLineSpeedApply > 0) ? tr("Lane Follow").toStdString().c_str() : tr("Laneless").toStdString().c_str());
    static char _strLatControlMode[128] = "";
    if (strcmp(strLatControlMode, _strLatControlMode)) ui_draw_text_a(s, dx, dy, strLatControlMode, 30, COLOR_WHITE, BOLD);
    strcpy(_strLatControlMode, strLatControlMode);

    dx = x + 220;
    dy = y + 77;
    sprintf(str, "%d", gap);
    ui_draw_text(s, dx, dy, str, 40, COLOR_WHITE, BOLD);
    static int _gap1 = 0;
    if (_gap1 != gap) ui_draw_text_a(s, dx, dy, str, 40, COLOR_WHITE, BOLD);
    _gap1 = gap;
}

void DrawApilot::drawSpeed(const UIState* s, int x, int y) {
    char str[128];
    SubMaster& sm = *(s->sm);
    auto controls_state = sm["controlsState"].getControlsState();
    if (s->show_mode == 3) {
        y = -100;
        if (s->fb_w < 1200) x = 650;
        else x = 950;
    }
    else if (s->show_mode == 4) {
        y = 410;// 380;
        x = 120;// 150;
    }
    else if (s->show_mode == 5) {
        y = 620;
        x = 400;
    }

    // 속도표시
    static float cruiseAdjustment = 0.0;
    if (true) {

        //bool is_metric = s->scene.is_metric;
        //bool long_control = 1;// scc_smoother.getLongControl();

        // kph
        const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        float cruiseMaxSpeed = controls_state.getVCruiseCluster();// scc_smoother.getCruiseMaxSpeed();
        float applyMaxSpeed = controls_state.getVCruise();// HW: controls_state.getVCruiseOut();// scc_smoother.getApplyMaxSpeed();
        float longVCruiseTarget = lp.getVCruiseTarget();
        QString longVCruiseTargetSource = QString::fromStdString(lp.getVCruiseTargetSource().cStr());
        float curveSpeed = 0.0;
        float lpSpeed = lp.getSpeeds()[0] * MS_TO_KPH;//HW: controls_state.getCurveSpeed();
        auto carstate = sm["carState"].getCarState();
        float vCluRatio = carstate.getVCluRatio();
        vCluRatio = (vCluRatio > 0.5) ? vCluRatio : 1.0;
        cruiseAdjustment = lpSpeed / vCluRatio;// *0.1 + cruiseAdjustment * 0.9;
        bool speedCtrlActive = true;
        curveSpeed = cruiseAdjustment;

        auto car_control = sm["carControl"].getCarControl();
        auto cruise_control = car_control.getCruiseControl();
        int longOverride = cruise_control.getOverride();// HW: car_control.getLongOverride();

        int bx = x;
        int by = y + 270;

        if (s->show_mode == 4 || s->show_mode == 5) {
            if (getTrafficMode() == 1) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2 + 270, icon_size, icon_size }, "ic_traffic_red", 1.0f);
            else if (getTrafficMode() == 2) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2 + 270, icon_size, icon_size }, "ic_traffic_green", 1.0f);
        }

        float cur_speed = getVEgo() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
        if (cur_speed < 0.0) cur_speed = 0.0;
        char speed[128];
        sprintf(speed, "%.0f", cur_speed);
        ui_draw_text(s, bx, by + 50, speed, 120, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        ui_draw_image(s, { bx - 100, by - 60, 350, 150 }, "ic_speed_bg", 1.0f);

        //color = QColor(255, 255, 255, 255);
#ifdef __TEST
        cruiseMaxSpeed = 110;
        //longActiveUser = 2;
        applyMaxSpeed = 109;
        curveSpeed = 111;
#endif
        static char _speed_str[128] = "";
        if (isEnabled() && (isLongActive() || (longOverride && isBlinkerOn()))) {
            sprintf(str, "%d", (int)(cruiseMaxSpeed * (s->scene.is_metric ? 1.0 : KM_TO_MILE) + 0.5));
            if (strcmp(_speed_str, str)) ui_draw_text_a(s, bx + 170, by + 15, str, 60, COLOR_GREEN, BOLD);
            strcpy(_speed_str, str);
        }
        else strcpy(str, "--");
        ui_draw_text(s, bx + 170, by + 15, str, 60, COLOR_GREEN, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);

        if (isEnabled() && isLongActive() && applyMaxSpeed > 0) {
            NVGcolor textColor = COLOR_GREEN;
            str[0] = 0;
            char str_source[128] = "";
            static float dispApplyMaxSpeed = 0.0;
                if (longVCruiseTarget < cruiseMaxSpeed - 0.5) {
                    //dispApplyMaxSpeed = dispApplyMaxSpeed * 0.95 + longVCruiseTarget * (s->scene.is_metric ? 1.0 : KM_TO_MILE) * 0.05;
                    dispApplyMaxSpeed = longVCruiseTarget * (s->scene.is_metric ? 1.0 : KM_TO_MILE);
                    sprintf(str, "%d", (int)(dispApplyMaxSpeed + 0.5));
                    strcpy(str_source, longVCruiseTargetSource.toStdString().c_str());
                    textColor = COLOR_OCHRE;
                }
                else if (applyMaxSpeed != cruiseMaxSpeed) {
                    dispApplyMaxSpeed = applyMaxSpeed * (s->scene.is_metric ? 1.0 : KM_TO_MILE);
                    sprintf(str, "%d", (int)(dispApplyMaxSpeed + 0.5));
                }
                else dispApplyMaxSpeed = cruiseMaxSpeed;
            if (strlen(str)) {
                ui_draw_text(s, bx + 250, by - 50, str, 50, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
                if (strlen(str_source)) {
                    ui_draw_text(s, bx + 250, by - 100, str_source, 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
                    //ui_draw_text(s, bx + 250, by - 20, str_source, 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
                }
            }
        }

        if (false) {    // longcontrol speed[0] : display..
            if (isEnabled()) {// && curveSpeed > 0 && curveSpeed < 150) {
                sprintf(str, "%d", (int)(curveSpeed * (s->scene.is_metric ? 1.0 : KM_TO_MILE) + 0.5));
                ui_draw_text(s, bx + 140, by + 110, str, 50, (speedCtrlActive) ? COLOR_ORANGE : COLOR_YELLOW, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            }
        }
        drawGapInfo2(s, bx, by);

        bx = x - 200;
        by = y + 250;
        if (s->show_mode == 4) {
            bx = 150;
            by = 410;// 380;
        }
        else if (s->show_mode == 5) {
            bx = 100;
            by = 650;
        }
        if (s->left_dist > 0) {
            if (s->scene.is_metric) {
                if (s->left_dist < 1000) sprintf(str, "%d m", s->left_dist);
                else  sprintf(str, "%.1f km", s->left_dist / 1000.f);
            }
            else {
                // 1 미터 = 3.28084 피트로 계산
                if (s->left_dist < 1609) sprintf(str, "%d ft", (int)(s->left_dist * 3.28084)); // 1609m(1마일)보다 작으면 피트로 표시
                else sprintf(str, "%.1f mi", s->left_dist / 1609.34f); // 1609m 이상이면 마일로 표시
            }
            ui_draw_text(s, bx, by + 120, str, 40, COLOR_WHITE, BOLD);
        }

        static float left_dist_x = 0.0;
        static float left_dist_y = 0.0;
        static bool left_dist_flag = true;
        if (s->limit_speed <= 0) left_dist_flag = true;

        if (s->limit_speed > 0) {
            if (s->xSignType == 124 || s->camType == 22) {
                ui_draw_image(s, { bx - 60, by - 50, 120, 150 }, "ic_speed_bump", 1.0f);
            }
            else {
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 140 / 2);
                nvgFillColor(s->vg, COLOR_WHITE);
                nvgFill(s->vg);
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 130 / 2);
                nvgFillColor(s->vg, COLOR_RED);
                nvgFill(s->vg);
                nvgBeginPath(s->vg);
                nvgCircle(s->vg, bx, by, 110 / 2);
                nvgFillColor(s->vg, COLOR_WHITE);
                nvgFill(s->vg);
                sprintf(str, "%d", s->limit_speed);
                ui_draw_text(s, bx, by + 25, str, 60, COLOR_BLACK, BOLD, 0.0f, 0.0f);
            }
            if(true) {
                float scale = 0.2;
                if (s->left_dist < 200) scale = 1.0 - (0.8 * s->left_dist / 200.);
                bx = s->left_dist_point.x() + 140 * scale;
                by = s->left_dist_point.y();
                if (left_dist_flag) {
                    left_dist_x = bx;
                    left_dist_y = by;
                }
                else {
                    left_dist_x = left_dist_x * 0.9 + bx * 0.1;
                    left_dist_y = left_dist_y * 0.9 + by * 0.1;
                }

                bx = left_dist_x;
                by = left_dist_y;             
                left_dist_flag = false;

                if (s->left_dist > 0) {

                    if (s->scene.is_metric) {
                        if (s->left_dist < 1000) sprintf(str, "%d m", s->left_dist);
                        else  sprintf(str, "%.1f km", s->left_dist / 1000.f);
                    }
                    else {
                        // 1 미터 = 3.28084 피트로 계산
                        if (s->left_dist < 1609) sprintf(str, "%d ft", (int)(s->left_dist * 3.28084)); // 1609m(1마일)보다 작으면 피트로 표시
                        else sprintf(str, "%.1f mi", s->left_dist / 1609.34f); // 1609m 이상이면 마일로 표시
                    }
                    ui_draw_text(s, bx, by + 120*scale, str, 40*scale, COLOR_WHITE, BOLD);
                }

                if (s->xSignType == 124 || s->camType == 22) {
                    ui_draw_image(s, { bx - (int)(60*scale), by - (int)(50*scale), (int)(120*scale), (int)(150*scale) }, "ic_speed_bump", 1.0f);
                }
                else {
                    nvgBeginPath(s->vg);
                    nvgCircle(s->vg, bx, by, 140 / 2 * scale);
                    nvgFillColor(s->vg, COLOR_WHITE);
                    nvgFill(s->vg);
                    nvgBeginPath(s->vg);
                    nvgCircle(s->vg, bx, by, 130 / 2 * scale);
                    nvgFillColor(s->vg, COLOR_RED);
                    nvgFill(s->vg);
                    nvgBeginPath(s->vg);
                    nvgCircle(s->vg, bx, by, 110 / 2 * scale);
                    nvgFillColor(s->vg, COLOR_WHITE);
                    nvgFill(s->vg);
                    sprintf(str, "%d", s->limit_speed);
                    ui_draw_text(s, bx, by + 25 * scale - 6*(1-scale), str, 60 * scale, COLOR_BLACK, BOLD, 0.0f, 0.0f);
                }
            }
        }
        else if (s->xTurnInfo >= 0) {
            switch (s->xTurnInfo) {
            case 1: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f); break;
            case 2: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f); break;
            case 3: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f); break;
            case 4: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f); break;
            case 5:
                if(s->xNavModifier == "uturn") ui_draw_image(s, {bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size}, "ic_turn_u", 1.0f);
                else {
                    sprintf(str, "%s", (s->xNavModifier.length()>0)?s->xNavModifier.toStdString().c_str() : "unknown");
                    ui_draw_text(s, bx, by + 20, str, 35, COLOR_WHITE, BOLD);
                }
                break;
            case 6: ui_draw_text(s, bx, by + 20, "TG", 35, COLOR_WHITE, BOLD); break;
            case 35: ui_draw_text(s, bx, by + 20, "좌측고가 진입", 35, COLOR_WHITE, BOLD, 0.0f, 0.0f); break;
            case 43: ui_draw_text(s, bx, by + 20, "지하차도 우측", 35, COLOR_WHITE, BOLD, 0.0f, 0.0f); break;
            case 48: ui_draw_text(s, bx, by + 20, "휴게소", 35, COLOR_WHITE, BOLD, 0.0f, 0.0f); break;
            default:
                sprintf(str, "%d", s->xTurnInfo);
                ui_draw_text(s, bx, by + 20, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
                break;
            }
        }
        else if (s->roadLimitSpeed >= 30 && s->roadLimitSpeed < 200) {
            ui_draw_image(s, { bx - 60, by - 50, 120, 150 }, "ic_road_speed", 1.0f);
            sprintf(str, "%d", s->roadLimitSpeed);
            ui_draw_text(s, bx, by + 75, str, 50, COLOR_BLACK, BOLD, 0.0f, 0.0f);
        }        
    }
}
void DrawApilot::drawTurnInfo(const UIState* s, int x, int y) {
    SubMaster& sm = *(s->sm);
    //auto lateralPlan = sm["lateralPlan"].getLateralPlan();
    auto desireEvent = 0;//HW: lateralPlan.getDesireEvent();
    auto meta = sm["modelV2"].getModelV2().getMeta();
    auto laneChangeDirection = meta.getLaneChangeDirection();
    auto laneChangeState = meta.getLaneChangeState();
    float desireStateTurnLeft = meta.getDesireState()[1];
    float desireStateTurnRight = meta.getDesireState()[2];
    float desireStateLaneChangeLeft = meta.getDesireState()[3];
    float desireStateLaneChangeRight = meta.getDesireState()[4];

    auto car_state = sm["carState"].getCarState();
    auto controls_state = sm["controlsState"].getControlsState();
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    bool leftBlinker = car_state.getLeftBlinker() || (controls_state.getLeftBlinkerExt()%10000 > 0) || (lp.getLeftBlinkerExt()%10000 > 0);
    bool rightBlinker = car_state.getRightBlinker() || (controls_state.getRightBlinkerExt()%10000 > 0) || (lp.getRightBlinkerExt() % 10000 > 0);
    bool bsd_l = car_state.getLeftBlindspot();
    bool bsd_r = car_state.getRightBlindspot();

    // 차로변경, 턴 표시~
    if (true) {
        if (desireStateTurnLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f);
        else if (desireStateTurnRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f);
        else if (desireStateLaneChangeLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
        else if (desireStateLaneChangeRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);

        if (desireEvent == 57) {
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
        }
        else if (desireEvent == 58) {
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
            ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
        }
        else if (desireEvent == 71) {
            if (laneChangeDirection == cereal::LaneChangeDirection::LEFT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            }
            else if (laneChangeDirection == cereal::LaneChangeDirection::RIGHT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            }
        }
        if (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) {
            if (laneChangeDirection == cereal::LaneChangeDirection::LEFT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            }
            else if (laneChangeDirection == cereal::LaneChangeDirection::RIGHT) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            }
        }
    }
    // blinker 표시~~
    if (true) {
        if (rightBlinker && isBlinkerOn()) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_r", 1.0f);
        if (leftBlinker && isBlinkerOn()) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_l", 1.0f);
    }
    // BSD 표시
    if (false) {
        if (bsd_l) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_bsd_l", 1.0f);
        if (bsd_r) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_bsd_r", 1.0f);
    }   

    static float navi_turn_point_x[2] = { 0.0, };
    static float navi_turn_point_y[2] = { 0.0, };
    static bool navi_turn_point_flag = true;
    if (s->xDistToTurn < 1500 && s->xDistToTurn > 0) {
        float scale = 0.2;
        if (s->xDistToTurn < 200) scale = 1.0 - (0.8 * s->xDistToTurn / 200.);
        scale *= 0.5;
        int size_x = 348 * scale;
        int size_y = 540 * scale;
        int img_x = 0;
        int img_y = 0;
        float alpha = (navi_turn_point_flag)?1.0:0.1;

        for (int i = 0; i < 2; i++) {
            navi_turn_point_x[i] = navi_turn_point_x[i] * (1.0 - alpha) + s->navi_turn_point[i].x() * alpha;
            navi_turn_point_y[i] = navi_turn_point_y[i] * (1.0 - alpha) + s->navi_turn_point[i].y() * alpha;
        }
        navi_turn_point_flag = false;

        switch (s->xTurnInfo) {
        case 1: case 3: case 5:
            img_x = (int)navi_turn_point_x[0] - size_x / 2;
            img_y = (int)navi_turn_point_y[0] - size_y;
            ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
            break;
        case 2: case 4:
            img_x = (int)navi_turn_point_x[1] - size_x / 2;
            img_y = (int)navi_turn_point_y[1] - size_y;
            ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
            break;
        }
    }
    else navi_turn_point_flag = true;

}
void DrawApilot::drawSteer(const UIState* s, int x, int y) {
    char str[128];
    // steer handle 그리기..
    bool showBg = false; // (disp_dist > 0.0) ? true : false;
    bool    uiDrawSteeringRotate = s->show_steer_rotate;
    SubMaster& sm = *(s->sm);
    auto car_state = sm["carState"].getCarState();
    float steer_angle = car_state.getSteeringAngleDeg();
#ifdef __TEST
    float radar_rel_speed = 20.0;
    if (test_seq > 50) radar_rel_speed = -20.0;
#else
    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
    float radar_rel_speed = lead_radar.getVRel();
#endif
    //float disp_dist = (isRadarDetected()) ? getRadarDist() : getVisionDist();
    NVGcolor bgColor = nvgRGBA(0, 0, 0, 166);

#ifdef __TEST
    static float steer_ang = 0.0;
    steer_ang += 1.0;
    steer_angle = steer_ang;
#endif
#if 0
    if (s->show_steer_mode == 2) {
        switch (getTrafficMode()) {
        case 1: ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_traffic_red", 1.0f); break;
        case 2: ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_traffic_green", 1.0f); break;
        }
    }
#endif

    if (s->show_steer_mode == 1) {
        if (uiDrawSteeringRotate) {      // 시간이 많이(3msec)걸려 3번에 한번씩만 그리자..
            switch (getTrafficMode()) {
            case 0: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_momo", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
            case 1: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_red", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
            case 2: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_green", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
            case 3: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_yellow", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
            }
        }
        else ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_steer_momo", 0.7f);
        bgColor = nvgRGBA(0, 0, 0, 0);
    }
    else if (s->show_steer_mode == 0) {
        if (uiDrawSteeringRotate) {
            ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_momo", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle);
        }
        else ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_steer_momo", 0.7f);
        switch (getTrafficMode()) {
        case 0: bgColor = nvgRGBA(0, 0, 0, 0); break;
        case 1: bgColor = nvgRGBA(255, 0, 0, 100); break;
        case 2: bgColor = nvgRGBA(0, 255, 0, 100); break;
        case 3: bgColor = nvgRGBA(255, 255, 0, 100); break;
        }
    }
    else {
        showBg = false;
    }
    if (showBg) {
        nvgBeginPath(s->vg);
        nvgCircle(s->vg, x, y, 160 / 2.);
        nvgFillColor(s->vg, bgColor);
        nvgFill(s->vg);

    }
    NVGcolor textColor = COLOR_WHITE;
    NVGcolor borderColor = COLOR_BLACK;

    if (isRadarDetected()) {
        sprintf(str, "%.1f %s", (getVEgo()*MS_TO_KPH + radar_rel_speed * MS_TO_KPH) * (s->scene.is_metric ? 1.0 : KM_TO_MILE), s->scene.is_metric ? "km/h" : "mile");
        if (radar_rel_speed < -0.1) {
            textColor = COLOR_RED;
            borderColor = COLOR_BLACK;
        }
        else {
            textColor = COLOR_GREEN;
            borderColor = COLOR_BLACK;
        }

        nvgFontSize(s->vg, 40);
    }
    if (s->show_mode >= 2) {
        if (isLeadDetected() && s->show_path_end > 1) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, (isRadarDetected()) ? "ic_radar" : "ic_radar_vision", 1.0f);
    }
    else ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, (!isLeadDetected()) ? "ic_radar_no" : (isRadarDetected()) ? "ic_radar" : "ic_radar_vision", 1.0f);
}
void DrawApilot::makePathXY(const UIState* s, int& path_bx1, int& path_x, int& path_y, int& path_width) {
    const UIScene& scene = s->scene;

    static float path_fx = s->fb_w / 2;
    static float path_fy = s->fb_h - 400;
    static float path_fwidth = 160;
    static float path_bx = s->fb_w / 2;
    float alpha = 0.85;

    int len = scene.track_vertices.length();
    //int hlen = len / 2;
    float _path_x = path_fx;
    float _path_y = path_fy;
    float _path_width = path_fwidth;

    float lex = scene.path_end_left_vertices[0].x();
    float rex = scene.path_end_right_vertices[0].x();
    float ley = scene.path_end_left_vertices[0].y();
    float rey = scene.path_end_right_vertices[0].y();
    _path_width = rex - lex;
    _path_x = (lex + rex) / 2.;
    _path_y = (ley + rey) / 2.;

    if (len >= 2) {
        float lbx = scene.track_vertices[0].x();
        //float lby = scene.track_vertices[0].y();
        float rbx = scene.track_vertices[len - 1].x();
        //float rby = scene.track_vertices[len - 1].y();
        //float lex = scene.track_vertices[hlen - 1].x();
        //float ley = scene.track_vertices[hlen - 1].y();
        //float rex = scene.track_vertices[hlen].x();
        //float rey = scene.track_vertices[hlen].y();
        //_path_width = rex - lex;
        //_path_x = (lex + rex) / 2.;
        //_path_y = (ley + rey) / 2.;
        path_bx = path_bx * alpha + (lbx + rbx) / 2. * (1. - alpha);
    }
    //if (radar_detected) {
    //    _path_x = (int)std::clamp((float)vd.x(), 550.f, s->fb_w - 550.f);
    //    _path_y = (int)std::clamp((float)vd.y(), 200.f, s->fb_h - 100.f);
    //}
    //_path_x = (int)std::clamp(_path_x, 550.f, s->fb_w - 550.f);
    _path_x = (int)std::clamp(_path_x, 350.f, s->fb_w - 350.f);
    _path_y = (int)std::clamp(_path_y, 200.f, s->fb_h - 80.f);
    if (isnan(_path_x) || isnan(_path_y) || isnan(_path_width));
    else {
        path_fx = path_fx * alpha + _path_x * (1. - alpha);
        path_fy = path_fy * alpha + _path_y * (1. - alpha);

        if (_path_width < 120.) _path_width = 120.;
        else if (_path_width > 800.) _path_width = 800.;
        path_fwidth = path_fwidth * alpha + _path_width * (1. - alpha);
    }

    //printf("%.0f, %.0f, %.0f, %.0f, %.0f\n", _path_x, _path_y, path_fx, path_fy, path_fwidth);

    path_x = (int)path_fx;
    path_y = (int)path_fy;
    path_width = (int)path_fwidth;
    path_bx1 = (int)path_bx;

    //if(s->show_path_end>0) ui_fill_rect(s->vg, { path_x - path_width / 2, path_y, path_width, -4 }, COLOR_RED, 5);

    // 과녁을 표시할 위치를 계산
#ifdef __TEST
    //const int d_rel = 0;
    if (++test_seq > 100) test_seq = 0;
#else
    //const int d_rel = lead_data.getX()[0];
#endif
}
void DrawApilot::drawPathEnd(const UIState* s, int x, int y, int path_x, int path_y, int path_width) {
    char    str[128];
    SubMaster& sm = *(s->sm);
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    auto xState = lp.getXState();
    float disp_size = (s->show_path_end == 1) ? 60.0 : 45.0;
    //int disp_y = y + 120;
    int disp_y = y + 195;// 175;
    bool draw_dist = false;
    disp_size = 50;
    if (isBrakeHold() || isSoftHold()) {
        sprintf(str, "%s", (isBrakeHold()) ? "AUTOHOLD" : "SOFTHOLD");
        ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
    }
    else if (isLongActive()) {
        if (xState == 3 || xState == 5) {      //XState.e2eStop, XState.e2eStopped
            if (getVEgo() < 1.0) {
                sprintf(str, "%s", (lp.getTrafficState() >= 1000) ? "신호오류" : "신호대기");
                ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
            }
            else if (getStopDist() > 0.5) {
                float dist = getStopDist() * (s->scene.is_metric ? 1 : METER_TO_FOOT);
                if (dist < 10.0) sprintf(str, "%.1fM", dist);
                else sprintf(str, "%.0fM", dist);
                ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
            }
        }
        else if (xState == 0) {     //XState.lead
            draw_dist = true;
        }
    }
    else draw_dist = true;

    if (s->show_path_end > 0) {
        if (draw_dist) {
            //float dist = (getRadarDist() > 0.0) ? getRadarDist() : getVisionDist();
            //if (dist < 10.0) sprintf(str, "%.1f", dist);
            //else sprintf(str, "%.0f", dist);
            //ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
            int wStr = 0, w=80;
            float dist = getRadarDist() * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x - w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, isLeadSCC() ? COLOR_RED : COLOR_ORANGE, 15);
                ui_draw_text(s, x - w, disp_y, str, 40, COLOR_WHITE, BOLD);
            }
            dist = getVisionDist() * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x + w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, COLOR_BLUE, 15);
                ui_draw_text(s, x + w, disp_y, str, 40, COLOR_WHITE, BOLD);
            }
        }
        if (!isLeadDetected()) {
            sprintf(str, "%d", getLpSource());
            ui_draw_text(s, x, disp_y - 70, str, 25, COLOR_WHITE, BOLD);
        }
    }

    // 타겟하단: 롱컨상태표시
    if (true) {
        QString qstr;
        static QString _qstr = "";
        if (isBrakeHold()) qstr = "AUTOHOLD";
        else if (isSoftHold()) qstr = "SOFTHOLD";
        else if (isLongActive()) qstr = "CRUISE"; // qstr = (xState == 3) ? "STOPPING" : (xState == 4) ? "STARTING" : "CRUISE";
        else qstr = "MANUAL";
        if (qstr != _qstr) ui_draw_text_a(s, x, y + 175, qstr.toStdString().c_str(), 40, COLOR_WHITE, BOLD);
        _qstr = qstr;

    }

    if (true) { 

        float px[7], py[7];
        NVGcolor rcolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
        NVGcolor  pcolor = !isRadarDetected() ? ((getTrafficMode() == 1) ? rcolor : COLOR_GREEN) : isRadarDetected() ? rcolor : COLOR_BLUE;
        if (s->show_path_end && !isLeadDetected()) {
            px[0] = path_x - path_width / 2;
            px[1] = path_x + path_width / 2;
            px[2] = path_x + path_width / 2;
            px[3] = path_x + 20;
            px[4] = path_x;
            px[5] = path_x - 20;
            px[6] = path_x - path_width / 2;
            py[0] = path_y;
            py[1] = path_y;
            py[2] = path_y - 7;
            py[3] = path_y - 17;
            py[4] = path_y - 0;
            py[5] = path_y - 17;
            py[6] = path_y - 7;
            ui_draw_line2(s, px, py, 7, &pcolor, nullptr, 3.0f);
        }
        if (isLeadDetected()) {
            px[0] = path_x - path_width / 2 - 10;
            px[1] = px[0] + path_width + 20;
            px[2] = px[1];
            px[3] = px[0];
            py[0] = path_y + 20;// 5;
            py[1] = py[0];
            py[2] = py[1] - path_width * 0.8;
            py[3] = py[2];
            NVGcolor color2 = COLOR_BLACK_ALPHA(20);
            ui_draw_line2(s, px, py, 4, &color2, nullptr, 3.0f, isRadarDetected() ? rcolor : COLOR_BLUE);

            auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
            float h = path_width * 0.8 / 2.;
            float ax[4], ay[4], vx[4], vy[4];
            vx[0] = px[0];
            vx[1] = vx[0] + 20;
            vx[2] = vx[1];
            vx[3] = vx[0];
            vy[0] = py[0] - h;
            vy[1] = vy[0];
            float v = lead_radar.getVRel() / 5.0 * h;
            if (v < -h) v = -h;
            if (v > h) v = h;
            vy[2] = vy[1] - v;
            vy[3] = vy[2];
            NVGcolor vcolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
            ui_draw_line2(s, vx, vy, 4, &vcolor, nullptr, 0.0f);

            ax[0] = px[1];
            ax[1] = px[1] - 20;
            ax[2] = ax[1];
            ax[3] = ax[0];
            ay[0] = py[0] - h;
            ay[1] = ay[0];
            float a = lead_radar.getALeadK() / 2.0 * h;
            if (a < -h) a = -h;
            if (a > h) a = h;
            ay[2] = ay[1]  - a;
            ay[3] = ay[2];
            NVGcolor acolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
            ui_draw_line2(s, ax, ay, 4, &acolor, nullptr, 0.0f);
        }
    }
}
void DrawApilot::makeData(const UIState* s) {
    makeLeadData(s);
    SubMaster& sm = *(s->sm);
    auto controls_state = sm["controlsState"].getControlsState();
    auto car_state = sm["carState"].getCarState();
    auto car_control = sm["carControl"].getCarControl();
    auto hud_control = car_control.getHudControl();

    m_enabled = controls_state.getEnabled();
    m_longActiveUser = controls_state.getEnabled();// LongActiveUser();
    m_longActiveUserReady = controls_state.getEnabled();// getLongActiveUserReady();
    m_blinkerTimer = (m_blinkerTimer + 1) % 16;

    m_brakeHoldActive = car_state.getBrakeHoldActive();
    m_softHoldActive = hud_control.getSoftHold();
    m_experimentalMode = controls_state.getExperimentalMode() ? 1 : 0;

    m_vEgo = car_state.getVEgoCluster();

    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    if (!isLongActive()) m_trafficMode = 0;  // 크루즈가 꺼져있으면... 신호등을 모두 꺼버려?
    else if (lp.getTrafficState() >= 100) m_trafficMode = 3; // yellow
    else {
        switch (lp.getTrafficState() % 100) {
        case 0: m_trafficMode = 0; break;
        case 1: m_trafficMode = 1; break;   // red
        case 2: m_trafficMode = 2; break;  // green // 표시안함.
        case 3: m_trafficMode = 3; break; // yellow
        }
    }
}

void DrawApilot::drawLeadApilot(const UIState* s) {
    SubMaster& sm = *(s->sm);
    //const UIScene& scene = s->scene;

#ifndef __TEST
    if (!sm.alive("controlsState") || !sm.alive("radarState") || !sm.alive("carControl")) {
        //printf("not ready....\n");
        return;
    }
    if (!sm.alive("lateralPlan") || !sm.alive("longitudinalPlan") || !sm.alive("liveParameters") || !sm.alive("roadLimitSpeed") || !sm.alive("liveTorqueParameters")) {
        //printf("not ready 2....\n");
        return;
    }
#endif
    makeData(s);


    nvgFontFace(s->vg, BOLD);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);

    drawPlot.draw(s);
    drawRadarInfo(s);

    drawBackground(s);

    int path_bx = 0;
    int path_x = 0;
    int path_y = 0;
    int path_width = 0;
    makePathXY(s, path_bx, path_x, path_y, path_width);

    int x = path_x;
    int y = path_y;
    y = path_y - 135;
    if (s->show_mode == 1 || s->show_mode == 2) {
        if (y > s->fb_h - 400) y = s->fb_h - 400;
    }

    if (s->show_mode == 2) {
        y = s->fb_h - 400;
        x = path_bx;
    }

    if(s->show_mode==1) 
        x = std::clamp((float)x, 550.f, s->fb_w - 550.f);

    //if ((desireStateTurnLeft > 0.5) || (desireStateTurnRight > 0.5) || (desireStateLaneChangeLeft > 0.5) || (desireStateLaneChangeRight > 0.5)) {
    //    showBg = true;
    // }

    drawSteer(s, x, y);
    drawTurnInfo(s, x, y);

    static float tf_distance_x = 0.0, tf_distance_y = 0.0;
    tf_distance_x = tf_distance_x * 0.9 + s->tf_distance_point.x() * 0.1;
    tf_distance_y = tf_distance_y * 0.9 + s->tf_distance_point.y() * 0.1;
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, tf_distance_x, tf_distance_y, 20 / 2);
    nvgFillColor(s->vg, COLOR_RED);
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, tf_distance_x - 60, tf_distance_y, 20 / 2);
    nvgFillColor(s->vg, COLOR_RED);
    nvgFill(s->vg);
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, tf_distance_x + 60, tf_distance_y, 20 / 2);
    nvgFillColor(s->vg, COLOR_RED);
    nvgFill(s->vg);

    drawPathEnd(s, x, y, path_x, path_y, path_width);
    drawGapInfo(s, x, y);
    drawAccel(s, x, y);
    drawRpm(s, x, y);

    drawSpeed(s, x, y);
    drawConnInfo(s);
    drawTPMS(s);
    drawDateTime(s);
}

#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
char ip_address[64] = "";
QString gitBranch = "branch";
char *read_ip_address() {
    int     fd;
    struct ifreq ifr;

    const char* iface = "wlan0";
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    ioctl(fd, SIOCGIFADDR, &ifr);
    close(fd);

    strcpy(ip_address, inet_ntoa(((struct sockaddr_in*)&ifr.ifr_addr)->sin_addr));
    printf("ip_address= %s\n", ip_address);

    return ip_address;
}
void DrawApilot::drawDeviceState(UIState* s, bool show) {
    const SubMaster& sm = *(s->sm);
    auto deviceState = sm["deviceState"].getDeviceState();
    char  str[128];
    const auto freeSpacePercent = deviceState.getFreeSpacePercent();
    const auto memoryUsagePercent = deviceState.getMemoryUsagePercent();

    const auto cpuTempC = deviceState.getCpuTempC();
    //const auto gpuTempC = deviceState.getGpuTempC();
    //float ambientTemp = deviceState.getAmbientTempC();
    float cpuTemp = 0.f;
    //float gpuTemp = 0.f;

    static int read_ip_count = 60;
    if (read_ip_count == 60) {
        read_ip_address();
        gitBranch = QString::fromStdString(params.get("GitBranch"));
    }
    if (read_ip_count-- < 0) read_ip_count = 60;
    nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);

    if (std::size(cpuTempC) > 0) {
        for (int i = 0; i < std::size(cpuTempC); i++) {
            cpuTemp += cpuTempC[i];
        }
        cpuTemp = cpuTemp / (float)std::size(cpuTempC);
    }
    auto car_state = sm["carState"].getCarState();
    //const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
    sprintf(str, "MEM: %d%% STORAGE: %.0f%% CPU: %.0f°C", memoryUsagePercent, freeSpacePercent, cpuTemp);
    int r = interp<float>(cpuTemp, { 50.f, 90.f }, { 200.f, 255.f }, false);
    int g = interp<float>(cpuTemp, { 50.f, 90.f }, { 255.f, 200.f }, false);
    NVGcolor textColor = nvgRGBA(r, g, 200, 255);
    float engineRpm = car_state.getEngineRpm();
    float motorRpm = car_state.getMotorRpm();
    if (s->fb_w > 1200 && show) {
        ui_draw_text(s, s->fb_w - 20, 40, str, 35, textColor, BOLD);
        sprintf(str, "FPS: %d, %s: %.0f CHARGE: %.0f%%                           ", g_fps, (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
        ui_draw_text(s, s->fb_w - 20, 85, str, 35, textColor, BOLD);
    }
    //nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
    //ui_draw_text(s, s->fb_w - 20, s->fb_h - 15, (read_ip_count < 30) ? ip_address : gitBranch.toStdString().c_str(), 30, COLOR_WHITE, BOLD);

#if 0
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    const auto lat_plan = sm["lateralPlan"].getLateralPlan();
    QString latDebugText = QString::fromStdString(lat_plan.getLatDebugText());
    if(s->show_debug>0)
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 50, latDebugText.toStdString().c_str(), 30, COLOR_WHITE, BOLD);
    //float laneWidth = lat_plan.getLaneWidth();
    //float laneWidthLeft = lat_plan.getLaneWidthLeft();
    //float laneWidthRight = lat_plan.getLaneWidthRight();
    //sprintf(str, "%3.1fm    %3.1fm    %3.1fm", laneWidthLeft, laneWidth, laneWidthRight);
    //ui_draw_text(s, s->fb_w / 2, s->fb_h - 50, str, 30, COLOR_WHITE, BOLD);

    auto meta = sm["modelV2"].getModelV2().getMeta();
    QString debugModelV2 = QString::fromStdString(meta.getDebugText().cStr());
    auto controls_state = sm["controlsState"].getControlsState();
    QString debugControlsState = QString::fromStdString(controls_state.getDebugText1().cStr());
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    QString debugLong2 = QString::fromStdString(lp.getDebugLongText2().cStr());
    if (s->show_debug > 0) {
        if (debugModelV2.length() > 2) {
            ui_draw_text(s, s->fb_w / 2, s->fb_h - 15, debugModelV2.toStdString().c_str(), 30, COLOR_WHITE, BOLD);
        }
        else if (debugLong2.length() > 2) {
            ui_draw_text(s, s->fb_w / 2, s->fb_h - 15, debugLong2.toStdString().c_str(), 30, COLOR_WHITE, BOLD);
        }
        else if (debugControlsState.length() > 2) {
            ui_draw_text(s, s->fb_w / 2, s->fb_h - 15, debugControlsState.toStdString().c_str(), 30, COLOR_WHITE, BOLD);
        }
        else if (s->fb_w > 1200 && show) {
            sprintf(str, "MEM: %d%% DISK: %.0f%% CPU: %.0f°C FPS: %d, %s: %.0f BATTERY: %.0f%%", memoryUsagePercent, freeSpacePercent, cpuTemp, g_fps, (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
            ui_draw_text(s, s->fb_w / 2, s->fb_h - 15, str, 30, COLOR_WHITE, BOLD);
        }


        const auto live_params = sm["liveParameters"].getLiveParameters();
        float   liveSteerRatio = live_params.getSteerRatio();
        QString debugLong = QString::fromStdString(lp.getDebugLongText().cStr());
        debugLong += (" LiveSR:" + QString::number(liveSteerRatio, 'f', 2));
        ui_draw_text(s, s->fb_w / 2, 30, debugLong.toStdString().c_str(), 30, COLOR_WHITE, BOLD);
    }
#endif

}
void DrawApilot::drawDebugText(UIState* s, bool show) {
    if (s->fb_w < 1200) return;
    if (show == false) return;
    const SubMaster& sm = *(s->sm);
    char  str[128];
    QString qstr;

    nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);

    int y = 450, dy = 40;

    const int text_x = s->fb_w - 220;
    const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
    
    sprintf(str, "LT[%.0f]:%s (%.4f/%.4f)", live_torque_params.getTotalBucketPoints(), live_torque_params.getLiveValid() ? "ON" : "OFF", live_torque_params.getLatAccelFactorFiltered(), live_torque_params.getFrictionCoefficientFiltered());
    ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);

    //qstr = QString::fromStdString(live_torque_params.getDebugText().cStr());
    //y += dy;
    //ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);

    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    //qstr = QString::fromStdString(lp.getDebugLongText().cStr());
    //y += dy;
    //ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
    qstr = QString::fromStdString(lp.getDebugLongText2().cStr());
    y += dy;
    ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);

    //const auto live_params = sm["liveParameters"].getLiveParameters();
    //float   liveSteerRatio = live_params.getSteerRatio();
    //sprintf(str, "LiveSR = %.2f", liveSteerRatio);
    //y += dy;
    //ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);

    auto controls_state = sm["controlsState"].getControlsState();
    qstr = QString::fromStdString(controls_state.getDebugText1().cStr());
    y += dy;
    ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
    qstr = QString::fromStdString(controls_state.getDebugText2().cStr());
    y += dy;
    ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);


#if 0
    const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
    bool navEnabled = model.getNavEnabled();
    auto meta = sm["modelV2"].getModelV2().getMeta();
    const char* desireStr[7] = { "None", "TurnLeft", "TurnRight", "ChangeLeft", "ChangeRight", "KeepLeft", "KeepRight" };
    float desireState[7];
    int     desireActive = 0;
    for (int i = 0; i < 7; i++) {
        desireState[i] = meta.getDesireState()[i];
        if (desireState[i] > 0.5) desireActive = i;
    }
    sprintf(str, "NOO:%d, %10s(%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f)", navEnabled, desireStr[desireActive], desireState[0], desireState[1], desireState[2], desireState[3], desireState[4], desireState[5], desireState[6]);
    y += dy;
    ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);


    auto instruction = sm["navInstruction"].getNavInstruction();
    float distance = instruction.getManeuverDistance();
    float distance_remaining = instruction.getDistanceRemaining();
    QString type = QString::fromStdString(instruction.getManeuverType());
    QString modifier = QString::fromStdString(instruction.getManeuverModifier());
    QString text2 = QString::fromStdString(instruction.getManeuverSecondaryText());
    sprintf(str,"[%s], %.1f/%.1f %s(%s)", text2.toStdString().c_str(), distance, distance_remaining, type.length() ? type.toStdString().c_str() : "", modifier.length() ? modifier.toStdString().c_str() : "");
    y += dy;
    ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
#endif
    //const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
    //int xTurnInfo = road_limit_speed.getXTurnInfo();
    //int xDistToTurn = road_limit_speed.getXDistToTurn();
    //int xSpdDist = road_limit_speed.getXSpdDist();
    //int xSpdLimit = road_limit_speed.getXSpdLimit();
    //int xSignType = road_limit_speed.getXSignType();
    //int xRoadSignType = road_limit_speed.getXRoadSignType();
    //int xRoadLimitSpeed = road_limit_speed.getXRoadLimitSpeed();

#if 0
    auto lateralPlan = sm["lateralPlan"].getLateralPlan();
    float laneWidth = lateralPlan.getLaneWidth();
    //int roadEdgeStat = lateralPlan.getRoadEdgeStat();
    QString latDebugText = QString::fromStdString(lateralPlan.getLatDebugText());


    //sprintf(str, "Mappy: Turn(%d,%d), Spd(%d,%d),Sign(%d), Road(%d,%d), LW:%.1f", xTurnInfo, xDistToTurn, xSpdDist, xSpdLimit, xSignType, xRoadSignType, xRoadLimitSpeed, laneWidth);
    //y += dy;
    //ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
    //sprintf(str, "LW:%.1f, Edge(%d), %s", laneWidth, roadEdgeStat, latDebugText.toStdString().c_str());
    sprintf(str, "LW:%.1f, %s", laneWidth, latDebugText.toStdString().c_str());
    y += dy;
    ui_draw_text(s, text_x, y, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
#endif
    //p.drawText(text_x, y + 160, QString::fromStdString(controls_state.getDebugText2().cStr()));
    //p.drawText(text_x, y + 240, QString::fromStdString(controls_state.getDebugText1().cStr()));

    auto car_control = sm["carControl"].getCarControl();
    qstr = QString::fromStdString(car_control.getDebugTextCC().cStr());
    y += dy;
    ui_draw_text(s, text_x, y, qstr.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
}
DrawApilot::DrawApilot() {

}
const char* class_names_[] = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
    "pottedplant", "bed", "diningtable", "toilet", "TV monitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};
const char* class_names_kr[] = {
    "사람", "자전거", "자동차", "오토바이", "비행기", "버스", "기차", "트럭",
    "보트", "신호등", "소화전", "정지 신호", "주차 요금기", "벤치",
    "새", "고양이", "개", "말", "양", "소", "코끼리", "곰", "얼룩말",
    "기린", "배낭", "우산", "핸드백", "넥타이", "여행 가방", "프리스비",
    "스키", "스노보드", "스포츠 공", "연", "야구 방망이", "야구 글러브",
    "스케이트보드", "서핑보드", "테니스 라켓", "병", "와인 잔", "컵",
    "포크", "나이프", "숟가락", "그릇", "바나나", "사과", "샌드위치", "오렌지",
    "브로콜리", "당근", "핫도그", "피자", "도넛", "케이크", "의자", "소파",
    "화분", "침대", "식탁", "화장실", "TV 모니터", "노트북", "마우스",
    "리모컨", "키보드", "휴대폰", "전자레인지", "오븐", "토스터", "싱크대",
    "냉장고", "책", "시계", "꽃병", "가위", "테디 베어", "헤어 드라이어",
    "칫솔"
};
const char* class_names[] = {
    "green", "left turn", "red light", "yellow light"
};
void DrawApilot::drawCarrotModel(const UIState* s) {
    SubMaster& sm = *(s->sm);
    auto detections = sm["carrotModel"].getCarrotModel().getDetections();

    for (auto detection : detections) {
        auto box = detection.getBox();
        float xMin = box.getXMin();
        float yMin = box.getYMin();
        float xMax = box.getXMax();
        float yMax = box.getYMax();

        // NanoVG로 바운딩 박스 그리기
        nvgBeginPath(s->vg);
        nvgRect(s->vg, xMin, yMin, xMax - xMin, yMax - yMin);
        nvgStrokeColor(s->vg, COLOR_GREEN);
        nvgStroke(s->vg);
        char str[128];
        const char* className = class_names_kr[detection.getClassId()];
        sprintf(str, "%s, %.2f", className, detection.getScore());
        ui_draw_text(s, xMin, yMin - 10, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
    }
}

DrawApilot* drawApilot;
Alert alert;
NVGcolor alert_color;
void ui_draw(UIState *s, int w, int h) {
  // Update intrinsics matrix after possible wide camera toggle change
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_DST_COLOR, GL_ZERO);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);
  drawApilot->drawLaneLines(s);
  drawApilot->drawLeadApilot(s);
  drawApilot->drawDebugText(s, s->show_debug>1);
  drawApilot->drawDeviceState(s, s->show_device_stat);
  if(s->fb_w > 1200)
    drawApilot->drawCarrotModel(s);

  ui_draw_text_a2(s);

  //ui_draw_vision(s);
  //dashcam(s);
    //param value
  //nvgFontSize(s->vg, 170);
  //nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  //ui_print(s, s->fb_w/2, s->fb_h/2, "%s", "APILOT");

  ui_draw_alert(s);

  nvgResetScissor(s->vg);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}
void ui_draw_alert(UIState* s) {
    if (alert.size != cereal::ControlsState::AlertSize::NONE) {
        alert_color = COLOR_ORANGE;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 300, alert.text1.toStdString().c_str(), 100, alert_color, BOLD, 3.0f, 8.0f);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 200, alert.text2.toStdString().c_str(), 70, alert_color, BOLD, 3.0f, 8.0f);
    }
}
void ui_update_alert(const Alert& a) {
    //alert_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
    //printf("r=%d, g=%d, b=%d\n", color.red(), color.green(), color.blue());
    alert = a;
}

void ui_nvg_init(UIState *s) {
  // on EON, we enable MSAA
  //s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
      {"KaiGenGothicKR-Normal", "../assets/addon/font/KaiGenGothicKR-Normal.ttf"},
      {"KaiGenGothicKR-Medium", "../assets/addon/font/KaiGenGothicKR-Medium.ttf"},
      {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
      {"Inter-Bold", "../assets/fonts/Inter-Bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }
  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    //{"wheel", "../assets/img_chffr_wheel.png"},
    //{"driver_face", "../assets/img_driver_face.png"},
    //{"speed_bump", "../assets/addon/img/img_speed_bump.png"},
  {"ic_radar", "../assets/images/radar_red.png"},
  {"ic_radar_vision", "../assets/images/radar_vision.png"},
  {"ic_radar_no", "../assets/images/no_radar.png"},
  {"ic_steer_momo", "../assets/images/steer_momo.png"},
  {"ic_steer_red", "../assets/images/steer_red.png"},
  {"ic_steer_green", "../assets/images/steer_green.png"},
  {"ic_steer_yellow", "../assets/images/steer_yellow.png"},
  {"ic_lane_change_l", "../assets/images/lane_change_l.png"},
  {"ic_lane_change_r", "../assets/images/lane_change_r.png"},
  {"ic_lane_change_inhibit", "../assets/images/lane_change_inhibit.png"},
  {"ic_lane_change_steer", "../assets/images/lane_change_steer.png"},
  {"ic_bsd_l", "../assets/images/bsd_l.png"},
  {"ic_bsd_r", "../assets/images/bsd_r.png"},
  {"ic_turn_l", "../assets/images/turn_l.png"},
  {"ic_turn_r", "../assets/images/turn_r.png"},
  {"ic_turn_u", "../assets/images/turn_u.png"},
  {"ic_blinker_l", "../assets/images/blink_l.png"},
  {"ic_blinker_r", "../assets/images/blink_r.png"},
  {"ic_speed_bg", "../assets/images/speed_bg.png"},
  {"ic_traffic_green", "../assets/images/traffic_green.png"},
  {"ic_traffic_red", "../assets/images/traffic_red.png"},
  {"ic_tire", "../assets/images/img_tire.png"},
  {"ic_road_speed", "../assets/images/road_speed.png"},
  {"ic_speed_bump", "../assets/images/speed_bump.png"},
  {"ic_nda", "../assets/images/img_nda.png"},
  {"ic_navi","../assets/images/img_navi.png"},
  {"ic_scc2", "../assets/images/img_scc2.png"},
  {"ic_radartracks", "../assets/images/img_radartracks.png"},
  {"ic_apm", "../assets/images/img_apm.png"},
  {"ic_apn", "../assets/images/img_apn.png"},
  {"ic_hda", "../assets/images/img_hda.png"},
  {"ic_osm", "../assets/images/img_osm.png"},
  {"ic_navi_point", "../assets/images/navi_point.png"}

  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
  drawApilot = new DrawApilot();
}
void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;
#if 0
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(width / 2, height / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
#endif
}
