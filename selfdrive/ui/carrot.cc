#include "selfdrive/ui/carrot.h"

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


constexpr float MIN_DRAW_DISTANCE = 10.0;
constexpr float MAX_DRAW_DISTANCE = 100.0;

ModelRenderer* _model = NULL;
extern  int get_path_length_idx(const cereal::XYZTData::Reader& line, const float path_height);
int g_fps= 0;

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
static void ui_draw_text_vg(NVGcontext* vg, float x, float y, const char* string, float size, NVGcolor color, const char* font_name, float borderWidth = 3.0, float shadowOffset = 0.0, NVGcolor borderColor = COLOR_BLACK, NVGcolor shadowColor = COLOR_BLACK) {
    //y += 6;
    nvgFontFace(vg, font_name);
    nvgFontSize(vg, size);
    if (borderWidth > 0.0) {
        //NVGcolor borderColor = COLOR_BLACK;
        nvgFillColor(vg, borderColor);
        for (int i = 0; i < 360; i += 45) {
            float angle = i * NVG_PI / 180.0f;
            float offsetX = borderWidth * cos(angle);
            float offsetY = borderWidth * sin(angle);
            nvgText(vg, x + offsetX, y + offsetY, string, NULL);
        }
    }
    if (shadowOffset != 0.0) {
        //NVGcolor shadowColor = COLOR_BLACK;
        nvgFillColor(vg, shadowColor);
        nvgText(vg, x + shadowOffset, y + shadowOffset, string, NULL);
    }
    nvgFillColor(vg, color);
    nvgText(vg, x, y, string, NULL);
}
void ui_draw_line_vg(NVGcontext* vg, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {
    if (vd.size() == 0) return;

    nvgBeginPath(vg);
    nvgMoveTo(vg, vd.at(0).x(), vd.at(0).y());
    for (int i = 1; i < vd.size(); i++) {
        nvgLineTo(vg, vd.at(i).x(), vd.at(i).y());
    }
    nvgClosePath(vg);
    if (color) {
        nvgFillColor(vg, *color);
    }
    else if (paint) {
        nvgFillPaint(vg, *paint);
    }
    nvgFill(vg);
    if (stroke > 0.0) {
        nvgStrokeColor(vg, strokeColor);
        nvgStrokeWidth(vg, stroke);
        nvgStroke(vg);
    }
}
void ui_draw_line2_vg(NVGcontext* vg, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {

    nvgBeginPath(vg);
    nvgMoveTo(vg, x[0], y[0]);
    for (int i = 1; i < size; i++) {
        nvgLineTo(vg, x[i], y[i]);
    }
    nvgClosePath(vg);
    if (color) {
        nvgFillColor(vg, *color);
    }
    else if (paint) {
        nvgFillPaint(vg, *paint);
    }
    nvgFill(vg);

    if (stroke > 0.0) {
        nvgStrokeColor(vg, strokeColor);
        nvgStrokeWidth(vg, stroke);
        nvgStroke(vg);
    }
}

void ui_draw_image(const UIState* s, const Rect1& r, const char* name, float alpha) {
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
    nvgRect(s->vg, r.x, r.y, r.w, r.h);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
}
void ui_draw_line(const UIState* s, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {
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
void ui_draw_line2(const UIState* s, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke = 0.0, NVGcolor strokeColor = COLOR_WHITE) {

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
#if 0
void ui_draw_rect(NVGcontext* vg, const Rect1& r, NVGcolor color, int width, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
}
#endif

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


////////////////////  draw_text animated
float a_x = 0.;
float a_y = 0.;
float a_size = 0.;
const int a_max = 100;
char a_font[128] = "";
int a_time = -1;
char a_string[256] = "";
NVGcolor a_color = COLOR_WHITE;
static void ui_draw_text_a2(const UIState* s) {
    if (a_time <= 0) return;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    a_time -= 10;
    int a_time1 = a_time;
    if (a_time1 > 100) a_time1 = 100;
    int x = (s->fb_w / 2 * a_time1 + a_x * (a_max - a_time1)) / a_max;
    int y = ((s->fb_h - 400) * a_time1 + a_y * (a_max - a_time1)) / a_max;
    int size = (350 * a_time1 + a_size * (a_max - a_time1)) / a_max;
    if (a_time >= 100) ui_draw_text(s, x, y, a_string, size, a_color, a_font, 9.0, 8.0, COLOR_BLACK, COLOR_BLACK);
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
////////////////////  draw_text animated


//##################### drawPlot
#define PLOT_MAX 400 //500
class DrawPlot : public QObject {
    Q_OBJECT
protected:
    int     plotSize = 0;
    int     plotIndex = 0;
    float   plotQueue[3][PLOT_MAX];
    float   plotMin = 0.;
    float   plotMax = 0.;
    float   plotShift = 0.0;
    float   plotX = 40.0;// 300.0;
    float   plotWidth = 1000;
    float   plotY = 120.0;// 30.0;
    float   plotHeight = 300.0;
    float   plotRatio = 1.0;
    float   plotDx = 2.0;
    int     show_plot_mode = 0;
    int     show_plot_mode_prev = -1;
    void	drawPlotting(const UIState* s, int index, int start, float x, float y[], int size, NVGcolor* color, float stroke = 0.0) {
        nvgBeginPath(s->vg);
        plotRatio = (plotMax - plotMin) < 1.0 ? plotHeight : plotHeight / (plotMax - plotMin);
        float dx = plotDx;
        char str[128];

        for (int i = 0; i < size; i++) {
            float data = y[(start - i + PLOT_MAX) % PLOT_MAX];
            float plot_y = plotY + plotHeight - (data - plotMin) * plotRatio;
            if (i == 0) {
                nvgMoveTo(s->vg, x + (size - i) * dx, plot_y);
                sprintf(str, "%.2f", data);
                ui_draw_text(s, x + (size - i) * dx + 50, plot_y + ((index > 0) ? 40 : 0), str, 40, *color, BOLD);
            }
            else nvgLineTo(s->vg, x + (size - i) * dx, plot_y);
        }
        //nvgClosePath(s->vg);

        if (stroke > 0.0) {
            nvgStrokeColor(s->vg, *color);
            nvgStrokeWidth(s->vg, stroke);
            nvgStroke(s->vg);
        }

    }
    void	makePlotData(const UIState* s, float data[], char* title) {

        SubMaster& sm = *(s->sm);
        auto    car_state = sm["carState"].getCarState();
        auto    lp = sm["longitudinalPlan"].getLongitudinalPlan();
        auto    car_control = sm["carControl"].getCarControl();

        float   a_ego = car_state.getAEgo();
        float   v_ego = car_state.getVEgo();

        float   accel = lp.getAccels()[0];
        float   speeds_0 = lp.getSpeeds()[0];

        float   accel_out = car_control.getActuators().getAccel();

        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto position = model.getPosition();
        const auto velocity = model.getVelocity();

        auto lead_radar = sm["radarState"].getRadarState().getLeadOne();

        // 0: yellow, 1: green, 2: orange
        switch (show_plot_mode) {
        case 0:
        case 1:
            data[0] = a_ego;
            data[1] = accel;
            data[2] = accel_out;
            sprintf(title, "1.Accel (Y:a_ego, G:a_target, O:a_out)");
            break;
        case 2:
            data[0] = speeds_0;
            data[1] = v_ego;
            data[2] = a_ego;
            sprintf(title, "2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)");
            break;
        case 3:
            data[0] = position.getX()[32];
            data[1] = velocity.getX()[32];
            data[2] = velocity.getX()[0];
            sprintf(title, "3.Model(Y:pos_32, G:vel_32, O:vel_0)");
			break;
        case 4:
            data[0] = a_ego;
            data[1] = lead_radar.getALeadK();
            data[2] = lead_radar.getVRel();
            sprintf(title, "4.Lead(Y:a_ego, G:a_lead, O:v_rel)");
            break;
        case 5:
            data[0] = a_ego;
            data[1] = lead_radar.getALead();
            data[2] = lead_radar.getALeadK();
            sprintf(title, "4.Lead(Y:a_ego, G:a_lead, O:a_lead_k)");
            break;
        default:
            data[0] = data[1] = data[2] = 0;
            sprintf(title, "no data");
            break;
        }
        if (show_plot_mode != show_plot_mode_prev) {
            plotSize = 0;
            plotIndex = 0;
            plotMin = 0.;
            plotMax = 0.;
            show_plot_mode_prev = show_plot_mode;
        }
    }

    Params  params;

public:
    void	draw(const UIState* s) {
        show_plot_mode = params.getInt("ShowPlotMode");
        if (show_plot_mode == 0) return;
        SubMaster& sm = *(s->sm);
        if (sm.alive("carState") && sm.alive("longitudinalPlan"));
        else return;

        ui_fill_rect(s->vg, { (int)plotX - 10, (int)plotY - 50, (int)(PLOT_MAX * plotDx) + 150, (int)plotHeight + 100}, COLOR_BLACK_ALPHA(90), 30);


        float plot_data[3] =  {0., 0., 0. };
        char title[128] = "";

        makePlotData(s, plot_data, title);

        for (int i = 0; i < 3; i++) {
            if (plotMin > plot_data[i]) plotMin = plot_data[i];
            if (plotMax < plot_data[i]) plotMax = plot_data[i];
        }
        if (plotMin > -2.) plotMin = -2.0;
        if (plotMax < 2.0) plotMax = 2.0;

        plotIndex = (plotIndex + 1) % PLOT_MAX;
        for(int i=0;i<3;i++) plotQueue[i][plotIndex] = plot_data[i];
        if (plotSize < PLOT_MAX - 1) plotSize++;

        if (s->fb_w < 1200) return;

        NVGcolor color[3] = { COLOR_YELLOW, COLOR_GREEN, COLOR_ORANGE };
        for (int i = 0; i < 3; i++) {
            drawPlotting(s, i, plotIndex, plotX, plotQueue[i], plotSize, &color[i], 3.0f);
        }
        ui_draw_text(s, 400, plotY - 20, title, 25, COLOR_WHITE, BOLD);
    }
};

class ModelDrawer {
protected:
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
    void update_line_data(const UIState* s, const cereal::XYZTData::Reader& line,
        float y_off, float z_off_left, float z_off_right, QPolygonF* pvd, int max_idx, bool allow_invert = true, float y_shift = 0.0, int start_idx = 0) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(max_idx + 1);
        right_points.reserve(max_idx + 1);

        for (int i = start_idx; i <= max_idx; i++) {
            // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
            if (line_x[i] < 0) continue;
            QPointF left, right;
            bool l = _model->mapToScreen(line_x[i], line_y[i] - y_off + y_shift, line_z[i] + z_off_left, &left);
            bool r = _model->mapToScreen(line_x[i], line_y[i] + y_off + y_shift, line_z[i] + z_off_right, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
        }
        *pvd = left_points + right_points;
    }
};
class PathEndDrawer : ModelDrawer {
private:
    QPointF path_end_left_vertex;
    QPointF path_end_right_vertex;
    int     radarTrackId = -1;
    float   alpha = 0.85;
    float   path_fx = 0.0;
    float   path_fy = 0.0;
    float   path_fwidth = 0.0;

    int     path_x = 0;
    int     path_y = 0;
    int     path_width = 0;
    bool    lead_status = false;
    float   radarDist = 0.0;
    float   visionDist = 0.0;
    int     xState = 0;
    int     trafficState = 0;

    float   v_ego = 0.0;
    bool    brakeHoldActive = false;
    int    softHoldActive = 0;
    bool    longActive = false;

protected:
    bool make_data(const UIState* s) {
		SubMaster& sm = *(s->sm);
		if (!sm.alive("modelV2")) return false;

        v_ego = sm["carState"].getCarState().getVEgo();
        brakeHoldActive = sm["carState"].getCarState().getBrakeHoldActive();
        softHoldActive = sm["carState"].getCarState().getSoftHoldActive();
        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        longActive = selfdrive_state.getEnabled();
        //longActive = sm["carControl"].getCarControl().getLongActive();
        //longActive = sm["carControl"].getCarControl().getEnabled();
        auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        xState = lp.getXState();
        trafficState = lp.getTrafficState();
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        auto line = model.getPosition();
        float max_distance = s->max_distance;
        int idx = get_path_length_idx(line, max_distance);
        float y = line.getY()[idx];
        float z = line.getZ()[idx];

        auto leadsV3 = model.getLeadsV3()[0];
        visionDist = leadsV3.getProb() > .5 ? leadsV3.getX()[0] - 1.52 : 0.0;

        auto lead_one = sm["radarState"].getRadarState().getLeadOne();
        lead_status = lead_one.getStatus();
        if (lead_status) {
            z = line.getZ()[get_path_length_idx(line, lead_one.getDRel())];
            max_distance = lead_one.getDRel();
            y = -lead_one.getYRel();
            radarTrackId = lead_one.getRadarTrackId();
            radarDist = (lead_one.getRadar()) ? lead_one.getDRel() : 0;
        }
        else {
            radarTrackId = -1;
            radarDist = 0;
        }
        _model->mapToScreen(max_distance, y - 1.2, z + 1.22, &path_end_left_vertex);
        _model->mapToScreen(max_distance, y + 1.2, z + 1.22, &path_end_right_vertex);

        float lex = path_end_left_vertex.x();
        float ley = path_end_left_vertex.y();
        float rex = path_end_right_vertex.x();
        float rey = path_end_right_vertex.y();

        float _path_width = rex - lex;
        float _path_x = (lex + rex) / 2;
        float _path_y = (ley + rey) / 2;
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
        path_x = (int)path_fx;
        path_y = (int)path_fy;
        path_width = (int)path_fwidth;

        return true;
	};
    bool isLeadSCC() {
        return radarTrackId == 0;
    }
    bool isRadarDetected() {
        return radarTrackId >= 0;
    }
    bool isLeadDetected() {
        return lead_status;
    }
public:
    int getPathX() { return path_x; }
    int getPathY() { return path_y; }

    void draw(const UIState* s) {
		if (!make_data(s)) return;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        int x = path_x;
        int y = path_y - 135;
        char str[128];
        int disp_y = y + 195;// 175;
        bool draw_dist = false;
        float disp_size = 50;
        if (softHoldActive || brakeHoldActive) {
            sprintf(str, "%s", (brakeHoldActive) ? "AUTOHOLD" : "SOFTHOLD");
            ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
        }
        else if (longActive) {
            if (xState == 3 || xState == 5) {      //XState.e2eStop, XState.e2eStopped
                if (v_ego < 1.0) {
                    sprintf(str, "%s", (trafficState >= 1000) ? "신호오류" : "신호대기");
                    ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
                }
                else {
                    ui_draw_text(s, x, disp_y, "신호감속중", disp_size, COLOR_WHITE, BOLD);
                }
#if 0
                else if (getStopDist() > 0.5) {
                    float dist = getStopDist() * (s->scene.is_metric ? 1 : METER_TO_FOOT);
                    if (dist < 10.0) sprintf(str, "%.1fM", dist);
                    else sprintf(str, "%.0fM", dist);
                    ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
                }
#endif
            }
            else if (xState == 4) {     //XState.e2ePrepare
				ui_draw_text(s, x, disp_y, "신호출발중", disp_size, COLOR_WHITE, BOLD);
			}
            else if (xState == 0 || xState == 1 || xState == 2) {     //XState.lead
                draw_dist = true;
            }
        }
        else draw_dist = true;
        if (draw_dist) {
            //float dist = (getRadarDist() > 0.0) ? getRadarDist() : getVisionDist();
            //if (dist < 10.0) sprintf(str, "%.1f", dist);
            //else sprintf(str, "%.0f", dist);
            //ui_draw_text(s, x, disp_y, str, disp_size, COLOR_WHITE, BOLD);
            int wStr = 0, w = 80;
            float dist = radarDist * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            NVGcolor text_color = (xState==0) ? COLOR_WHITE : (xState==1) ? COLOR_GREY : COLOR_GREEN;
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x - w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, isLeadSCC() ? COLOR_RED : COLOR_ORANGE, 15);
                ui_draw_text(s, x - w, disp_y, str, 40, text_color, BOLD);
            }
            dist = visionDist * (s->scene.is_metric ? 1 : METER_TO_FOOT);
            if (dist > 0.0) {
                sprintf(str, "%.1f", dist);
                wStr = 32 * (strlen(str) + 0);
                ui_fill_rect(s->vg, { (int)(x + w - wStr / 2), (int)(disp_y - 35), wStr, 42 }, COLOR_BLUE, 15);
                ui_draw_text(s, x + w, disp_y, str, 40, text_color, BOLD);
            }
        }


        float px[7], py[7];
        NVGcolor rcolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
        NVGcolor  pcolor = !isRadarDetected() ? ((trafficState == 1) ? rcolor : COLOR_GREEN) : isRadarDetected() ? rcolor : COLOR_BLUE;
        bool show_path_end = true;
        if (show_path_end && !isLeadDetected()) {
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

#if 0
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
            ay[2] = ay[1] - a;
            ay[3] = ay[2];
            NVGcolor acolor = isLeadSCC() ? COLOR_RED : COLOR_ORANGE;
            ui_draw_line2(s, ax, ay, 4, &acolor, nullptr, 0.0f);
#endif
        }

	}
};

class LaneLineDrawer : ModelDrawer {
private:
    float lane_line_probs[4];
    float road_edge_stds[2];
    QPolygonF lane_line_vertices[4];
    QPolygonF road_edge_vertices[2];

protected:
    bool make_data(const UIState* s) {
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        const auto model_lane_lines = model.getLaneLines();
        const auto model_lane_line_probs = model.getLaneLineProbs();
        int max_idx = get_path_length_idx(model_lane_lines[0], s->max_distance);
        for (int i = 0; i < std::size(lane_line_vertices); i++) {
            lane_line_probs[i] = model_lane_line_probs[i];
            update_line_data(s, model_lane_lines[i], 0.025 * lane_line_probs[i], 0.0, 0.0, &lane_line_vertices[i], max_idx);
        }

        // roadedges
        const auto model_road_edges = model.getRoadEdges();
        const auto model_road_edge_stds = model.getRoadEdgeStds();
        int max_idx_road_edge = get_path_length_idx(model_lane_lines[0], 100);
        for (int i = 0; i < std::size(road_edge_vertices); i++) {
            road_edge_stds[i] = model_road_edge_stds[i];
            update_line_data(s, model_road_edges[i], 0.025, 0.0, 0.0, &road_edge_vertices[i], max_idx_road_edge);
        }

        return true;
    }
    void drawRoadEdge(const UIState* s) {
        NVGcolor color;
        for (int i = 0; i < std::size(road_edge_vertices); ++i) {
            float temp_f = std::clamp<float>(road_edge_stds[i] / 2.0, 0.0, 1.0);
            color = nvgRGBAf(1.0 - temp_f, 0.0, temp_f, 1.0);
            ui_draw_line(s, road_edge_vertices[i], &color, nullptr);
        }
    }

public:
    void draw(const UIState* s, int show_lane_info) {
        if (show_lane_info < 1) return;
        if(!make_data(s)) return;
        NVGcolor color;
        for (int i = 0; i < std::size(lane_line_vertices); ++i) {
            color = nvgRGBAf(1.0, 1.0, 1.0, (lane_line_probs[i] > 0.3) ? 1.0 : 0.0);
            ui_draw_line(s, lane_line_vertices[i], &color, nullptr);
        }
        if(show_lane_info > 1) drawRoadEdge(s);
    }
};
class TurnInfoDrawer : ModelDrawer {
private:
    int icon_size = 256;
    int xSpdLimit = 0;
    int xSpdDist = 0;
    int xSignType = -1;
    int xTurnInfo = -1;
    int xDistToTurn = 0;
    int nRoadLimitSpeed = 20;
    int active_carrot = 0;

    int nGoPosDist = 0;
    int nGoPosTime = 0;

    QString szSdiDescr = "";
    QString atc_type;

protected:
    QPointF navi_turn_point[2];
    float navi_turn_point_x[2] = { 0.0, };
    float navi_turn_point_y[2] = { 0.0, };
    bool navi_turn_point_flag = true;
    void drawTurnInfo(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        if (xDistToTurn < 1500 && xDistToTurn > 0) {
            SubMaster& sm = *(s->sm);

            const auto carrot_man = sm["carrotMan"].getCarrotMan();
            QString szTBTMainText = QString::fromStdString(carrot_man.getSzTBTMainText());

            const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
            const auto road_edges = model.getRoadEdges();
            int idx = get_path_length_idx(road_edges[0], xDistToTurn);
            int m_idx[2] = { 0, 1 };
            for (int i = 0; i < 2; i++) {
                int m = m_idx[i];
                _model->mapToScreen(road_edges[m].getX()[idx], road_edges[m].getY()[idx], road_edges[m].getZ()[idx], &navi_turn_point[i]);
            }

            float scale = 0.2;
            if (xDistToTurn < 200) scale = 1.0 - (0.8 * xDistToTurn / 200.);
            scale *= 0.5;
            int size_x = 348 * scale;
            int size_y = 540 * scale;
            int img_x = 0;
            int img_y = 0;
            float alpha = (navi_turn_point_flag) ? 1.0 : 0.1;

            for (int i = 0; i < 2; i++) {
                navi_turn_point_x[i] = navi_turn_point_x[i] * (1.0 - alpha) + navi_turn_point[i].x() * alpha;
                navi_turn_point_y[i] = navi_turn_point_y[i] * (1.0 - alpha) + navi_turn_point[i].y() * alpha;
            }
            navi_turn_point_flag = false;

            switch (xTurnInfo) {
            case 1: case 3: case 5:
                img_x = (int)navi_turn_point_x[0] - size_x / 2;
                img_y = (int)navi_turn_point_y[0] - size_y;
                ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
                nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                break;
            case 2: case 4:
                img_x = (int)navi_turn_point_x[1] - size_x / 2;
                img_y = (int)navi_turn_point_y[1] - size_y;
                ui_draw_image(s, { img_x, img_y, size_x, size_y }, "ic_navi_point", 1.0f);
                nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
                break;
            }
            char str[128] = "";
            sprintf(str, "%s", szTBTMainText.toStdString().c_str());
            float text_scale = (scale>0.5)?scale:0.5;
            ui_draw_text(s, img_x + size_x / 2, img_y + size_y + 100 * text_scale, str, 100 * text_scale, COLOR_WHITE, BOLD);
        }
        else navi_turn_point_flag = true;
    }
    QPointF left_dist_point;
    float left_dist_x = 0.0;
    float left_dist_y = 0.0;
    bool left_dist_flag = true;

    void drawSpeedLimit(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        if (xSpdLimit <= 0) left_dist_flag = true;
        if (xSpdDist > 0) {
            SubMaster& sm = *(s->sm);
            const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
            const auto road_edges = model.getRoadEdges();
            const auto lane_lines = model.getLaneLines();
            char str[128] = "";

            if (xSpdDist < 100) {
                int idx = get_path_length_idx(lane_lines[2], xSpdDist);
                _model->mapToScreen(lane_lines[2].getX()[idx], lane_lines[2].getY()[idx], lane_lines[2].getZ()[idx], &left_dist_point);
            }
            else {
                int idx = get_path_length_idx(road_edges[0], xSpdDist);
                _model->mapToScreen(road_edges[1].getX()[idx], road_edges[1].getY()[idx], road_edges[1].getZ()[idx], &left_dist_point);
            }

            float scale = 0.2;
            if (xSpdDist < 200) scale = 1.0 - (0.8 * xSpdDist / 200.);
            int bx = left_dist_point.x() + 140 * scale;
            int by = left_dist_point.y();
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

            if (xSpdDist > 0) {

                if (s->scene.is_metric) {
                    if (xSpdDist < 1000) sprintf(str, "%d m", xSpdDist);
                    else  sprintf(str, "%.1f km", xSpdDist / 1000.f);
                }
                else {
                    // 1 미터 = 3.28084 피트로 계산
                    if (xSpdDist < 1609) sprintf(str, "%d ft", (int)(xSpdDist * 3.28084)); // 1609m(1마일)보다 작으면 피트로 표시
                    else sprintf(str, "%.1f mi", xSpdDist / 1609.34f); // 1609m 이상이면 마일로 표시
                }
                ui_draw_text(s, bx, by + 120 * scale, str, 40 * scale, COLOR_WHITE, BOLD);
            }

            if (xSignType == 22) {
                ui_draw_image(s, { bx - (int)(60 * scale), by - (int)(50 * scale), (int)(120 * scale), (int)(150 * scale) }, "ic_speed_bump", 1.0f);
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
                sprintf(str, "%d", xSpdLimit);
                ui_draw_text(s, bx, by + 25 * scale - 6 * (1 - scale), str, 60 * scale, COLOR_BLACK, BOLD, 0.0f, 0.0f);
            }
        }
	}
    void drawTurnInfoHud(const UIState* s) {
#if 0
        active_carrot = 2;
        nGoPosDist = 500000;
        nGoPosTime = 4 * 60 * 60;
        szSdiDescr = "어린이 보호구역(스쿨존 시작 구간)";
        xTurnInfo = 1;
        xDistToTurn = 1000;
#endif

        if (active_carrot <= 0) return;
        if (xDistToTurn <= 0 || nGoPosDist <= 0) return;
        char str[128] = "";

        int tbt_x = s->fb_w - 800;
        int tbt_y = s->fb_h - 300;
        ui_fill_rect(s->vg, { tbt_x, tbt_y, 790, 240 }, COLOR_BLACK_ALPHA(120), 30);

        if(xTurnInfo > 0) {
            int bx = tbt_x + 100;
            int by = tbt_y + 85;
            if (atc_type.length() > 0 && !atc_type.contains("prepare")) {
                ui_fill_rect(s->vg, { bx - 80, by - 65, 160, 210 }, COLOR_GREEN_ALPHA(100), 15);
            }
            switch (xTurnInfo) {
            case 1: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f); break;
            case 2: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f); break;
            case 3: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f); break;
            case 4: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f); break;
            case 7: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_u", 1.0f); break;
            case 6: ui_draw_text(s, bx, by + 20, "TG", 35, COLOR_WHITE, BOLD); break;
            case 8: ui_draw_text(s, bx, by + 20, "목적지", 35, COLOR_WHITE, BOLD); break;
            default:
                sprintf(str, "감속:%d", xTurnInfo);
                ui_draw_text(s, bx, by + 20, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
                break;
            }
            if (xDistToTurn < 1000) sprintf(str, "%d m", xDistToTurn);
            else  sprintf(str, "%.1f km", xDistToTurn / 1000.f);
            ui_draw_text(s, bx, by + 120, str, 40, COLOR_WHITE, BOLD);
        }
        nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        if (szSdiDescr.length() > 0) {
            float bounds[4];  // [xmin, ymin, xmax, ymax]를 저장하는 배열
            nvgFontSize(s->vg, 40);
            nvgTextBounds(s->vg, tbt_x + 200, tbt_y + 200, szSdiDescr.toStdString().c_str(), NULL, bounds);
            float text_width = bounds[2] - bounds[0];
            float text_height = bounds[3] - bounds[1];
            ui_fill_rect(s->vg, { (int)bounds[0] - 10, (int)bounds[1] - 2, (int)text_width + 20, (int)text_height + 13 }, COLOR_GREEN, 10);
            ui_draw_text(s, tbt_x + 200, tbt_y + 200, szSdiDescr.toStdString().c_str(), 40, COLOR_WHITE, BOLD);
        }
        if (nGoPosDist > 0 && nGoPosTime > 0) {
            time_t now = time(NULL);  // 현재 시간 얻기
            struct tm* local = localtime(&now);
            int remaining_minutes = (int)nGoPosTime / 60;
            local->tm_min += remaining_minutes;
            mktime(local);
            sprintf(str, "도착: %.1f분(%02d:%02d)", (float)nGoPosTime / 60., local->tm_hour, local->tm_min);
            ui_draw_text(s, tbt_x + 190, tbt_y + 80, str, 50, COLOR_WHITE, BOLD);
            sprintf(str, "%.1fkm", nGoPosDist / 1000.);
            ui_draw_text(s, tbt_x + 190 + 120, tbt_y + 130, str, 50, COLOR_WHITE, BOLD);
        }
    }
public:
    void draw(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2") || !sm.alive("carrotMan") || !sm.alive("carState")) {
            active_carrot = -1;
            return;
        }
        const auto carrot_man = sm["carrotMan"].getCarrotMan();
        //const auto car_state = sm["carState"].getCarState();
        xSpdLimit = carrot_man.getXSpdLimit();
        xSpdDist = carrot_man.getXSpdDist();
        xSignType = carrot_man.getXSpdType();
        xTurnInfo = carrot_man.getXTurnInfo();
        xDistToTurn = carrot_man.getXDistToTurn();
        nRoadLimitSpeed = carrot_man.getNRoadLimitSpeed();
        active_carrot = carrot_man.getActive();
        atc_type = QString::fromStdString(carrot_man.getAtcType());

        nGoPosDist = carrot_man.getNGoPosDist();
        nGoPosTime = carrot_man.getNGoPosTime();
        szSdiDescr = QString::fromStdString(carrot_man.getSzSdiDescr());

        //active_carrot = 2;
        //xSpdLimit = 110;
        //xSpdDist = 12345;
        //nRoadLimitSpeed = 110;

        int bx = s->fb_w - 120;// 350;// 150;
        int by = 300;// s->fb_h - 150; // 410;
        char str[128] = "";

        if (xSpdLimit > 0) {
            if (xSignType == 22) {
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
                sprintf(str, "%d", xSpdLimit);
                ui_draw_text(s, bx, by + 25, str, 60, COLOR_BLACK, BOLD, 0.0f, 0.0f);
            }
            if (xSpdDist < 1000) sprintf(str, "%d m", xSpdDist);
            else  sprintf(str, "%.1f km", xSpdDist / 1000.f);
            ui_draw_text(s, bx, by + 120, str, 40, COLOR_WHITE, BOLD);
        }
        else if(false && xTurnInfo > 0) {
            switch (xTurnInfo) {
            case 1: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f); break;
            case 2: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f); break;
            case 3: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f); break;
            case 4: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f); break;
            case 7: ui_draw_image(s, { bx - icon_size / 2, by - icon_size / 2, icon_size, icon_size }, "ic_turn_u", 1.0f); break;
            case 6: ui_draw_text(s, bx, by + 20, "TG", 35, COLOR_WHITE, BOLD); break;
            case 8: ui_draw_text(s, bx, by + 20, "arrived", 35, COLOR_WHITE, BOLD); break;
            default:
                sprintf(str, "unknown(%d)", xTurnInfo);
                ui_draw_text(s, bx, by + 20, str, 35, COLOR_WHITE, BOLD, 0.0f, 0.0f);
                break;
            }
            if (xDistToTurn < 1000) sprintf(str, "%d m", xDistToTurn);
            else  sprintf(str, "%.1f km", xDistToTurn / 1000.f);
            ui_draw_text(s, bx, by + 120, str, 40, COLOR_WHITE, BOLD);
        }
        else if (active_carrot > 1 && nRoadLimitSpeed >= 30 && nRoadLimitSpeed < 200) {
            ui_draw_image(s, { bx - 60, by - 50, 120, 150 }, "ic_road_speed", 1.0f);
            sprintf(str, "%d", nRoadLimitSpeed);
            ui_draw_text(s, bx, by + 75, str, 50, COLOR_BLACK, BOLD, 0.0f, 0.0f);
        }
        drawTurnInfo(s);
        drawSpeedLimit(s);
        drawTurnInfoHud(s);

    }
};
bool _right_blinker = false;
bool _left_blinker = false;
class DesireDrawer : ModelDrawer {
protected:
    int icon_size = 256;
    int blinker_timer = 0;
public:
    void draw(const UIState* s, int x, int y) {
        blinker_timer = (blinker_timer + 1) % 16;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2") || !sm.alive("carrotMan") || !sm.alive("carState")) return;
        auto meta = sm["modelV2"].getModelV2().getMeta();
        auto laneChangeDirection = meta.getLaneChangeDirection();
        auto laneChangeState = meta.getLaneChangeState();
        float desireStateTurnLeft = meta.getDesireState()[1];
        float desireStateTurnRight = meta.getDesireState()[2];
        float desireStateLaneChangeLeft = meta.getDesireState()[3];
        float desireStateLaneChangeRight = meta.getDesireState()[4];
        int desireEvent = 0;    // TODO

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

        const auto carrot_man = sm["carrotMan"].getCarrotMan();
        const auto car_state = sm["carState"].getCarState();
        QString atc_type = QString::fromStdString(carrot_man.getAtcType());

        bool left_blinker = car_state.getLeftBlinker() || atc_type=="fork left" || atc_type =="turn left";
        bool right_blinker = car_state.getRightBlinker() || atc_type=="fork right" || atc_type =="turn right";

        _right_blinker = false;
        _left_blinker = false;
        if (blinker_timer <= 8) {
            if (right_blinker) {
		_right_blinker = true;
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_r", 1.0f);
            }
            if (left_blinker) {
		_left_blinker = true;
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_l", 1.0f);
            }
        }
    }
};

class BlindSpotDrawer : ModelDrawer{
protected:
    QPolygonF lane_barrier_vertices[2];

protected:
    void ui_draw_bsd(const UIState* s, const QPolygonF& vd, NVGcolor* color, bool right) {
        int index = vd.length();

        float x[4], y[4];
        for (int i = 0; i < index / 2 - 2; i += 2) {

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
    bool make_data(const UIState* s) {
        SubMaster& sm = *(s->sm);
        if (!sm.alive("modelV2")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        auto model_position = model.getPosition();
        int max_idx_barrier_l = get_path_length_idx(model_position, 40.0);
        int max_idx_barrier_r = get_path_length_idx(model_position, 40.0);
        update_line_data(s, model_position, 0, 1.2 - 0.05, 1.2 - 0.6, &lane_barrier_vertices[0], max_idx_barrier_l, false, -1.7); // 차선폭을 알면 좋겠지만...
        update_line_data(s, model_position, 0, 1.2 - 0.05, 1.2 - 0.6, &lane_barrier_vertices[1], max_idx_barrier_r, false, 1.7);
        return true;
    }
public:
    void draw(const UIState* s) {
        if (!make_data(s)) return;

        NVGcolor color = nvgRGBA(255, 215, 0, 150);
        NVGcolor color2 = nvgRGBA(0, 204, 0, 150);

        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        bool left_blindspot = car_state.getLeftBlindspot();
        bool right_blindspot = car_state.getRightBlindspot();

        auto lead_left = sm["radarState"].getRadarState().getLeadLeft();
        auto lead_right = sm["radarState"].getRadarState().getLeadRight();
        auto meta = sm["modelV2"].getModelV2().getMeta();
        auto laneChangeState = meta.getLaneChangeState();
        auto laneChangeDirection = meta.getLaneChangeDirection();
        bool rightLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) &&
            (laneChangeDirection == cereal::LaneChangeDirection::RIGHT);
        bool leftLaneChange = (laneChangeState == cereal::LaneChangeState::PRE_LANE_CHANGE) &&
            (laneChangeDirection == cereal::LaneChangeDirection::LEFT);

#if 0
        left_blindspot = right_blindspot = true;
#endif
        if (left_blindspot) {
            ui_draw_bsd(s, lane_barrier_vertices[0], &color, false);
        }
        else if (lead_left.getStatus() && lead_left.getDRel() < car_state.getVEgo() * 3.0 && leftLaneChange) {
            ui_draw_bsd(s, lane_barrier_vertices[0], &color2, false);
        }

        if (right_blindspot) {
            ui_draw_bsd(s, lane_barrier_vertices[1], &color, true);
        }
        else if (lead_right.getStatus() && lead_right.getDRel() < car_state.getVEgo() * 3.0 && rightLaneChange) {
            ui_draw_bsd(s, lane_barrier_vertices[1], &color2, true);
        }
    }
};

class PathDrawer : ModelDrawer {
private:
    QPolygonF track_vertices;
    int     show_path_mode = 13;
    int     show_path_color = 14;
    int     show_path_mode_normal = 13;
    int     show_path_color_normal = 14;
    int     show_path_mode_lane = 13;
    int     show_path_color_lane = 14;
    int     show_path_mode_cruise_off = 13;
    int     show_path_color_cruise_off = 14;
    float   show_path_width = 1.0;
    Params  params;
    int     params_count = 0;
    bool    active_lane_line = false;

protected:
    void update_line_data2(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, int max_idx, bool allow_invert = true) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(max_idx + 1);
        right_points.reserve(max_idx + 1);

        for (int i = 0; i <= max_idx; i++) {
            // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
            if (line_x[i] < 0) continue;
            float z_off = interp<float>((float)line_x[i], { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
            float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
            y_off *= width_apply;

            QPointF left, right;
            bool l = _model->mapToScreen(line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
            bool r = _model->mapToScreen(line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
        }
        *pvd = left_points + right_points;
    }
    void update_line_data_dist(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {
        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(40 + 1);
        right_points.reserve(40 + 1);
        float idxs[line_x.size()], line_xs[line_x.size()], line_ys[line_x.size()], line_zs[line_x.size()];
        float   x_prev = 0;
        for (int i = 0; i < line_x.size(); i++) {
            idxs[i] = (float)i;
            if (i > 0 && line_x[i] < x_prev) {
                //printf("plan data error.\n");
                line_xs[i] = x_prev;
            }
            else line_xs[i] = line_x[i];
            x_prev = line_xs[i];
            line_ys[i] = line_y[i];
            line_zs[i] = line_z[i];
        }

        float   dist = 2.0;// , dist_dt = 1.;
        bool    exit = false;
        //printf("\ndist = ");
        for (; !exit; dist = dist + dist * 0.15) {
            //dist_dt += (i*0.05);
            if (dist >= max_dist) {
                dist = max_dist;
                exit = true;
            }
            //printf("%.0f ", dist);
            float z_off = interp<float>(dist, { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
            float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
            y_off *= width_apply;
            float  idx = interp<float>(dist, line_xs, idxs, line_x.size(), false);
            if (idx >= line_x.size()) break;
            float line_y1 = interp<float>(idx, idxs, line_ys, line_x.size(), false);
            float line_z1 = interp<float>(idx, idxs, line_zs, line_x.size(), false);

            QPointF left, right;
            bool l = _model->mapToScreen(dist, line_y1 - y_off, line_z1 + z_off, &left);
            bool r = _model->mapToScreen(dist, line_y1 + y_off, line_z1 + z_off, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                    //printf("invert...\n");
                    continue;
                }
                left_points.push_back(left);
                right_points.push_front(right);
            }
            //else printf("range out..\n");
        }
        *pvd = left_points + right_points;
    }
    float dist_function(float t, float max_dist) {
        float dist = 3.0 * pow(1.2, t);
        return (dist >= max_dist) ? max_dist : dist;
    }
    float pos_t = 0.0f;
    void update_line_data_dist3(const UIState* s, const cereal::XYZTData::Reader& line,
        float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {

        const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
        QPolygonF left_points, right_points;
        left_points.reserve(41);
        right_points.reserve(41);

        // Prepare line data (correcting x-axis if needed)
        std::vector<float> idxs(line_x.size()), line_xs(line_x.size()), line_ys(line_x.size()), line_zs(line_x.size());
        float x_prev = 0;
        for (int i = 0; i < line_x.size(); i++) {
            idxs[i] = static_cast<float>(i);
            line_xs[i] = (i > 0 && line_x[i] < x_prev) ? x_prev : line_x[i];
            x_prev = line_xs[i];
            line_ys[i] = line_y[i];
            line_zs[i] = line_z[i];
        }

        // Get simulation state
        SubMaster& sm = *(s->sm);
        //auto controls_state = sm["controlsState"].getControlsState();
        auto car_state = sm["carState"].getCarState();
        float v_ego_kph = car_state.getVEgoCluster() * MS_TO_KPH;


        // Calculate time step and position
        //float pos_t = 0.0f;
        float dt = std::min(v_ego_kph * 0.01f, (show_path_mode >= 10) ? 0.6f : 1.0f);
        if (v_ego_kph < 1) pos_t = 4.0f;
        else if (dt < 0.2f) dt = 0.2f;
        pos_t += dt;
        if (pos_t > 24.0f) pos_t = pos_t - 24.0f;

        // Add time points based on path mode
        std::vector<float> draw_t;
        draw_t.reserve(50);
        draw_t.push_back(pos_t);

        auto add_time_points = [&](int count, float interval) {
            for (int i = 0; i < count; i++) {
                float t = draw_t.back();
                draw_t.push_back((t + interval > 24.0f) ? t + interval - 24.0f : t + interval);
            }
            };

        switch (show_path_mode) {
        case 9:
            add_time_points(1, 3.0f);
            add_time_points(1, 10.0f);
            add_time_points(1, 3.0f);
            break;
        case 10:
            add_time_points(7, 3.0f);
            break;
        case 11:
            add_time_points(5, 3.0f);
            break;
        case 12: {
            int n = std::clamp(static_cast<int>(v_ego_kph * 0.058f - 0.5f), 0, 7);
            add_time_points(n, 3.0f);
            break;
        }
        }

        // Find the smallest time value
        auto min_elem = std::min_element(draw_t.begin(), draw_t.end());
        int draw_t_idx = std::distance(draw_t.begin(), min_elem);

        // Process each time point and calculate points
        bool exit = false;
        for (int i = 0; i <= draw_t.size() && !exit; i++) {
            float t = draw_t[draw_t_idx];
            draw_t_idx = (draw_t_idx + 1) % draw_t.size();
            if (t < 3.0f) continue;

            float dist = dist_function(t, max_dist);
            if (dist == max_dist) exit = true;

            for (int j = 2; j >= 0; j--) {
                dist = exit ? dist_function(100, max_dist) : dist_function(t - j * 1.0f, max_dist);
                float z_off = interp<float>(dist, { 0.0f, 100.0f }, { z_off_start, z_off_end }, false);
                float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false) * width_apply;
                float idx = interp<float>(dist, line_xs.data(), idxs.data(), line_x.size(), false);

                if (idx >= line_x.size()) {
                    printf("index... %.1f\n", idx);
                    break;
                }

                float line_y1 = interp<float>(idx, idxs.data(), line_ys.data(), line_x.size(), false);
                float line_z1 = interp<float>(idx, idxs.data(), line_zs.data(), line_x.size(), false);

                QPointF left, right;
                bool l = _model->mapToScreen(dist, line_y1 - y_off, line_z1 + z_off, &left);
                bool r = _model->mapToScreen(dist, line_y1 + y_off, line_z1 + z_off, &right);
                if (l && r) {
                    left_points.push_back(left);
                    right_points.push_front(right);
                }

                if (exit) break;
            }
        }

        // Combine left and right points into the polygon
        *pvd = left_points + right_points;
    }
protected:
    bool make_data(const UIState* s) {
		SubMaster& sm = *(s->sm);
		if (!sm.alive("modelV2") || !sm.alive("carState")) return false;
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        active_lane_line = sm["controlsState"].getControlsState().getActiveLaneLine();
        auto model_position = model.getPosition();
        float max_distance = s->max_distance;
        max_distance -= 2.0;
        int max_idx = get_path_length_idx(model_position, max_distance);

        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        bool longActive = selfdrive_state.getEnabled();
        if (longActive == false) {
            show_path_mode = show_path_mode_cruise_off;
            show_path_color = show_path_color_cruise_off;
        }
        else {
			if (active_lane_line) {
				show_path_mode = show_path_mode_lane;
				show_path_color = show_path_color_lane;
			}
            else {
                show_path_mode = show_path_mode_normal;
                show_path_color = show_path_color_normal;
            }
        }

        if (show_path_mode == 0) {
            update_line_data2(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_idx);
        }
        else if (show_path_mode < 9 || show_path_mode == 13 || show_path_mode == 14 || show_path_mode == 15) {
            update_line_data_dist(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_distance, false);
        }
        else
            update_line_data_dist3(s, model_position, show_path_width, 1.22, 1.22, &track_vertices, max_distance, false);

        return true;
    }
public:
    void draw(const UIState* s, float& pathDrawSeq) {
        params_count = (params_count + 1) % 20;
        if (params_count == 0) {
            show_path_mode_normal = params.getInt("ShowPathMode");
            show_path_color_normal = params.getInt("ShowPathColor");
            show_path_mode_lane = params.getInt("ShowPathModeLane");
            show_path_color_lane = params.getInt("ShowPathColorLane");
            show_path_mode_cruise_off = params.getInt("ShowPathModeCruiseOff");
            show_path_color_cruise_off = params.getInt("ShowPathColorCruiseOff");
        }
        if (!make_data(s)) return;
        static bool forward = true;
        int alpha = 120;
        NVGcolor colors[10] = {
            COLOR_RED_ALPHA(alpha),           nvgRGBA(255, 153, 0, alpha),
            COLOR_YELLOW_ALPHA(alpha),        COLOR_GREEN_ALPHA(alpha),
            COLOR_BLUE_ALPHA(alpha),          nvgRGBA(0, 0, 128, alpha),
            nvgRGBA(0x8b, 0, 0xff, alpha),    COLOR_OCHRE_ALPHA(alpha),
            COLOR_WHITE_ALPHA(alpha),         COLOR_BLACK_ALPHA(alpha),
        };

        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        bool brake_valid = car_state.getBrakeLights();

        if (show_path_mode == 0) {
            ui_draw_line(s, track_vertices, &colors[show_path_color % 10], nullptr,
                (show_path_color >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        else if (show_path_mode >= 13 && show_path_mode <= 15) {
            drawSpecialModes(s, show_path_mode, show_path_color, brake_valid, colors);
        }
        else if (show_path_mode >= 9) {
            drawComplexPath(s, show_path_color, brake_valid, colors);
        }
        else {
            drawAnimatedPath(s, car_state, pathDrawSeq, forward, show_path_mode, show_path_color, brake_valid, colors);
        }
    }

private:
    void drawSpecialModes(const UIState* s, int mode, int color_idx,
        bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float xp[3][128], yp[3][128];
        float g = 0.05f, gc = 0.4f;

        if (mode == 13) {
            g = 0.2f;
            gc = 0.10f;
        }
        else if (mode == 14) {
            g = 0.45f;
            gc = 0.05f;
        }
        else if (mode == 15) {
            gc = g;
        }

        int glen = track_vertices_len / 2 - 1;
        for (int i = 0; i < glen; i++) {
            int e = track_vertices_len - i - 1;
            int ge = glen * 2 - 1 - i;
            float x1 = track_vertices[i].x();
            float y1 = track_vertices[i].y();
            float x2 = track_vertices[e].x();
            float y2 = track_vertices[e].y();
            xp[0][i] = x1;
            yp[0][i] = y1;
            xp[0][ge] = x1 + (x2 - x1) * g;
            yp[0][ge] = y1 + (y2 - y1) * g;
            xp[1][i] = x1 + (x2 - x1) * (0.5f - gc);
            yp[1][i] = y1 + (y2 - y1) * (0.5f - gc);
            xp[1][ge] = x1 + (x2 - x1) * (0.5f + gc);
            yp[1][ge] = y1 + (y2 - y1) * (0.5f + gc);
            xp[2][i] = x1 + (x2 - x1) * (1.0f - g);
            yp[2][i] = y1 + (y2 - y1) * (1.0f - g);
            xp[2][ge] = x2;
            yp[2][ge] = y2;
        }
        if (mode == 13 || mode == 14) {
            ui_draw_line2(s, xp[0], yp[0], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        if (mode == 13 || mode == 15) {
            ui_draw_line2(s, xp[1], yp[1], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
        if (mode == 13 || mode == 14) {
            ui_draw_line2(s, xp[2], yp[2], glen * 2, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
        }
    }

    void drawComplexPath(const UIState* s, int color_idx, bool brake_valid,
        NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[6], y[6];
        int color_n = 0;
        for (int i = 0; i < track_vertices_len / 2 - 1; i += 3) {
            int e = track_vertices_len - i - 1;
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 1].x();
            y[1] = track_vertices[i + 1].y();
            x[2] = (track_vertices[i + 2].x() + track_vertices[e - 2].x()) / 2;
            y[2] = (track_vertices[i + 2].y() + track_vertices[e - 2].y()) / 2;
            x[3] = track_vertices[e - 1].x();
            y[3] = track_vertices[e - 1].y();
            x[4] = track_vertices[e].x();
            y[4] = track_vertices[e].y();
            x[5] = (x[1] + x[3]) / 2;
            y[5] = (y[1] + y[3]) / 2;
            ui_draw_line2(s, x, y, 6, &colors[color_idx % 10], nullptr,
                (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                brake_valid ? COLOR_RED : COLOR_WHITE);
            if (++color_n > 6) color_n = 0;
        }
    }

    void drawAnimatedPath(const UIState* s, const cereal::CarState::Reader& car_state,
        float& pathDrawSeq, bool& forward, int mode, int color_idx, bool brake_valid,
        NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float accel = car_state.getAEgo();
        float v_ego_kph = car_state.getVEgo() * MS_TO_KPH;

#ifdef __TEST
        v_ego_kph = 20.0f;
        accel = -1.2f;
#endif

        float seq = std::max(0.3f, v_ego_kph / 100.0f);
        if (accel < -1.0f) forward = false;
        if (accel > -0.5f) forward = true;
        int max_seq = std::min(track_vertices_len / 4 + 3, 16);

        static int pathDrawSeq2 = -1;

        if (forward) {
            pathDrawSeq += seq;
            if (pathDrawSeq > max_seq) {
                pathDrawSeq = (pathDrawSeq2 >= 0) ? pathDrawSeq2 : 0;
            }
        }
        else {
            pathDrawSeq -= seq;
            if (pathDrawSeq < 0) {
                pathDrawSeq = (pathDrawSeq2 >= 0) ? pathDrawSeq2 : max_seq;
            }
        }
        pathDrawSeq2 = (max_seq > 15) ? ((int)pathDrawSeq - max_seq / 2 + max_seq) % max_seq : -5;

        //float x[6], y[6];
        switch (mode) {
        case 1:
        case 2:
        case 5:
        case 6:
            drawMode1To6(s, pathDrawSeq, pathDrawSeq2, mode, color_idx, brake_valid, colors);
            break;
        case 3:
        case 4:
        case 7:
        case 8:
            drawMode3To8(s, pathDrawSeq, pathDrawSeq2, mode, color_idx, brake_valid, colors);
            break;
        }
    }

    void drawMode1To6(const UIState* s, float pathDrawSeq, int pathDrawSeq2, int mode,
        int color_idx, bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[4], y[4];
        for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 2].x();
            y[1] = track_vertices[i + 2].y();
            x[2] = track_vertices[track_vertices_len - i - 3].x();
            y[2] = track_vertices[track_vertices_len - i - 3].y();
            x[3] = track_vertices[track_vertices_len - i - 1].x();
            y[3] = track_vertices[track_vertices_len - i - 1].y();

            bool draw = ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2 ||
                pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2);
            if (track_vertices_len / 2 < 8 || mode == 5 || mode == 6) draw = true;

            if (draw) {
                int idx = (mode == 2 || mode == 6) ? color_n : color_idx % 10;
                ui_draw_line2(s, x, y, 4, &colors[idx], nullptr,
                    (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                    brake_valid ? COLOR_RED : COLOR_WHITE);
            }

            if (i > 1) color_n = (color_n + 1) % 7;
        }
    }

    void drawMode3To8(const UIState* s, float pathDrawSeq, int pathDrawSeq2, int mode,
        int color_idx, bool brake_valid, NVGcolor* colors) {
        int track_vertices_len = track_vertices.length();
        float x[6], y[6];
        for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
            x[0] = track_vertices[i].x();
            y[0] = track_vertices[i].y();
            x[1] = track_vertices[i + 2].x();
            y[1] = track_vertices[i + 2].y();
            x[2] = (track_vertices[i + 4].x() + track_vertices[track_vertices_len - i - 5].x()) / 2;
            y[2] = (track_vertices[i + 4].y() + track_vertices[track_vertices_len - i - 5].y()) / 2;
            x[3] = track_vertices[track_vertices_len - i - 3].x();
            y[3] = track_vertices[track_vertices_len - i - 3].y();
            x[4] = track_vertices[track_vertices_len - i - 1].x();
            y[4] = track_vertices[track_vertices_len - i - 1].y();
            x[5] = (x[1] + x[3]) / 2;
            y[5] = (y[1] + y[3]) / 2;

            bool draw = ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2 ||
                pathDrawSeq2 == i / 2 || pathDrawSeq2 == i / 2 - 2);
            if (track_vertices_len / 2 < 8 || mode == 7 || mode == 8) draw = true;

            if (draw) {
                int idx = (mode == 4 || mode == 8) ? color_n : color_idx % 10;
                ui_draw_line2(s, x, y, 6, &colors[idx], nullptr,
                    (color_idx >= 10 || brake_valid) ? 2.0 : 0.0,
                    brake_valid ? COLOR_RED : COLOR_WHITE);
            }

            if (i > 1) color_n = (color_n + 1) % 7;
        }
    }
};


#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>

char    carrot_man_debug[128] = "";
class DrawCarrot : public QObject {
    Q_OBJECT

protected:
    Params	params;
    Params	params_memory{ "/dev/shm/params" };
    int     icon_size = 256;
public:
    DrawCarrot() {
    }
    float   v_cruise = 0.0;
    float   v_ego = 0.0;
    float   apply_speed = 250.0;
    QString apply_source = "";
    bool    latActive = false;
    bool    longActive = false;
    int     xState = 0;
    int     trafficState = 0;
    int     trafficState_carrot = 0;
    int     active_carrot = 0;
    float   xTarget = 0.0;

    QString szPosRoadName = "";

    void updateState(UIState *s) {
        const SubMaster& sm = *(s->sm);
        const bool cs_alive = sm.alive("carState");
        const bool car_state_alive = sm.alive("carState");
        const bool car_control_alive = sm.alive("carControl");
        const bool carrot_man_alive = sm.alive("carrotMan");
        const bool lp_alive = sm.alive("longitudinalPlan");
        //const auto cs = sm["controlsState"].getControlsState();
        const auto car_state = sm["carState"].getCarState();
        const auto car_control = sm["carControl"].getCarControl();
        const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
        const auto carrot_man = sm["carrotMan"].getCarrotMan();
        const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
        auto lead_one = sm["radarState"].getRadarState().getLeadOne();
        auto model_position = model.getPosition();

        if (!cs_alive || !car_control_alive || !car_state_alive || !lp_alive) return;
        auto selfdrive_state = sm["selfdriveState"].getSelfdriveState();
        longActive = selfdrive_state.getEnabled();
        latActive = car_control.getLatActive();

        v_cruise = car_state.getVCruiseCluster();
        v_ego = car_state.getVEgoCluster();
        if (carrot_man_alive) {
            active_carrot = carrot_man.getActive();
            apply_speed = carrot_man.getDesiredSpeed();
            apply_source = QString::fromStdString(carrot_man.getDesiredSource());
            if (apply_speed >= v_cruise) apply_source = "";
            szPosRoadName = QString::fromStdString(carrot_man.getSzPosRoadName());
            QString atcType = QString::fromStdString(carrot_man.getAtcType());
            trafficState_carrot = carrot_man.getTrafficState();
            const auto velocity = model.getVelocity();

            auto meta = sm["modelV2"].getModelV2().getMeta();
            QString desireLog = QString::fromStdString(meta.getDesireLog());
            sprintf(carrot_man_debug, "model_kph= %d, %s, %dkm/h TBT(%d): %dm, CAM(%d): %dkm/h, %dm, ATC(%s), T(%d)",
                (int)(velocity.getX()[32] * 3.6),
                desireLog.toStdString().c_str(),
                carrot_man.getDesiredSpeed(),
                carrot_man.getXTurnInfo(),
                carrot_man.getXDistToTurn(),
                carrot_man.getXSpdType(),
                carrot_man.getXSpdLimit(),
                carrot_man.getXSpdDist(),
                atcType.toStdString().c_str(),
                carrot_man.getTrafficState());
        }
		else {
            active_carrot = 0;
			apply_speed = 250.0;
            apply_source = "";
            carrot_man_debug[0] = 0;
            szPosRoadName = "";
		}
        xState = lp.getXState();
        trafficState = lp.getTrafficState();
        xTarget = lp.getXTarget();

        s->max_distance = std::clamp(*(model_position.getX().end() - 1),
            MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

        if (lead_one.getStatus()) {
            const float lead_d = lead_one.getDRel();
            s->max_distance = std::clamp((float)lead_d, 0.0f, s->max_distance);
        }
	}
    void drawDebug(UIState* s) {
        if (params.getInt("ShowDebugUI") > 0) {
            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            ui_draw_text(s, s->fb_w, s->fb_h - 10, carrot_man_debug, 35, COLOR_WHITE, BOLD, 1.0f, 1.0f);
        }
    }
    char    cruise_speed_last[32] = "";
    char    driving_mode_str_last[32] = "";
    int     gap_last = 0;
    void drawHud(UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);

        int x = 150;// 120;
        int y = s->fb_h - 420;// 300;// 410;

        int bx = x;
        int by = y + 270;

        ui_fill_rect(s->vg, { bx - 120, by - 145, 420, 270}, COLOR_BLACK_ALPHA(90), 30);


        // draw traffic light
        int icon_red = icon_size;
        int icon_green = icon_size;
        bool red_light = trafficState == 1;
        bool green_light = trafficState == 2;

        if(trafficState_carrot == 1) {
			red_light = true;
            icon_red *= 1.5;
		}
		else if(trafficState_carrot == 2) {
			green_light = true;
            icon_green *= 1.5;
		}
        if (red_light) ui_draw_image(s, { x - icon_red / 2, y - icon_red / 2 + 270, icon_red, icon_red }, "ic_traffic_red", 1.0f);
        else if (green_light) ui_draw_image(s, { x - icon_green / 2, y - icon_green / 2 + 270, icon_green, icon_green }, "ic_traffic_green", 1.0f);

        // draw speed
        char speed[32];
        sprintf(speed, "%.0f", v_ego * 3.6);
        ui_draw_text(s, bx, by + 50, speed, 120, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        ui_draw_image(s, { bx - 100, by - 60, 350, 150 }, "ic_speed_bg", 1.0f);

        // draw cruise speed
        char cruise_speed[32];
        int cruise_x = bx + 170;
        int cruise_y = by + 15;
        if(longActive) sprintf(cruise_speed, "%.0f", v_cruise);
		else sprintf(cruise_speed, "--");
        if (strcmp(cruise_speed_last, cruise_speed) != 0) {
			strcpy(cruise_speed_last, cruise_speed);
            if(strcmp(cruise_speed, "--"))
                ui_draw_text_a(s, cruise_x, cruise_y, cruise_speed, 60, COLOR_GREEN, BOLD);
		}
        ui_draw_text(s, cruise_x, cruise_y, cruise_speed, 60, COLOR_GREEN, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);

        // draw apply speed
        NVGcolor textColor = COLOR_GREEN;
        char apply_speed_str[32];
        int apply_x = bx + 250;
        int apply_y = by - 50;

        if (apply_source.length()) {
            sprintf(apply_speed_str, "%.0f", apply_speed);
            textColor = COLOR_OCHRE;    // apply speed가 작동되면... 색을 바꾸자.
            ui_draw_text(s, apply_x, apply_y, apply_speed_str, 50, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            ui_draw_text(s, apply_x, apply_y - 50, apply_source.toStdString().c_str(), 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
        }
		else if(abs(xTarget - v_cruise) > 0.5) {
            sprintf(apply_speed_str, "%.0f", xTarget);
			ui_draw_text(s, apply_x, apply_y, apply_speed_str, 50, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            ui_draw_text(s, apply_x, apply_y - 50, "eco", 30, textColor, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
		}

        // draw gap info
        char driving_mode_str[32] = "연비";
        int driving_mode = params.getInt("MyDrivingMode");
        switch (driving_mode) {
        case 1: strcpy(driving_mode_str, tr("ECO").toStdString().c_str()); break;
        case 2: strcpy(driving_mode_str, tr("SAFE").toStdString().c_str()); break;
        case 3: strcpy(driving_mode_str, tr("NORM").toStdString().c_str()); break;
        case 4: strcpy(driving_mode_str, tr("FAST").toStdString().c_str()); break;
        default: strcpy(driving_mode_str, tr("ERRM").toStdString().c_str()); break;
        }
        int dx = bx + 50;
        int dy = by + 110;
        ui_draw_text(s, dx, dy, driving_mode_str, 30, COLOR_WHITE, BOLD);
        if (strcmp(driving_mode_str, driving_mode_str_last)) ui_draw_text_a(s, dx, dy, driving_mode_str, 30, COLOR_WHITE, BOLD);
        strcpy(driving_mode_str_last, driving_mode_str);

        char gap_str[32];
        int gap = params.getInt("LongitudinalPersonality") + 1;
        dx = bx + 220;
        dy = by + 77;
        sprintf(gap_str, "%d", gap);
        ui_draw_text(s, dx, dy, gap_str, 40, COLOR_WHITE, BOLD);
        if (gap_last != gap) ui_draw_text_a(s, dx, dy, gap_str, 40, COLOR_WHITE, BOLD);
        gap_last = gap;
    }
    void drawDateTime(const UIState* s) {
        char str[128];
        // 시간표시
        int show_datetime = params.getInt("ShowDateTime");
        if (show_datetime) {
            time_t now = time(nullptr);
            struct tm* local = localtime(&now);

            int x = s->fb_w - 250;
            int y = 150;
            int nav_y = y + 50;

            nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
            if (show_datetime == 1 || show_datetime == 2) {
                strftime(str, sizeof(str), "%H:%M", local);
                ui_draw_text(s, x, y, str, 100, COLOR_WHITE, BOLD, 3.0f, 8.0f);

            }
            if (show_datetime == 1 || show_datetime == 3) {
                strftime(str, sizeof(str), "%m-%d-%a", local);
                ui_draw_text(s, x, y + 70, str, 60, COLOR_WHITE, BOLD, 3.0f, 8.0f);
                nav_y += 70;
            }
            if (szPosRoadName.size() > 0) {
                nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
                ui_draw_text(s, x, nav_y, szPosRoadName.toStdString().c_str(), 35, COLOR_WHITE, BOLD, 3.0f, 8.0f);
            }
        }
    }
    void drawConnInfo(const UIState* s) {
        int y = 10;
        int x = 30;
        if (false) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_scc2", 1.0f);
            x += 135;
        }
        if (active_carrot >= 2) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apn", 1.0f);
            x += 135;
        }
        //else if (hda_speedLimit > 0 && hda_speedLimitDistance > 0) {
        //    ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
        //    x += 135;
        //}
        else if (active_carrot >= 1) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_apm", 1.0f);
            x += 135;
        }
        else if (false) {
            ui_draw_image(s, { x, y, 120, 54 }, "ic_hda", 1.0f);
            x += 135;
        }
        if (false) {
            ui_draw_image(s, { x, y, 240, 54 }, "ic_radartracks", 1.0f);
            x += (120 + 135);
        }
    }
    NVGcolor get_tpms_color(float tpms) {
        if (tpms < 5 || tpms > 60) // N/A
            return nvgRGBA(255, 255, 255, 220);
        if (tpms < 31)
            return nvgRGBA(255, 90, 90, 220);
        return nvgRGBA(255, 255, 255, 220);
    }

    const char* get_tpms_text(float tpms) {
        if (tpms < 5 || tpms > 60) return "  -";
        static char str[32];
        snprintf(str, sizeof(str), "%.0f", round(tpms));
        return str;
    }
    void drawTpms(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();

        int bx = (192 - 24) / 2 + UI_BORDER_SIZE + 10;
        int by = s->fb_h - 280 / 2 + 10;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
        ui_draw_text(s, bx - 90, by - 55, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
        ui_draw_text(s, bx + 90, by - 55, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
        ui_draw_text(s, bx - 90, by + 80, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
        ui_draw_text(s, bx + 90, by + 80, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
    }
    void drawTpms2(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();

        int bx = s->fb_w - 125;
        int by = 130;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
        //fl = fr = rl = rr = 29;
        int dw = 80;
        ui_draw_text(s, bx - dw, by - 55, get_tpms_text(fl), 30, get_tpms_color(fl), BOLD);
        ui_draw_text(s, bx + dw, by - 55, get_tpms_text(fr), 30, get_tpms_color(fr), BOLD);
        ui_draw_text(s, bx - dw, by + 70, get_tpms_text(rl), 30, get_tpms_color(rl), BOLD);
        ui_draw_text(s, bx + dw, by + 70, get_tpms_text(rr), 30, get_tpms_color(rr), BOLD);
    }
    void drawDeviceInfo(const UIState* s) {
        nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
        SubMaster& sm = *(s->sm);
        auto deviceState = sm["deviceState"].getDeviceState();
        const auto freeSpace = deviceState.getFreeSpacePercent();
        const auto memoryUsage = deviceState.getMemoryUsagePercent();
        const auto cpuTempC = deviceState.getCpuTempC();
        const auto cpuUsagePercent = deviceState.getCpuUsagePercent();
        float cpuTemp = 0.0f;
        QString str = "";
        int   size = sizeof(cpuTempC) / sizeof(cpuTempC[0]);
        if (size > 0) {
            for (int i = 0; i < size; i++) {
                cpuTemp += cpuTempC[i];
            }
            cpuTemp /= static_cast<float>(size);
        }
        float cpuUsage = 0.0f;
        size = sizeof(cpuUsagePercent) / sizeof(cpuUsagePercent[0]);
        if (size > 0) {
            int cpu_size = 0;
            for (cpu_size = 0; cpu_size < size; cpu_size++) {
                if (cpuUsagePercent[cpu_size] <= 0) break;
                cpuUsage += cpuUsagePercent[cpu_size];
            }
            if (cpu_size > 0) cpuUsage /= cpu_size;
        }
        str.sprintf("MEM:%d%% DISK:%.0f%% CPU:%.0f%%,%.0f\u00B0C", memoryUsage, freeSpace, cpuUsage, cpuTemp);
        NVGcolor top_right_color = (cpuTemp > 85.0 || memoryUsage > 85.0) ? COLOR_ORANGE : COLOR_WHITE;
		ui_draw_text(s, s->fb_w - 10, 2, str.toStdString().c_str(), 30, top_right_color, BOLD, 3.0f, 1.0f);
    }

};


DrawPlot drawPlot;
DrawCarrot drawCarrot;
BlindSpotDrawer drawBlindSpot;
PathDrawer drawPath;
LaneLineDrawer drawLaneLine;
PathEndDrawer drawPathEnd;
DesireDrawer drawDesire;
TurnInfoDrawer drawTurnInfo;

OnroadAlerts::Alert alert;
NVGcolor alert_color;

void ui_draw(UIState *s, ModelRenderer* model_renderer, int w, int h) {
  _model = model_renderer;
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  //nvgFontSize(s->vg, 170);
  //nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  //ui_draw_text(s, 500, 500, "Carrot", 100, COLOR_GREEN, BOLD);
  Params params;
  drawCarrot.updateState(s);
  static float pathDrawSeq = 0.0;
  int show_lane_info = params.getInt("ShowLaneInfo");
  if(show_lane_info >= 0) drawPath.draw(s, pathDrawSeq);
  drawLaneLine.draw(s, show_lane_info);
  if (params.getInt("ShowPathEnd") > 0) drawPathEnd.draw(s);

  int path_x = drawPathEnd.getPathX();
  int path_y = drawPathEnd.getPathY();
  drawDesire.draw(s, path_x, path_y - 135);
  

  drawPlot.draw(s);

  drawBlindSpot.draw(s);

  drawCarrot.drawHud(s);

  drawCarrot.drawDebug(s);
  drawCarrot.drawDateTime(s);
  drawCarrot.drawConnInfo(s);
  drawCarrot.drawDeviceInfo(s);
  drawCarrot.drawTpms2(s);

  drawTurnInfo.draw(s);

  ui_draw_text_a2(s);
  ui_draw_alert(s);

  nvgResetScissor(s->vg);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

class BorderDrawer {
protected:
    float   a_ego_width = 0.0;
    float steering_angle_pos = 0.0;
    NVGcolor get_tpms_color(float tpms) {
        if (tpms < 5 || tpms > 60) // N/A
            return COLOR_GREEN;
        if (tpms < 31)
            return COLOR_RED;
        return COLOR_GREEN;
    }

    const char* get_tpms_text(float tpms) {
        if (tpms < 5 || tpms > 60) return "-";
        static char str[32];
        snprintf(str, sizeof(str), "%.0f", round(tpms));
        return str;
    }
public:
    void drawTpms(UIState* s, int w, int h) {
        NVGcontext* vg = s->vg_border;
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        SubMaster& sm = *(s->sm);
        if (!sm.alive("carState")) return;
        auto car_state = sm["carState"].getCarState();

        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();

        //fl = fr = rl = rr = 29;
        //fl = fr = 40;
        ui_draw_text_vg(vg, 25, 30, get_tpms_text(fl), 30, get_tpms_color(fl), BOLD, 3);
        ui_draw_text_vg(vg, w - 25, 30, get_tpms_text(fr), 30, get_tpms_color(fr), BOLD, 3);
        ui_draw_text_vg(vg, 25, h, get_tpms_text(rl), 30, get_tpms_color(rl), BOLD, 3);
        ui_draw_text_vg(vg, w - 25, h, get_tpms_text(rr), 30, get_tpms_color(rr), BOLD, 3);
    }
    void draw(UIState *s, int w, int h, NVGcolor bg, NVGcolor bg_long) {
        NVGcontext* vg = s->vg_border;

        ui_fill_rect(vg, { 0,0, w, h / 2  - 100}, bg, 15);
        ui_fill_rect(vg, { 0, h / 2 + 100, w, h }, bg_long, 15);

        ui_fill_rect(vg, {w - 50, h/2 - 95, 50, 190}, (_right_blinker)?COLOR_ORANGE:COLOR_BLACK, 15);
        ui_fill_rect(vg, {0, h/2 - 95, 50, 190}, (_left_blinker)?COLOR_ORANGE:COLOR_BLACK, 15);

        const SubMaster& sm = *(s->sm);
        auto car_state = sm["carState"].getCarState();
        float a_ego = car_state.getAEgo();

        a_ego_width = a_ego_width * 0.5 + (w * std::abs(a_ego) / 4.0) * 0.5;
        ui_fill_rect(vg, { w/2 - (int)(a_ego_width / 2), h - 30, (int)a_ego_width, 30 }, (a_ego >= 0)? COLOR_YELLOW : COLOR_RED, 15);

        steering_angle_pos = steering_angle_pos * 0.5 + (w / 2. - w / 2. * car_state.getSteeringAngleDeg() / 90) * 0.5;
        int x_st = (int)steering_angle_pos - 50;
        int x_ed = (int)steering_angle_pos + 50;
        if (x_st < 0) x_st = 0;
        if (x_ed < 50) x_ed = 50;
        if (x_st > w - 50) x_st = w - 50;
        if (x_ed > w) x_ed = w;
        ui_fill_rect(vg, { x_st, 0, x_ed - x_st, 30 }, COLOR_ORANGE, 15);


        char top[256] = "", top_left[256] = "", top_right[256] = "";
        char bottom[256] = "", bottom_left[256] = "", bottom_right[256] = "";

        QString str;

        // top
        str = QString::fromStdString(car_state.getLogCarrot());
        sprintf(top, "%s", str.toStdString().c_str());
        // top_right
        const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
        str.sprintf("LT[%.0f]:%s (%.4f/%.4f)",
            live_torque_params.getTotalBucketPoints(), live_torque_params.getLiveValid() ? "ON" : "OFF", live_torque_params.getLatAccelFactorFiltered(), live_torque_params.getFrictionCoefficientFiltered());
        sprintf(top_right, "%s", str.toStdString().c_str());

        //top_left
        Params params = Params();
        QString carName = QString::fromStdString(params.get("CarName"));
        bool longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
        if (params.getInt("HyundaiCameraSCC") > 0) {
            carName += "(CAMERA SCC)";
        }
        else if (longitudinal_control) {
            carName += " - OP Long";
        }
        sprintf(top_left, "%s", carName.toStdString().c_str());

        // bottom
        const auto lat_plan = sm["lateralPlan"].getLateralPlan();
        str = lat_plan.getLatDebugText().cStr();
        strcpy(bottom, str.toStdString().c_str());

        // bottom_left
        QString gitBranch = QString::fromStdString(params.get("GitBranch"));
        sprintf(bottom_left, "%s", gitBranch.toStdString().c_str());

        // bottom_right
        Params params_memory = Params("/dev/shm/params");
        if (false && carrot_man_debug[0] != 0 && params.getInt("ShowDebugUI") > 0) {
            strcpy(bottom_right, carrot_man_debug);
        }
        else {
            QString ipAddress = QString::fromStdString(params_memory.get("NetworkAddress"));
            //extern QString gitBranch;
            sprintf(bottom_right, "%s", ipAddress.toStdString().c_str());
        }

        int text_margin = 30;
        // top
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, w / 2, 0, top, 30, COLOR_WHITE, BOLD);
        // top left
        nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, text_margin, 0, top_left, 30, COLOR_WHITE, BOLD);
        // top right
        nvgTextAlign(vg, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
        ui_draw_text_vg(vg, w - text_margin, 0, top_right, 30, COLOR_WHITE, BOLD);
        // bottom
        nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, w / 2, h, bottom, 30, COLOR_WHITE, BOLD);
        // bottom left
        nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, text_margin, h, bottom_left, 30, COLOR_WHITE, BOLD);
        // bottom right
        nvgTextAlign(vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
        ui_draw_text_vg(vg, w- text_margin, h, bottom_right, 30, COLOR_WHITE, BOLD);

        //drawTpms(s, w, h);
    }
};
NVGcolor QColorToNVGcolor(const QColor& color) {
    return nvgRGBAf(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}
BorderDrawer borderDrawer;
void ui_draw_border(UIState* s, int w, int h, QColor bg, QColor bg_long) {

    if (s->vg_border == nullptr) {
        s->vg_border = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
        std::pair<const char*, const char*> fonts[] = {
            {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
        };
        for (auto [name, file] : fonts) {
            int font_id = nvgCreateFont(s->vg_border, name, file);
            assert(font_id >= 0);
        }
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glViewport(0, 0, w, h);
    nvgBeginFrame(s->vg_border, w, h, 1.0f);
	nvgScissor(s->vg_border, 0, 0, w, h);

    borderDrawer.draw(s, w, h, QColorToNVGcolor(bg), QColorToNVGcolor(bg_long));

    nvgResetScissor(s->vg_border);
    nvgEndFrame(s->vg_border);
    glDisable(GL_BLEND);
}
void ui_draw_alert(UIState* s) {
    if (alert.size != cereal::SelfdriveState::AlertSize::NONE) {
        alert_color = COLOR_ORANGE;
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 300, alert.text1.toStdString().c_str(), 100, alert_color, BOLD, 3.0f, 8.0f);
        ui_draw_text(s, s->fb_w / 2, s->fb_h - 200, alert.text2.toStdString().c_str(), 70, alert_color, BOLD, 3.0f, 8.0f);
    }
}
void ui_update_alert(const OnroadAlerts::Alert& a) {
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
      //{"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      //{"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      //{"sans-bold", "../assets/fonts/opensans_bold.ttf"},
      //{"KaiGenGothicKR-Normal", "../assets/addon/font/KaiGenGothicKR-Normal.ttf"},
      //{"KaiGenGothicKR-Medium", "../assets/addon/font/KaiGenGothicKR-Medium.ttf"},
      {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
      //{"Inter-Bold", "../assets/fonts/Inter-Bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }
  // init images
  std::vector<std::pair<const char *, const char *>> images = {
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
  {"ic_navi_point", "../assets/images/navi_point.png"}

  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
}
void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;
}

#include "selfdrive/ui/carrot.moc"
