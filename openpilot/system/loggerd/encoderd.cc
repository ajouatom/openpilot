#include <cassert>

#include "common/params.h"
#include "system/loggerd/loggerd.h"
#include "system/loggerd/encoder/jpeg_encoder.h"

#ifdef __TICI__
#include "system/loggerd/encoder/v4l_encoder.h"
#define Encoder V4LEncoder
#else
#include "system/loggerd/encoder/ffmpeg_encoder.h"
#define Encoder FfmpegEncoder
#endif

ExitHandler do_exit;

struct EncoderdState {
  int max_waiting = 0;

  // Sync logic for startup
  std::atomic<int> encoders_ready = 0;
  std::atomic<uint32_t> start_frame_id = 0;
  bool camera_ready[VISION_STREAM_WIDE_ROAD + 1] = {};
  bool camera_synced[VISION_STREAM_WIDE_ROAD + 1] = {};
};

// Handle initial encoder syncing by waiting for all encoders to reach the same frame id
bool sync_encoders(EncoderdState *s, VisionStreamType cam_type, uint32_t frame_id) {
  if (s->camera_synced[cam_type]) return true;

  if (s->max_waiting > 1 && s->encoders_ready != s->max_waiting) {
    // add a small margin to the start frame id in case one of the encoders already dropped the next frame
    update_max_atomic(s->start_frame_id, frame_id + 2);
    if (std::exchange(s->camera_ready[cam_type], true) == false) {
      ++s->encoders_ready;
      LOGD("camera %d encoder ready", cam_type);
    }
    return false;
  } else {
    if (s->max_waiting == 1) update_max_atomic(s->start_frame_id, frame_id);
    bool synced = frame_id >= s->start_frame_id;
    s->camera_synced[cam_type] = synced;
    if (!synced) LOGD("camera %d waiting for frame %d, cur %d", cam_type, (int)s->start_frame_id, frame_id);
    return synced;
  }
}


void encoder_thread(EncoderdState *s, const LogCameraInfo &cam_info, bool encode_on_demand) {
  util::set_thread_name(cam_info.thread_name);

  std::vector<std::unique_ptr<Encoder>> encoders;
  VisionIpcClient vipc_client = VisionIpcClient("camerad", cam_info.stream_type, false);
  Params params;

  std::unique_ptr<JpegEncoder> jpeg_encoder;

  int cur_seg = 0;
  bool prewarm_pending = encode_on_demand;
  while (!do_exit) {
    if (!vipc_client.connect(false)) {
      util::sleep_for(5);
      continue;
    }

    // init encoders
    if (encoders.empty()) {
      const VisionBuf &buf_info = vipc_client.buffers[0];
      LOGW("encoder %s init %zux%zu", cam_info.thread_name, buf_info.width, buf_info.height);
      assert(buf_info.width > 0 && buf_info.height > 0);

      for (const auto &encoder_info : cam_info.encoder_infos) {
        auto &e = encoders.emplace_back(new Encoder(encoder_info, buf_info.width, buf_info.height));
        e->set_idle(false);
        e->encoder_open();
      }

      // Only one thumbnail can be generated per camera stream
      if (auto thumbnail_name = cam_info.encoder_infos[0].thumbnail_name) {
        jpeg_encoder = std::make_unique<JpegEncoder>(thumbnail_name, buf_info.width / 4, buf_info.height / 4);
      }
    }

    bool lagging = false;
    while (!do_exit) {
      VisionIpcBufExtra extra;
      VisionBuf* buf = vipc_client.recv(&extra);
      if (buf == nullptr) continue;

      // detect loop around and drop the frames
      if (buf->get_frame_id() != extra.frame_id) {
        if (!lagging) {
          LOGE("encoder %s lag  buffer id: %" PRIu64 " extra id: %d", cam_info.thread_name, buf->get_frame_id(), extra.frame_id);
          lagging = true;
        }
        continue;
      }
      lagging = false;

      if (!sync_encoders(s, cam_info.stream_type, extra.frame_id)) {
        continue;
      }
      if (do_exit) break;

      // Prime VIDC with one frame so firmware loading, DMA mapping, and codec
      // headers are complete before a user requests Carrot Vision.
      const bool session_active = !encode_on_demand || params.getBool("CarrotVisionActive");
      const bool idle = !session_active && !prewarm_pending;
      for (auto &e : encoders) {
        e->set_idle(idle);
      }
      if (idle) {
        continue;
      }

      // do rotation if required
      const int frames_per_seg = SEGMENT_LENGTH * MAIN_FPS;
      if (!encode_on_demand && cur_seg >= 0 && extra.frame_id >= ((cur_seg + 1) * frames_per_seg) + s->start_frame_id) {
        for (auto &e : encoders) {
          e->encoder_close();
          e->encoder_open();
        }
        ++cur_seg;
      }

      // encode a frame
      for (int i = 0; i < encoders.size(); ++i) {
        int out_id = encoders[i]->encode_frame(buf, &extra);

        if (out_id == -1) {
          LOGE("Failed to encode frame. frame_id: %d", extra.frame_id);
        }
      }
      prewarm_pending = false;

      if (jpeg_encoder && (extra.frame_id % 1200 == 100)) {
        jpeg_encoder->pushThumbnail(buf, extra);
      }
    }
  }
}

template <size_t N>
void encoderd_thread(const LogCameraInfo (&cameras)[N], bool encode_on_demand = false) {
  EncoderdState s;

  std::set<VisionStreamType> streams;
  while (!do_exit) {
    streams = VisionIpcClient::getAvailableStreams("camerad", false);
    if (!streams.empty()) {
      break;
    }
    util::sleep_for(100);
  }

  if (!streams.empty()) {
    std::vector<std::thread> encoder_threads;
    for (auto stream : streams) {
      auto it = std::find_if(std::begin(cameras), std::end(cameras),
                             [stream](auto &cam) { return cam.stream_type == stream; });
      if (it == std::end(cameras)) {
        continue;
      }
      ++s.max_waiting;
      encoder_threads.push_back(std::thread(encoder_thread, &s, *it, encode_on_demand));
    }

    for (auto &t : encoder_threads) t.join();
  }
}

int main(int argc, char* argv[]) {
  const std::string mode = argc > 1 ? argv[1] : "";
  const bool carrot_vision_mode = mode == "--carrot-vision-road";
  if (!Hardware::PC()) {
    // Carrot Vision is optional and may drop frames. Keeping its V4L setup and
    // worker threads out of SCHED_FIFO prevents on-demand video work from
    // delaying control and camera services.
    if (!carrot_vision_mode) {
      int ret = util::set_realtime_priority(52);
      assert(ret == 0);
    }
    // Main/logging encoders keep their established core 3 placement. Keep the
    // independent Carrot Vision encoder off the UI and camera/control cores.
    int ret = util::set_core_affinity({carrot_vision_mode ? 0 : 3});
    assert(ret == 0);
  }
  if (argc > 1) {
    if (mode == "--stream") {
      encoderd_thread(stream_cameras_logged);
    } else if (carrot_vision_mode) {
      encoderd_thread(carrot_vision_cameras_logged, true);
    } else if (mode == "--youtube-low") {
      encoderd_thread(youtube_low_cameras_logged);
    } else if (mode == "--youtube-medium") {
      encoderd_thread(youtube_medium_cameras_logged);
    } else if (mode == "--youtube") {
      encoderd_thread(youtube_cameras_logged);
    } else if (mode == "--youtube-wide") {
      encoderd_thread(youtube_wide_cameras_logged);
    } else {
      LOGE("Argument '%s' is not supported", mode.c_str());
    }
  } else {
    encoderd_thread(cameras_logged);
  }
  return 0;
}
