#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>

#include <OMX_Component.h>
extern "C" {
#include <libavformat/avformat.h>
}

#include "common/queue.h"

// OmxEncoder, lossey codec using hardware hevc
class OmxEncoder {
public:
  OmxEncoder(const char* path, int width, int height, int fps, int bitrate, bool h265, bool downscale);
  ~OmxEncoder();

  int encode_frame_rgba(const uint8_t *ptr, int in_width, int in_height, uint64_t ts);
  void encoder_open(const char* filename);
  void encoder_close();

  // OMX callbacks
  static OMX_ERRORTYPE event_handler(OMX_HANDLETYPE component, OMX_PTR app_data, OMX_EVENTTYPE event,
                                     OMX_U32 data1, OMX_U32 data2, OMX_PTR event_data);
  static OMX_ERRORTYPE empty_buffer_done(OMX_HANDLETYPE component, OMX_PTR app_data,
                                         OMX_BUFFERHEADERTYPE *buffer);
  static OMX_ERRORTYPE fill_buffer_done(OMX_HANDLETYPE component, OMX_PTR app_data,
                                        OMX_BUFFERHEADERTYPE *buffer);

private:
  void wait_for_state(OMX_STATETYPE state);
  static void handle_out_buf(OmxEncoder *e, OMX_BUFFERHEADERTYPE *out_buf);

  int width, height, fps;
  char vid_path[1024];
  char lock_path[1024];
  bool is_open = false;
  bool dirty = false;
  int counter = 0;

  std::string path;
  FILE *of = nullptr;

  size_t codec_config_len = 0;
  uint8_t *codec_config = NULL;
  bool wrote_codec_config = false;

  std::mutex state_lock;
  std::condition_variable state_cv;
  OMX_STATETYPE state = OMX_StateLoaded;

  OMX_HANDLETYPE handle = nullptr;

  std::vector<OMX_BUFFERHEADERTYPE *> in_buf_headers;
  std::vector<OMX_BUFFERHEADERTYPE *> out_buf_headers;

  uint64_t last_t = 0;

  SafeQueue<OMX_BUFFERHEADERTYPE *> free_in;
  SafeQueue<OMX_BUFFERHEADERTYPE *> done_out;

  AVFormatContext *ofmt_ctx = nullptr;
  AVCodecContext *codec_ctx = nullptr;
  AVStream *out_stream = nullptr;
  bool remuxing = false;

  bool downscale = false;
  uint8_t *y_ptr2 = nullptr, *u_ptr2 = nullptr, *v_ptr2 = nullptr;
};
