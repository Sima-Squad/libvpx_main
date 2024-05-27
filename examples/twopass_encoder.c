/*
 *  Copyright (c) 2010 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Two Pass Encoder
// ================
//
// This is an example of a two pass encoder loop. It takes an input file in
// YV12 format, passes it through the encoder twice, and writes the compressed
// frames to disk in IVF format. It builds upon the simple_encoder example.
//
// Twopass Variables
// -----------------
// Twopass mode needs to track the current pass number and the buffer of
// statistics packets.
//
// Updating The Configuration
// ---------------------------------
// In two pass mode, the configuration has to be updated on each pass. The
// statistics buffer is passed on the last pass.
//
// Encoding A Frame
// ----------------
// Encoding a frame in two pass mode is identical to the simple encoder
// example. To increase the quality while sacrificing encoding speed,
// VPX_DL_BEST_QUALITY can be used in place of VPX_DL_GOOD_QUALITY.
//
// Processing Statistics Packets
// -----------------------------
// Each packet of type `VPX_CODEC_CX_FRAME_PKT` contains the encoded data
// for this frame. We write a IVF frame header, followed by the raw data.
//
//
// Pass Progress Reporting
// -----------------------------
// It's sometimes helpful to see when each pass completes.
//
//
// Clean-up
// -----------------------------
// Destruction of the encoder instance must be done on each pass. The
// raw image should be destroyed at the end as usual.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "vpx/vpx_encoder.h"

#include "../tools_common.h"
#include "../video_writer.h"
#include "vpx/vpx_codec.h"
#include "../vpxenc.h"
#include "vpx/vp8cx.h"

// #include "../vpxenc.c"

static const char *exec_name;

void usage_exit(void) {
  fprintf(stderr,
          "Usage: %s <codec> <width> <height> <infile> <outfile> "
          "<frame limit>\n",
          exec_name);
  exit(EXIT_FAILURE);
}

// storing first pass stats
typedef struct {
  double frame;
  double weight;
  double intra_error;
  double coded_error;
  double sr_coded_error;
  double frame_noise_energy;
  double pcnt_inter;
  double pcnt_motion;
  double pcnt_second_ref;
  double pcnt_neutral;
  double pcnt_intra_low;   // Coded intra but low variance
  double pcnt_intra_high;  // Coded intra high variance
  double intra_skip_pct;
  double intra_smooth_pct;    // % of blocks that are smooth
  double inactive_zone_rows;  // Image mask rows top and bottom.
  double inactive_zone_cols;  // Image mask columns at left and right edges.
  double MVr;
  double mvr_abs;
  double MVc;
  double mvc_abs;
  double MVrv;
  double MVcv;
  double mv_in_out_count;
  double duration;
  double count;
  double new_mv_count; // not used in first pass features
  int64_t spatial_layer_id; // not used in first pass features
} FIRSTPASS_STATS;

typedef struct {
    FIRSTPASS_STATS* stats;
    size_t size;
    size_t capacity;
} StatsArray;


void addStat(StatsArray *sa, FIRSTPASS_STATS stat) {
    if (sa->size == sa->capacity) {
        sa->capacity *= 2;
        sa->stats = (FIRSTPASS_STATS*) realloc(sa->stats, sa->capacity * sizeof(FIRSTPASS_STATS));
    }
    sa->stats[sa->size++] = stat;
}

void removeStat(StatsArray *sa, size_t index) {
    if (index < sa->size) {
        for (size_t i = index; i < sa->size - 1; i++) {
            sa->stats[i] = sa->stats[i + 1];
        }
        sa->size--;
    }
}

void initStatsArray(StatsArray *sa, size_t initialCapacity) {
    sa->stats = (FIRSTPASS_STATS*) malloc(initialCapacity * sizeof(FIRSTPASS_STATS));
    sa->size = 0;
    sa->capacity = initialCapacity;
}

void freeStatsArray(StatsArray *sa) {
    free(sa->stats);
}

typedef struct {
    double psnr;
    double bitrate;
} EncodingResult;

// function prototypes for python interaction
StatsArray initialize_encoder(const char *infile, int width, int height);
EncodingResult encode_frame_external(int qp);
void finalize_encoder(void);
vpx_codec_enc_cfg_t cfg;
vpx_image_t raw;
vpx_fixed_buf_t stats;
StatsArray sa;

// defining a global file reader to read frames from a file
typedef struct {
    FILE *file;
} GlobalFileReader;

GlobalFileReader glob_reader = {NULL}; 
int glob_frame_counter = 0;


void set_global_file_reader(const char *filename) {
    freeStatsArray(&sa);
    if (glob_reader.file != NULL) {
        fclose(glob_reader.file);
    }
    glob_reader.file = fopen(filename, "rb");
    if (!glob_reader.file) {
        fprintf(stderr, "Failed to open %s for reading\n", filename);
        exit(EXIT_FAILURE);
    }
    printf("File set to %s\n", filename);
}

// orig functions begin now...
static int get_frame_stats(vpx_codec_ctx_t *ctx, const vpx_image_t *img,
                           vpx_codec_pts_t pts, unsigned int duration,
                           vpx_enc_frame_flags_t flags, unsigned int deadline,
                           vpx_fixed_buf_t *stats, StatsArray *sa) {
  int got_pkts = 0;
  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;
  const vpx_codec_err_t res =
      vpx_codec_encode(ctx, img, pts, duration, flags, deadline);
  if (res != VPX_CODEC_OK) die_codec(ctx, "Failed to get frame stats.");

  while ((pkt = vpx_codec_get_cx_data(ctx, &iter)) != NULL) {
    got_pkts = 1;

    if (pkt->kind == VPX_CODEC_STATS_PKT) {
      const uint8_t *const pkt_buf = pkt->data.twopass_stats.buf;
      const size_t pkt_size = pkt->data.twopass_stats.sz;
      stats->buf = realloc(stats->buf, stats->sz + pkt_size);
      if (!stats->buf) die("Failed to reallocate stats buffer.");
      memcpy((uint8_t *)stats->buf + stats->sz, pkt_buf, pkt_size);
      stats->sz += pkt_size;

      FIRSTPASS_STATS *fps_stats = (FIRSTPASS_STATS *)(pkt_buf);

      // add to stats array
      addStat(sa, *fps_stats);
    }
  }
  return got_pkts;
}


static int encode_frame(vpx_codec_ctx_t *ctx, const vpx_image_t *img,
                        vpx_codec_pts_t pts, unsigned int duration,
                        vpx_enc_frame_flags_t flags, unsigned int deadline,
                        VpxVideoWriter *writer, int qp, bool set_qp) {
  int got_pkts = 0;
  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;

  if (set_qp) {
    if (vpx_codec_control(ctx, VP8E_SET_CQ_LEVEL, qp)) {
      fprintf(stderr, "Failed to set constant quantization level\n");
    }
  }


  const vpx_codec_err_t res =
      vpx_codec_encode(ctx, img, pts, duration, flags, deadline);
  if (res != VPX_CODEC_OK) die_codec(ctx, "Failed to encode frame.");

  while ((pkt = vpx_codec_get_cx_data(ctx, &iter)) != NULL) {
    got_pkts = 1;
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      const int keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;

      if (!vpx_video_writer_write_frame(writer, pkt->data.frame.buf,
                                        pkt->data.frame.sz,
                                        pkt->data.frame.pts))
        die_codec(ctx, "Failed to write compressed frame.");
      printf(keyframe ? "K" : ".");
      fflush(stdout);
    }
  }

  return got_pkts;
}

// first pass to collect stats 
static vpx_fixed_buf_t pass0(vpx_image_t *raw, FILE *infile,
                             const VpxInterface *encoder,
                             const vpx_codec_enc_cfg_t *cfg, int max_frames, StatsArray *sa) {
  vpx_codec_ctx_t codec;
  int frame_count = 0;
  vpx_fixed_buf_t stats = { NULL, 0 };

  if (vpx_codec_enc_init(&codec, encoder->codec_interface(), cfg, 0))
    die("Failed to initialize encoder");

  // Calculate frame statistics.
  while (vpx_img_read(raw, infile)) {
    ++frame_count;
    get_frame_stats(&codec, raw, frame_count, 1, 0, VPX_DL_GOOD_QUALITY,
                    &stats, sa);
    if (max_frames > 0 && frame_count >= max_frames) break;
  }

  // Flush encoder.
  while (get_frame_stats(&codec, NULL, frame_count, 1, 0, VPX_DL_GOOD_QUALITY,
                         &stats, sa)) {
  }

  printf("Pass 0 complete. Processed %d frames.\n", frame_count);
  if (vpx_codec_destroy(&codec)) die_codec(&codec, "Failed to destroy codec.");

  // remove the last stat as it is not a frame stat
  removeStat(sa, sa->size - 1);

  return stats;
}

// second pass to encode frames from stats
static void pass1(vpx_image_t *raw, FILE *infile, const char *outfile_name,
                  const VpxInterface *encoder, const vpx_codec_enc_cfg_t *cfg,
                  int max_frames) {
  VpxVideoInfo info = { encoder->fourcc,
                        cfg->g_w,
                        cfg->g_h,
                        { cfg->g_timebase.num, cfg->g_timebase.den } };
  VpxVideoWriter *writer = NULL;
  vpx_codec_ctx_t codec;
  int frame_count = 0;

  writer = vpx_video_writer_open(outfile_name, kContainerIVF, &info);
  if (!writer) die("Failed to open %s for writing", outfile_name);

  if (vpx_codec_enc_init(&codec, encoder->codec_interface(), cfg, 0))
    die("Failed to initialize encoder");

  // Encode frames.
  while (vpx_img_read(raw, infile)) {
    ++frame_count;
    encode_frame(&codec, raw, frame_count, 1, 0, VPX_DL_GOOD_QUALITY, writer, 32, false);

    if (max_frames > 0 && frame_count >= max_frames) break;
  }

  // Flush encoder.
  while (encode_frame(&codec, NULL, -1, 1, 0, VPX_DL_GOOD_QUALITY, writer, 32, false)) {
  }

  printf("\n");

  if (vpx_codec_destroy(&codec)) die_codec(&codec, "Failed to destroy codec.");

  vpx_video_writer_close(writer);

  printf("Pass 1 complete. Processed %d frames.\n", frame_count);
}

StatsArray initialize_encoder(const char *infile, int width, int height) {
    set_global_file_reader(infile);

    int w, h;
    vpx_codec_ctx_t codec;
    vpx_codec_err_t res;
    
    const int fps = 30;
    const int bitrate = 200;
    const char *const codec_arg = "vp9";
    w = width;
    h = height;

    const VpxInterface *encoder = get_vpx_encoder_by_name(codec_arg);
    if (!encoder) die("Unsupported codec.");

    if (w <= 0 || h <= 0 || (w % 2) != 0 || (h % 2) != 0)
        die("Invalid frame size: %dx%d", w, h);

    if (!vpx_img_alloc(&raw, VPX_IMG_FMT_I420, w, h, 1))
        die("Failed to allocate image (%dx%d)", w, h);

    printf("Using %s\n", vpx_codec_iface_name(encoder->codec_interface()));

    // Configuration
    res = vpx_codec_enc_config_default(encoder->codec_interface(), &cfg, 0);
    if (res) die_codec(&codec, "Failed to get default codec config.");

    cfg.g_w = w;
    cfg.g_h = h;
    cfg.g_timebase.num = 1;
    cfg.g_timebase.den = fps;
    cfg.rc_target_bitrate = bitrate;

    // pass 0
    cfg.g_pass = VPX_RC_FIRST_PASS;
    stats = pass0(&raw, glob_reader.file, encoder, &cfg, 0, &sa);

    // setup for pass 1
    rewind(glob_reader.file);
    cfg.g_pass = VPX_RC_LAST_PASS;
    cfg.rc_twopass_stats_in = stats;

    return sa;
}

EncodingResult encode_frame_external(int qp) {
    // setting qp, performing encoding, and sending metrics back to python
    EncodingResult result;
    // result.psnr = calculate_psnr();
    // result.bitrate = calculate_bitrate(); 
    return result;
}

void finalize_encoder(void) {
    // memory cleaning: close files, free memory, destroy codec instances
    freeStatsArray(&sa);
    if (glob_reader.file != NULL) {
        fclose(glob_reader.file);
    }
    glob_reader.file = NULL;
    glob_frame_counter = 0;
    free(stats.buf);
    vpx_img_free(&raw);
}

int main() {
  printf("dummy print");
}

