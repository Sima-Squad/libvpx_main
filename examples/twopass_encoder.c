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
#include <math.h>

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
    bool* is_key_logs;
    bool* is_invis;
    size_t size;
    size_t capacity;
} StatsArray;

typedef struct {
    int got_pkts;
    double psnr;
    double bitrate;
} EncodingResult;


StatsArray sa;
StatsArray initialize_encoder(const char *infile, int width, int height, double fps_input, int target_bitrate);
EncodingResult encode_frame_external(int qp);
void finalize_encoder(void);


static void initStatsArray(size_t initialCapacity) {
    sa.stats = (FIRSTPASS_STATS*) malloc(initialCapacity * sizeof(FIRSTPASS_STATS));
    sa.is_key_logs = (bool*) malloc(initialCapacity * sizeof(bool));
    sa.is_invis = (bool*) malloc(initialCapacity * sizeof(bool));
    sa.size = 0;
    sa.capacity = initialCapacity;
}

static void addStat(FIRSTPASS_STATS stat, bool is_key, bool is_invis) {
    if (sa.size == sa.capacity) {
        sa.capacity *= 2;
        sa.stats = (FIRSTPASS_STATS*) realloc(sa.stats, sa.capacity * sizeof(FIRSTPASS_STATS));
        sa.is_key_logs = (bool*) realloc(sa.is_key_logs, sa.capacity * sizeof(bool));
        sa.is_invis = (bool*) realloc(sa.is_invis, sa.capacity * sizeof(bool));
    }
    sa.stats[sa.size] = stat;
    sa.is_key_logs[sa.size] = is_key;
    sa.is_invis[sa.size] = is_invis;
    sa.size++;
}

static void removeStat(size_t index) {
    if (index < sa.size) {
        for (size_t i = index; i < sa.size - 1; i++) {
            sa.stats[i] = sa.stats[i + 1];
            sa.is_key_logs[i] = sa.is_key_logs[i + 1];
            sa.is_invis[i] = sa.is_invis[i + 1];
        }
        sa.size--;
    }
}

static void freeStatsArray() {
  if (sa.stats != NULL) {
      free(sa.stats);
      sa.stats = NULL;
  }
  if (sa.is_key_logs != NULL) {
      free(sa.is_key_logs);
      sa.is_key_logs = NULL;
  }
  if (sa.is_invis != NULL) {
      free(sa.is_invis);
      sa.is_invis = NULL;
  }
  sa.size = 0;
  sa.capacity = 0;
}


typedef struct {
    FILE *infile;
    vpx_codec_ctx_t codec;
    const VpxInterface *encoder;
    vpx_image_t raw;
    vpx_codec_enc_cfg_t cfg;
    vpx_fixed_buf_t stats;
} EncoderContext;

EncoderContext enc_context = {NULL}; 
int glob_frame_counter = 0;

static void set_global_file_reader(const char *filename) {
    freeStatsArray();
    if (enc_context.infile != NULL) {
        fclose(enc_context.infile);
    }
    enc_context.infile = fopen(filename, "rb");
    if (!enc_context.infile) {
        fprintf(stderr, "Failed to open %s for reading\n", filename);
        exit(EXIT_FAILURE);
    }
    printf("File set to %s\n", filename);
}

// orig functions begin now...
static int get_frame_stats(vpx_codec_ctx_t *ctx, const vpx_image_t *img,
                           vpx_codec_pts_t pts, unsigned int duration,
                           vpx_enc_frame_flags_t flags, unsigned int deadline,
                           vpx_fixed_buf_t *stats) {
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
      uint8_t *new_buf = realloc(stats->buf, stats->sz + pkt_size);
      if (!new_buf) {
          free(stats->buf); // free old buffer to avoid memory leak
          die("Failed to reallocate stats buffer.");
      }
      stats->buf = new_buf;
      memcpy((uint8_t *)stats->buf + stats->sz, pkt_buf, pkt_size);
      stats->sz += pkt_size;

      FIRSTPASS_STATS *fps_stats = (FIRSTPASS_STATS *)(pkt_buf);

      bool is_key = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) ? 1 : 0;
      bool is_invis = (pkt->data.frame.flags & VPX_FRAME_IS_INVISIBLE) ? 1 : 0;

      // add to stats array
      addStat(*fps_stats, is_key, is_invis);
    }
  }

  return got_pkts;
}


static EncodingResult encode_frame(vpx_codec_ctx_t *ctx, const vpx_image_t *img,
                        vpx_codec_pts_t pts, unsigned int duration,
                        vpx_enc_frame_flags_t flags, unsigned int deadline,
                        VpxVideoWriter *writer, int qp, bool set_qp, bool write_to_file) {
  EncodingResult result;
  result.got_pkts = 0;
  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;

  // controlling QP
  if (set_qp) {
    if (vpx_codec_control(ctx, VP8E_SET_CQ_LEVEL, qp)) {
      fprintf(stderr, "Failed to set quantization level\n");
    }
  }

  const vpx_codec_err_t res =
      vpx_codec_encode(ctx, img, pts, duration, flags, deadline);
  if (res != VPX_CODEC_OK) die_codec(ctx, "Failed to encode frame.");


  while ((pkt = vpx_codec_get_cx_data(ctx, &iter)) != NULL) {
    result.got_pkts = 1;
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      if (write_to_file) {
        const int keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;

        if (!vpx_video_writer_write_frame(writer, pkt->data.frame.buf,
                                          pkt->data.frame.sz,
                                          pkt->data.frame.pts))
          die_codec(ctx, "Failed to write compressed frame.");
        printf(keyframe ? "K" : ".");
        fflush(stdout);
        
      }
      result.bitrate = pkt->data.frame.sz * 8;
    } else {
      if (pkt->kind == VPX_CODEC_PSNR_PKT) {
        result.psnr = pkt->data.psnr.psnr[0];
      }
    }
  }

  // result.bitrate = calculate_bitrate(); 

  return result;
}

// first pass to collect stats 
static vpx_fixed_buf_t pass0(vpx_image_t *raw, FILE *infile,
                             const VpxInterface *encoder,
                             const vpx_codec_enc_cfg_t *cfg, int max_frames) {
  vpx_codec_ctx_t codec;
  int frame_count = 0;
  vpx_fixed_buf_t stats = { NULL, 0 };

  if (vpx_codec_enc_init(&codec, encoder->codec_interface(), cfg, 0))
    die("Failed to initialize encoder");

  // Calculate frame statistics.
  while (vpx_img_read(raw, infile)) {
    ++frame_count;
    get_frame_stats(&codec, raw, frame_count, 1, 0, VPX_DL_GOOD_QUALITY,
                    &stats);
    if (max_frames > 0 && frame_count >= max_frames) break;
  }

  // Flush encoder.
  while (get_frame_stats(&codec, NULL, frame_count, 1, 0, VPX_DL_GOOD_QUALITY,
                         &stats)) {
  }

  printf("Pass 0 complete. Processed %d frames.\n", frame_count);
  if (vpx_codec_destroy(&codec)) die_codec(&codec, "Failed to destroy codec.");

  // remove the last stat as it is not a frame stat
  removeStat(sa.size - 1);

  return stats;
}

// second pass to encode frames from stats
static void pass1(vpx_image_t *raw, FILE *infile, const char *outfile_name,
                  const VpxInterface *encoder, const vpx_codec_enc_cfg_t *cfg,
                  int max_frames) {
  // VpxVideoInfo info = { encoder->fourcc,
  //                       cfg->g_w,
  //                       cfg->g_h,
  //                       { cfg->g_timebase.num, cfg->g_timebase.den } };
  VpxVideoWriter *writer = NULL;
  vpx_codec_ctx_t codec;
  int frame_count = 0;

  // writer = vpx_video_writer_open(outfile_name, kContainerIVF, &info);
  // if (!writer) die("Failed to open %s for writing", outfile_name);


  if (vpx_codec_enc_init(&codec, encoder->codec_interface(), cfg, 0))
    die("Failed to initialize encoder");

  EncodingResult res;

  // Encode frames.
  while (vpx_img_read(raw, infile)) {
    ++frame_count;
    res = encode_frame(&codec, raw, frame_count, 1, 0, VPX_DL_GOOD_QUALITY, writer, 32, true, false);

    if (max_frames > 0 && frame_count >= max_frames) break;
  }

  do {
    res = encode_frame(&codec, NULL, -1, 1, 0, VPX_DL_GOOD_QUALITY, writer, 32, true, false);
  } while (res.got_pkts);

  printf("\n");

  if (vpx_codec_destroy(&codec)) die_codec(&codec, "Failed to destroy codec.");
  vpx_video_writer_close(writer);

  printf("Pass 1 complete. Processed %d frames.\n", frame_count);
}

StatsArray initialize_encoder(const char *infile, int width, int height, double fps_input, int target_bitrate) {
    set_global_file_reader(infile);
    initStatsArray(100);

    int w, h;
    vpx_codec_err_t res;
    
    const double fps = fps_input;
    const char *const codec_arg = "vp9";
    w = width;
    h = height;

    enc_context.encoder = get_vpx_encoder_by_name(codec_arg);
    if (!enc_context.encoder) die("Unsupported codec.");

    if (w <= 0 || h <= 0 || (w % 2) != 0 || (h % 2) != 0)
        die("Invalid frame size: %dx%d", w, h);

    if (!vpx_img_alloc(&enc_context.raw, VPX_IMG_FMT_I420, w, h, 1))
        die("Failed to allocate image (%dx%d)", w, h);

    printf("Using %s\n", vpx_codec_iface_name(enc_context.encoder->codec_interface()));

    // Configuration
    res = vpx_codec_enc_config_default(enc_context.encoder->codec_interface(), &enc_context.cfg, 0);
    if (res) die_codec(&enc_context.codec, "Failed to get default codec config.");

    enc_context.cfg.g_w = w;
    enc_context.cfg.g_h = h;
    enc_context.cfg.g_timebase.num = 1;
    enc_context.cfg.g_timebase.den = fps;
    enc_context.cfg.rc_target_bitrate = target_bitrate;

    // pass 0
    enc_context.cfg.g_pass = VPX_RC_FIRST_PASS;
    enc_context.stats = pass0(&enc_context.raw, enc_context.infile, enc_context.encoder, &enc_context.cfg, 0);

    // setup for pass 1
    rewind(enc_context.infile);
    enc_context.cfg.g_pass = VPX_RC_LAST_PASS;
    enc_context.cfg.rc_twopass_stats_in = enc_context.stats;
    if (vpx_codec_enc_init(&enc_context.codec, enc_context.encoder->codec_interface(), &enc_context.cfg, VPX_CODEC_USE_PSNR))
        die("Failed to initialize encoder");

    return sa; 
}

EncodingResult encode_frame_external(int qp) {

    // setting qp, performing encoding, and sending metrics back to python
    EncodingResult result;
    ++glob_frame_counter;
    // read frame
    if (!vpx_img_read(&enc_context.raw, enc_context.infile)) {
        return result;
    }

    result = encode_frame(&enc_context.codec, &enc_context.raw, glob_frame_counter, 1, 0, VPX_DL_GOOD_QUALITY, NULL, qp, true, false);
    return result;
}

void finalize_encoder(void) {

    printf("Pass 1 complete. Processed %d frames.\n", glob_frame_counter);
    printf("Cleaning up...\n");

    // memory cleaning: close files, free memory, destroy codec instances
    freeStatsArray(&sa);
    if (enc_context.infile != NULL) {
        fclose(enc_context.infile);
    }
    enc_context.infile = NULL;
    glob_frame_counter = 0;
    free(enc_context.stats.buf);
    vpx_img_free(&enc_context.raw);

    // flush encoder -- CHECK writer param (before 32)
    EncodingResult res;
    do {
        res = encode_frame(&enc_context.codec, NULL, -1, 1, 0, VPX_DL_GOOD_QUALITY, NULL, 32, false, false);
    } while (res.got_pkts);

    if (vpx_codec_destroy(&enc_context.codec)) die_codec(&enc_context.codec, "Failed to destroy codec.");
}

int main(int argc, char **argv) {
  const char *const file_name = argv[1];
  const int width = (int)strtol(argv[2], NULL, 0);
  const int height = (int)strtol(argv[3], NULL, 0);
  exec_name = argv[0];
  initialize_encoder(file_name, width, height, 30, 200);
  // pass1(&enc_context.raw, enc_context.infile, "output.ivf", enc_context.encoder, &enc_context.cfg, 0);
  EncodingResult res;
  // loop through frames
  for (int i = 0; i < 100; i++) {
    printf("Encoding frame %d\n", i);
    res = encode_frame_external(32);
    printf("PSNR: %f\n", res.psnr);
    printf("Bitrate: %f\n", res.bitrate);
  }
  printf("global frame counter: %d\n", glob_frame_counter);
  return 0;
}


