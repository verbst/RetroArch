/*  RetroArch - A frontend for libretro.
 *
 *  RetroArch is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  RetroArch is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with RetroArch.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <boolean.h>
#include <libretro.h>
#include <string/stdstring.h>
#include <features/features_cpu.h>
#include <gfx/scaler/scaler.h>
#include <gfx/video_frame.h>
#include <audio/conversion/float_to_s16.h>
#include <streams/file_stream.h>
#include <file/file_path.h>
#include <retro_miscellaneous.h>
#include <retro_timers.h>
#ifdef HAVE_THREADS
#include <rthreads/rthreads.h>
#endif

#include "gfx_mister.h"
#include "video_crt_switch.h"
#include "video_defines.h"

#include "../configuration.h"
#include "../retroarch.h"
#include "../verbosity.h"
#include "../runloop.h"
#include "../paths.h"
#include "../file_path_special.h"

#ifdef HAVE_MENU
#include "../menu/menu_driver.h"
#endif

/* Pulls <winsock2.h> on Windows; keep it last and out of the header. */
#include <groovymister_wrapper.h>

/* The MiSTer's blit buffer is sized for 720x576x3. Exceeding it is a protocol
 * violation, not a soft limit, so every mode is clamped against it. */
#define MISTER_MAX_WIDTH   720
#define MISTER_MAX_HEIGHT  576
#define MISTER_MAX_BYTES   (MISTER_MAX_WIDTH * MISTER_MAX_HEIGHT * 3)

/* gmw_init rgbMode */
#define MISTER_RGB888      0
#define MISTER_RGB565      2

/* gmw_init lz4Frames value that selects the NLC near-lossless codec. */
#define MISTER_CODEC_NLC   7

/* mister_pacing: who owns the frame clock. */
#define MISTER_PACING_MISTER 0
#define MISTER_PACING_HOST   1

/* A failed connect costs a 60ms ACK timeout, so a dead address must not be
 * retried every frame. */
#define MISTER_RETRY_USEC  (2 * 1000000)

/* One keepalive per half the core's idle timeout, shortest 5s. The send waits
 * on the sender thread's 500ms idle wake, so the real interval is this plus up
 * to half a second. Sent whatever Allow Idle Timeout says. */
#define MISTER_KEEPALIVE_USEC (1500 * 1000)

/* Shortest gap between two "not blitting" / "blitting again" lines. Without
 * this a core running at half the modeline's rate logs an edge every frame. */
#define MISTER_STALL_LOG_USEC (2 * 1000000)

/* Frame periods with no frame at all before the log says so. The stall
 * counters only cover frames this driver was given; this covers the case they
 * cannot see, where it is not called at all. */
#define MISTER_NOFRAME_PERIODS 4

/* Audio staged between the audio path and the next blit. CmdAudio takes a
 * uint16 byte count, and one frame at 48kHz/60Hz is only 800 sample frames,
 * so this is generous; it is flushed early if a core ever runs ahead. */
/* One CmdAudio carries its length in 16 bits, so 16383 stereo frames is the
 * most a single packet can describe - which makes it the natural ceiling here
 * too. At 48 kHz that is about 340 ms of headroom for a late flush. It was half
 * this, and a hardware run threw away 9-15% of the audio by overrunning it. */
#define MISTER_AUDIO_MAX_FRAMES 16383

/* The most one CmdAudio may carry. The core stages audio between AUDIO_OFFSET
 * 0x32a000 and LZ4_OFFSET_A 0x332000 - 32768 bytes, or 8192 stereo frames -
 * and does not bound the copy into it. Anything longer overwrites the buffer
 * the next frame is decoded from. */
#define MISTER_AUDIO_PACKET_FRAMES 8192
#define MISTER_AUDIO_MAX_BYTES  (MISTER_AUDIO_MAX_FRAMES * 2 * (int)sizeof(int16_t))

/* Staged frames past which the sender is woken rather than left to its own
 * cadence. One frame's worth of sound, taken from the modeline, so a stalled
 * core cannot turn the stream into bursts the FPGA's FIFO has to ride out.
 * Floored so a nonsense modeline cannot drive it to zero. */
#define MISTER_AUDIO_WATERMARK_MIN 64
static size_t mister_audio_watermark = MISTER_AUDIO_PACKET_FRAMES;

/* Rate servo, see the state it drives further down. SERVO_FRAMES is how long
 * it takes to pay off a standing offset, in displayed frames - two seconds at
 * 60 Hz. SERVO_LIMIT holds the correction under nine cents. */
#define MISTER_AUDIO_SERVO_FRAMES 120.0
#define MISTER_AUDIO_SERVO_LIMIT  0.005

/* Reasons a frame did not reach the wire. Logged on transition rather than per
 * frame: a steady state costs one line, and "we stopped blitting, and why" is
 * the single thing the first two hardware runs could not answer. */
enum mister_stall
{
   MISTER_STALL_NONE = 0,
   MISTER_STALL_DISCONNECTED,
   MISTER_STALL_NO_MODELINE,
   MISTER_STALL_NO_VIEWPORT_PIN,
   MISTER_STALL_NO_READ_VIEWPORT,
   MISTER_STALL_READBACK_FAILED,
   MISTER_STALL_NO_MEMORY,
   MISTER_STALL_NO_FRAME,
   MISTER_STALL_PIXEL_FORMAT,
   MISTER_STALL_SCALE_FAILED,
   MISTER_STALL_NO_BLIT_BUFFER,
   MISTER_STALL_SENDER_BUSY,
   MISTER_STALL_LAST
};

typedef struct
{
   retro_time_t last_attempt;
   retro_time_t last_keepalive;
   retro_time_t last_telemetry;
   uint32_t frame;
   unsigned  width;          /* modeline active area, pixels             */
   unsigned  height;         /* modeline active area, lines (full frame) */
   unsigned  fb_height;      /* lines per blit; half of height when interlaced */
   unsigned  refused_width;  /* last modeline refused as too large, 0 = none */
   unsigned  refused_height;
   unsigned  rejected_width; /* last modeline ignored as degenerate     */
   unsigned  rejected_height;
   uint8_t   field;
   uint8_t   interlaced;     /* 0 progressive, 1 interlaced framebuffer  */
   uint8_t   rgb_mode;
   uint8_t   codec;
   bool      connected;
   bool      modeline_active;
   bool      mode_pending;
   bool      must_clear;

   bool      warned_hw;
   bool      hw_pinned;   /* viewport pinned for GPU readback */
   const void *pinned_data; /* video driver instance the pin was applied to */
   uint8_t   input_caps;  /* what CmdInit actually negotiated */
   bool      inputs_bound;/* gmw_bindInputs succeeded; v1 cores report no caps */
   bool      saw_analog;  /* an analog packet has arrived at least once */
   unsigned  blits_since_mode;
   uint32_t  echo_at_mode;  /* frameEcho when this modeline was programmed */
   bool      warned_no_echo;
   /* Where a frame's time goes on the streaming path, accumulated in
    * microseconds since the last telemetry line. Only sampled when pace
    * logging is on, so a normal run pays nothing for it. */
   uint64_t  t_readback;
   uint64_t  t_map;       /* of t_readback, time blocked on the GPU */
   uint64_t  t_convert;   /* of t_readback, time converting pixels  */
   uint64_t  t_stage;
   uint64_t  t_blit;
   unsigned  t_samples;
   uint32_t  last_echo;     /* frameEcho at the last blit that advanced it */
   retro_time_t echo_stall_at;   /* when frameEcho stopped moving, 0 if moving */
   bool      warned_echo_stall;  /* the stall notice is said once per stall */


   bool      disabled;   /* unrecoverable: no-op for the rest of the run */
   unsigned  attempts;   /* consecutive failed connects                  */
   bool      trace_blit;  /* log the pixel path on the next blit */
   uint8_t   warned_sync; /* vramSynced/frameskip notices, once each */
   bool      in_readback; /* re-entrancy guard, see mister_draw           */
   uint8_t   stall;       /* enum mister_stall, current reason for not blitting */
   retro_time_t last_stall_log; /* rate limit for the transition lines */
   /* Written by the frame loop, read by the sender's idle path - the only
    * thing still running when the frame loop produces nothing. Unsynchronised:
    * a stale read costs a log line a tick late. */
   retro_time_t last_draw_usec; /* when this driver was last handed a frame */
   bool      draw_gap_open;     /* a no-frame stretch has been reported      */
   /* Previous frame's shape, so the letterbox border can be cleared whenever
    * anything about what is staged changes. */
   unsigned  last_stage_w;
   unsigned  last_stage_h;
   uint8_t   last_field;
   uint8_t   field_seeded;     /* phase taken from the FPGA at least once   */
   uint8_t   last_rotation;
   bool      last_hw_frame;
   /* Session counters, reported periodically and at close. */
   uint64_t  stat_frames;         /* mister_draw calls                     */
   uint64_t  stat_blits;
   uint64_t  stat_dropped[MISTER_STALL_LAST];
   uint64_t  stat_audio_frames;

   /* Kept apart because they need different fixes: overrun is the staging
    * buffer filling faster than the sender empties it, nobuf is the client
    * having no audio buffer to hand us. */
   uint64_t  stat_audio_drop_overrun;
   uint64_t  stat_audio_drop_nobuf;
} mister_state_t;

static mister_state_t     mister_st;
static mister_modeline_t  mister_mode;
static gmw_fpgaStatus     mister_status;

/* Frame pacing bookkeeping. mister_frame_period_usec comes from the applied
 * modeline; mister_frame_end_usec is when the previous frame's wait finished,
 * so the two together say how much of this frame is left. */
static retro_time_t       mister_frame_period_usec;
static retro_time_t       mister_frame_end_usec;
/* Frame-loop side of the pacing, for iterations that send nothing. Separate
 * from mister_frame_end_usec above, which the sender thread owns. */
static retro_time_t       mister_skip_pace_usec;

/* ---------------------------------------------------------------------------
 * The sender thread.
 *
 * Encoding a frame and putting it on the wire costs about as much again as
 * reading it back, and in RetroArch both used to land on the thread running the
 * emulator - there is no equivalent of the render/emulation split a standalone
 * emulator has, so every microsecond spent here is one the core does not get.
 * On a light core that is affordable. On a heavy one it is the difference
 * between full speed and not.
 *
 * So the frame loop now reads the frame back and hands it over, and this thread
 * does the rest. The rule that keeps it safe is that once the thread is
 * running it is the ONLY caller of the client for video and audio - the client
 * is not thread-safe, and one owner is easier to keep true than a lock around
 * every call. Anything structural (opening a session, closing one, changing
 * modeline) waits for the thread to go idle first, which costs nothing because
 * those happen once in a while rather than once a frame.
 *
 * The queue is one frame deep on purpose. There is no backlog to pipeline
 * through, so the frame is on the wire as soon as it is packed and the only
 * latency added is a thread wake. Under MiSTer pacing the handoff blocks until
 * the thread is free, which is what keeps the raster the clock: the wait moves
 * from the frame loop into the queue, but it is the same wait. */
typedef struct
{
   uint8_t *stage;          /* staging buffer holding this frame's pixels */
   unsigned stage_w;
   unsigned stage_h;
   bool     flip_v;         /* readback rows arrive bottom-up */
   bool     trace;          /* emit the one-shot pixel path trace */
   /* Only for that trace line, so it can still describe where the frame came
    * from once the frame loop has moved on. */
   unsigned src_w;
   unsigned src_h;
   unsigned src_pitch;
   int      src_fmt;
   bool     src_menu;
   bool     src_hw;
   uint16_t vsync_line;     /* raster line the FPGA commits this frame at */
} mister_job_t;

static sthread_t *mister_sender;
static slock_t   *mister_send_lock;
static scond_t   *mister_send_ready;   /* a job was queued, or we are quitting */
static scond_t   *mister_send_idle;    /* the sender finished a job */
static slock_t   *mister_audio_lock;   /* audio staging crosses the two threads */
static mister_job_t mister_job;
static bool       mister_send_quit;

/* The queue slot and the sender's state are separate on purpose.
 *
 * They were one flag to begin with, held from the moment a frame was queued
 * until the sender had finished everything including its raster wait - so the
 * frame loop waited for the whole cycle and got no overlap at all, which was
 * the entire point of having a thread. The slot now frees the instant the
 * sender takes the frame, so emulation for the next frame runs alongside the
 * encode and the send of this one. */
static int        mister_job_queued   = -1;  /* staging index waiting, -1 if none */
static int        mister_job_inflight = -1;  /* staging index being read, -1 if none */
static bool       mister_send_working;       /* sender is mid-job */

/* Three staging buffers, not two.
 *
 * At any moment one can be in flight, one queued behind it, and one being
 * written by the frame loop. With only two, the loop starts overwriting the
 * buffer the sender is still reading as soon as the queue slot frees - which is
 * a real race, not a theoretical one, and it is the reason this is a ring
 * rather than a swap. mister_stage aliases whichever one is being written. */
#define MISTER_STAGE_BUFS 3
static uint8_t *mister_stage_buf[MISTER_STAGE_BUFS];
static size_t   mister_stage_bufsize[MISTER_STAGE_BUFS];
static int      mister_stage_write;

/* How long the sender waits for work before looking at the keepalive. Well
 * inside the core's idle timeout, and it is what keeps the session alive while
 * a modal window has the frame loop stopped - this thread is not on the
 * message pump, so nothing Windows does can hold it up. */
#define MISTER_SENDER_IDLE_USEC (500 * 1000)

/* ------------------------------------------------------------------------
 * Diagnostics
 *
 * RetroArch's own log is not usable here. RARCH_LOG, RARCH_WARN and RARCH_ERR
 * all return early unless log_verbosity is on (verbosity.c), that output only
 * reaches disk when log_to_file is on as well, and a Windows GUI build has no
 * console to fall back to. Three global settings, none of them in this menu,
 * and any one of them silently discards everything written here - which is
 * exactly what happened on the first two hardware runs.
 *
 * So this writes its own file, driven by the Log Verbosity setting on the
 * Groovy MiSTer page and nothing else, and mirrors to RARCH_LOG for anyone
 * who does have RetroArch logging turned on.
 * ------------------------------------------------------------------------ */

#define MISTER_LOG_MAX_BYTES (8 * 1024 * 1024)

/* Wall-clock spacing for the periodic counters at log level 1. */
#define MISTER_TELEMETRY_USEC (2 * 1000000)

enum mister_log_sev
{
   MISTER_SEV_ERR = 0,
   MISTER_SEV_WARN,
   MISTER_SEV_INFO,   /* setup and refusals; always written */
   MISTER_SEV_PACE,   /* level 1 and above                  */
   MISTER_SEV_TRACE   /* level 2                            */
};

static RFILE  *mister_logfile;
static char    mister_logpath[PATH_MAX_LENGTH];
static size_t  mister_logbytes;
static bool    mister_log_tried;
/* Process-scope: a run opens the log once and every later session appends to
 * it. mister_close() closes the file, so without this each reconnect truncated
 * the previous session away - a six-session run left one session on disk. */
static bool    mister_log_started;
static retro_time_t mister_log_epoch_usec;
#ifdef HAVE_THREADS
/* gmw_set_log_callback documents that the sink may be called from any
 * thread, and the client does call it from its receive path. */
static slock_t *mister_loglock;
#endif

static void mister_log_close(void)
{
#ifdef HAVE_THREADS
   /* The lock itself is never freed - see mister_logf. Take it so a client
    * thread part-way through a line finishes before the file goes. */
   if (mister_loglock)
      slock_lock(mister_loglock);
#endif
   if (mister_logfile)
   {
      filestream_close(mister_logfile);
      mister_logfile = NULL;
   }
   mister_logbytes  = 0;
   mister_log_tried = false;
#ifdef HAVE_THREADS
   if (mister_loglock)
      slock_unlock(mister_loglock);
#endif
}

/* log_dir when it is usable, otherwise a "logs" directory beside the config
 * RetroArch actually loaded. The configured value is not always usable - the
 * run that prompted all this had log_dir = ":\logs", which has no drive
 * letter and cannot be created. */
static bool mister_log_dir(char *s, size_t len)
{
   settings_t *settings = config_get_ptr();
   const char *log_dir  = settings ? settings->paths.log_dir : NULL;
   const char *cfg;

   if (!string_is_empty(log_dir) && !string_is_equal(log_dir, "default"))
   {
      strlcpy(s, log_dir, len);
      if (path_is_directory(s) || path_mkdir(s))
         return true;
   }

   if (!string_is_empty((cfg = path_get(RARCH_PATH_CONFIG))))
      fill_pathname_basedir(s, cfg, len);
   else if (!fill_pathname_application_data(s, len))
      return false;

   fill_pathname_join_special(s, s, "logs", len);
   return path_is_directory(s) || path_mkdir(s);
}

static void mister_log_open(void)
{
   char dir[DIR_MAX_LENGTH];

   if (mister_logfile || mister_log_tried)
      return;
   mister_log_tried = true;

   if (!mister_log_dir(dir, sizeof(dir)))
   {
      RARCH_ERR("[MiSTer] No writable log directory; MiSTer diagnostics will "
                "not be saved.\n");
      return;
   }

   fill_pathname_join_special(mister_logpath, dir, "mister.log",
         sizeof(mister_logpath));

   if (!(mister_logfile = filestream_open(mister_logpath,
               mister_log_started
                  ? (RETRO_VFS_FILE_ACCESS_WRITE
                     | RETRO_VFS_FILE_ACCESS_UPDATE_EXISTING)
                  : RETRO_VFS_FILE_ACCESS_WRITE,
               RETRO_VFS_FILE_ACCESS_HINT_NONE)))
   {
      RARCH_ERR("[MiSTer] Could not open \"%s\" for writing.\n",
            mister_logpath);
      return;
   }

   if (mister_log_started)
      mister_logbytes = (size_t)filestream_seek(mister_logfile, 0,
            RETRO_VFS_SEEK_POSITION_END);
   else
   {
      mister_logbytes = 0;
      RARCH_LOG("[MiSTer] Diagnostics are being written to \"%s\".\n",
            mister_logpath);
   }
   if (!mister_log_epoch_usec)
      mister_log_epoch_usec = cpu_features_get_time_usec();
   mister_log_started = true;
}

/* One rotation, so a level-2 session left running overnight cannot fill the
 * disk while still leaving the most recent history in place. */
static void mister_log_rotate(void)
{
   char old_path[PATH_MAX_LENGTH];

   filestream_close(mister_logfile);
   mister_logfile = NULL;

   strlcpy(old_path, mister_logpath, sizeof(old_path));
   strlcat(old_path, ".1", sizeof(old_path));
   if (path_is_valid(old_path))
      filestream_delete(old_path);
   filestream_rename(mister_logpath, old_path);

   mister_logfile  = filestream_open(mister_logpath,
         RETRO_VFS_FILE_ACCESS_WRITE, RETRO_VFS_FILE_ACCESS_HINT_NONE);
   mister_logbytes = 0;

   /* If the reopen failed, let mister_log_open try again on the next line
    * rather than leaving the sink silently dead for the rest of the run. */
   if (!mister_logfile)
      mister_log_tried = false;
}

/* Whether pace-level output is being recorded, so the timing probes on the
 * frame path can be skipped entirely when it is not. */
static bool mister_pace_logging(void)
{
   settings_t *settings = config_get_ptr();
   return settings && settings->uints.mister_log_level >= 1;
}

static void mister_logf(enum mister_log_sev sev, const char *fmt, ...)
{
   char    line[512];
   va_list ap;
   int     n;
   unsigned level;

   {
      settings_t *settings = config_get_ptr();
      level = settings ? settings->uints.mister_log_level : 0;
   }

   /* INFO and below are the setup and refusal messages: they are what makes
    * a failed session diagnosable at all, so the level never hides them. */
   if (sev == MISTER_SEV_PACE  && level < 1)
      return;
   if (sev == MISTER_SEV_TRACE && level < 2)
      return;

   va_start(ap, fmt);
   n = vsnprintf(line, sizeof(line), fmt, ap);
   va_end(ap);
   if (n < 0)
      return;

   switch (sev)
   {
      case MISTER_SEV_ERR:
         RARCH_ERR("[MiSTer] %s", line);
         break;
      case MISTER_SEV_WARN:
         RARCH_WARN("[MiSTer] %s", line);
         break;
      default:
         RARCH_LOG("[MiSTer] %s", line);
         break;
   }

#ifdef HAVE_THREADS
   /* Created once and never destroyed. The client calls this sink from its own
    * thread, so a lock freed on close is one that thread can still be waiting
    * on. One small allocation for the life of the process is the cheaper
    * trade. Creating it here is safe because the first line of every session
    * is written from the main thread during mister_init, before the client
    * has a thread of its own. */
   if (!mister_loglock)
      mister_loglock = slock_new();
   if (mister_loglock)
      slock_lock(mister_loglock);
#endif

   /* Under the lock: it opens the file and moves the byte counter, and the
    * client thread can be in here at the same time. */
   mister_log_open();

   if (mister_logfile)
   {
      static const char *tag[] = { "ERR ", "WARN", "INFO", "PACE", "TRC " };
      /* Milliseconds since the process started, so this can be lined up
       * against the MiSTer's own log, which timestamps the same way. */
      unsigned ms = (unsigned)((cpu_features_get_time_usec()
               - mister_log_epoch_usec) / 1000);
      int written = filestream_printf(mister_logfile, "[%8u.%03u] [%s] %s",
            ms / 1000, ms % 1000, tag[sev], line);

      if (written > 0)
         mister_logbytes += (size_t)written;
      filestream_flush(mister_logfile);

      if (mister_logbytes >= MISTER_LOG_MAX_BYTES)
         mister_log_rotate();
   }
#ifdef HAVE_THREADS
   if (mister_loglock)
      slock_unlock(mister_loglock);
#endif
}

#define MISTER_ERR(...)   mister_logf(MISTER_SEV_ERR,   __VA_ARGS__)
#define MISTER_WARN(...)  mister_logf(MISTER_SEV_WARN,  __VA_ARGS__)
#define MISTER_INFO(...)  mister_logf(MISTER_SEV_INFO,  __VA_ARGS__)
#define MISTER_PACE(...)  mister_logf(MISTER_SEV_PACE,  __VA_ARGS__)
#define MISTER_TRACE(...) mister_logf(MISTER_SEV_TRACE, __VA_ARGS__)

static const char *mister_stall_str(uint8_t stall)
{
   switch (stall)
   {
      case MISTER_STALL_DISCONNECTED:
         return "no session";
      case MISTER_STALL_NO_MODELINE:
         return "no modeline has been accepted";
      case MISTER_STALL_NO_VIEWPORT_PIN:
         return "the video driver cannot pin its viewport";
      case MISTER_STALL_NO_READ_VIEWPORT:
         return "the video driver cannot read its viewport back";
      case MISTER_STALL_READBACK_FAILED:
         return "reading the frame back off the GPU failed";
      case MISTER_STALL_NO_MEMORY:
         return "the staging buffer could not be allocated";
      case MISTER_STALL_NO_FRAME:
         return "the core produced no frame (duplicate or paused)";
      case MISTER_STALL_PIXEL_FORMAT:
         return "the core's pixel format is not supported";
      case MISTER_STALL_SCALE_FAILED:
         return "scaling the frame failed";
      case MISTER_STALL_NO_BLIT_BUFFER:
         return "the client returned no blit buffer";
      case MISTER_STALL_SENDER_BUSY:
         return "the previous frame was still being sent";
      default:
         break;
   }
   return "unknown";
}

/* Edge-triggered: report the first frame that stalls and the first that
 * recovers, and count the rest. Per-frame logging of a steady state is what
 * makes a log useless. */
static void mister_stall_set(uint8_t stall)
{
   if (stall < MISTER_STALL_LAST)
      mister_st.stat_dropped[stall]++;

   if (mister_st.stall == stall)
      return;

   /* Edge-triggered, but a core that alternates a real frame with a duplicate
    * - anything running at half the modeline's rate - produces an edge every
    * single frame, and each one is a line written and flushed to disk. Space
    * them out: the first transition is the interesting one, and the session
    * summary carries the totals regardless. */
   {
      retro_time_t now = cpu_features_get_time_usec();

      if (      !mister_st.last_stall_log
            || (now - mister_st.last_stall_log) >= MISTER_STALL_LOG_USEC)
      {
         mister_st.last_stall_log = now;

         if (stall == MISTER_STALL_NONE)
            MISTER_INFO("Blitting again after %s.\n",
                  mister_stall_str(mister_st.stall));
         else
            MISTER_INFO("Not blitting: %s.\n", mister_stall_str(stall));
      }
   }

   mister_st.stall = stall;
}

/* Scaled, BGR24, one image per frame. Sized on demand rather than to a fixed
 * maximum, because the scaled source can be larger than the modeline before
 * it is centred and clipped. */
static struct scaler_ctx *mister_scaler;
static uint8_t *mister_stage;
static size_t   mister_stage_size;

static int16_t *mister_audio_buf;      /* staged s16 stereo, interleaved */
static size_t   mister_audio_frames;   /* sample frames currently staged   */
static int16_t *mister_audio_scratch;  /* converted input, only when resampling */

/* Rate servo.
 *
 * What reaches mister_audio_push() has been through RetroArch's resampler,
 * which is servoed to the local sound card. The MiSTer plays out on its own
 * clock and reports no buffer level, so a difference between the two is
 * uncorrected and accumulates. Measured at 0.23% on hardware, against a core
 * FIFO that is prefilled once with at most 2100 samples and never topped up.
 *
 * The one clock both sides can see is the core's displayed-frame counter. It
 * consumes mister_audio_spf sample frames per frame displayed, so the running
 * difference between that and what we have sent is measurable with no host
 * clock at all, and a small correction to the resampling step holds it. */
static double   mister_audio_spf;        /* frames consumed per displayed frame */
static double   mister_audio_ratio = 1.0;/* output frames per input frame       */
static double   mister_audio_phase;      /* fractional read position, carried   */
static int16_t  mister_audio_hold[2];    /* last input frame, for interpolation */
static bool     mister_audio_hold_valid; /* false until the first resampled run */
static uint64_t mister_audio_sent;       /* frames the client actually sent     */
static uint32_t mister_audio_ref_frame;  /* displayed-frame count when armed    */
static uint32_t mister_audio_seen_frame; /* displayed-frame count last measured */
static double   mister_audio_drift;      /* last measurement, for the log       */
static bool     mister_audio_armed;

static void    *mister_menu_frame;
static unsigned mister_menu_width;
static unsigned mister_menu_height;
static bool     mister_menu_active;
static bool     mister_menu_prev;

/* Each video driver keeps its own private state struct, so the viewport pin
 * lives next to that state and is exported rather than reached into here. */
#ifdef HAVE_OPENGL
void gl2_mister_set_viewport(void *data, unsigned width, unsigned height);
#endif
#ifdef HAVE_OPENGL_CORE
void gl3_mister_set_viewport(void *data, unsigned width, unsigned height);
#endif
#ifdef HAVE_VULKAN
void vulkan_mister_set_viewport(void *data, unsigned width, unsigned height);
#endif
#ifdef HAVE_D3D11
void d3d11_mister_set_viewport(void *data, unsigned width, unsigned height);
#endif
#ifdef HAVE_D3D12
void d3d12_mister_set_viewport(void *data, unsigned width, unsigned height);
#endif

/* Pin the active driver's viewport to the modeline. Returns false when the
 * driver has no MiSTer-aware pin, in which case hardware-rendered cores are
 * left alone rather than read back at window size every frame. */
static bool mister_pin_viewport(video_driver_state_t *video_st,
      unsigned width, unsigned height)
{
   const char *ident = video_driver_get_ident();

   if (!ident || !video_st->data)
      return false;

#ifdef HAVE_OPENGL
   if (string_is_equal(ident, "gl"))
   {
      gl2_mister_set_viewport(video_st->data, width, height);
      return true;
   }
#endif
#ifdef HAVE_OPENGL_CORE
   if (string_is_equal(ident, "glcore"))
   {
      gl3_mister_set_viewport(video_st->data, width, height);
      return true;
   }
#endif
#ifdef HAVE_VULKAN
   if (string_is_equal(ident, "vulkan"))
   {
      vulkan_mister_set_viewport(video_st->data, width, height);
      return true;
   }
#endif
#ifdef HAVE_D3D11
   if (string_is_equal(ident, "d3d11"))
   {
      d3d11_mister_set_viewport(video_st->data, width, height);
      return true;
   }
#endif
#ifdef HAVE_D3D12
   if (string_is_equal(ident, "d3d12"))
   {
      d3d12_mister_set_viewport(video_st->data, width, height);
      return true;
   }
#endif
   return false;
}

/* The core accepts exactly three rates. gmw_init takes the rate in Hz and maps
 * it internally; anything else maps to "off", so an unsupported rate has to be
 * caught here rather than silently losing audio. */
static uint32_t mister_audio_rate(unsigned rate)
{
   if (rate == 22050 || rate == 44100 || rate == 48000)
      return rate;
   return 0;
}

static void mister_audio_flush(void);
static void mister_keepalive(void);

/* The sender thread, defined further down next to the work it does. */
static bool mister_sender_start(void);
static void mister_sender_stop(void);
static void mister_sender_idle(void);
static void mister_telemetry(void);

static void mister_log(const char *msg)
{
   /* The client's own messages are already tagged and newline-terminated, and
    * the level was applied inside the client when the sink was installed, so
    * anything arriving here is meant to be recorded. */
   mister_logf(MISTER_SEV_INFO, "%s", msg);
}

void mister_log_note(const char *fmt, ...)
{
   char    line[512];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(line, sizeof(line), fmt, ap);
   va_end(ap);

   mister_logf(MISTER_SEV_INFO, "%s", line);
}

bool mister_is_connected(void)
{
   return mister_st.connected;
}

/* The effective frame clock, which is not always the configured one.
 *
 * On Windows the client's send completion queue is drained in exactly one
 * place - drainSendCompletions(), called only from WaitSync (see
 * deps/mister/groovymister.cpp, whose own comment says "Called every frame
 * from WaitSync"). Host pacing skips WaitSync, so nothing dequeues the
 * BUFFER_SLICES-deep queue; a 480p frame posts around 630 sends, so it fills
 * within a couple of frames and RIOSend starts failing silently. The receive
 * re-post shares that queue, so ACKs stop too and it presents as a dead core
 * with false "no ACK" reconnects.
 *
 * POSIX has no RIO and no send queue - drainSendCompletions is compiled out -
 * so host pacing is honoured there. */
/* The Windows override that used to live here is gone. It forced the MiSTer
 * raster because host pacing stalled the stream within a few frames, but the
 * cause was mister_sync() skipping gmw_waitSync() in that mode and so never
 * draining the client's send completion queue. gmw_waitSync() is now called in
 * both modes, so the setting means what it says on every platform. */
static unsigned mister_pacing_mode(void)
{
   return config_get_ptr()->uints.mister_pacing;
}

/* Record why this frame is not being sent, and - under MiSTer pacing - hold
 * the frame loop to the modeline anyway.
 *
 * Under MiSTer pacing the raster is the clock, and the only thing that applies
 * it to the frame loop is mister_submit() blocking on the sender's slot. Every
 * path that gives up on a frame returns before reaching it, so on content that
 * duplicates frames there was nothing holding the loop at all: it free-ran,
 * retro_run() was driven far faster than the modeline, and the core answered
 * most calls with another duplicate. One session measured 13,562 duplicates in
 * 19,719 frames - 69%, about 180 fps against a 56.842 Hz mode - with audio
 * suffering for it, while the same content under host pacing, where
 * RetroArch's own limiter still applies, sat at 20% and sounded fine.
 *
 * Waiting out the rest of the period here costs nothing: there is no frame
 * waiting to go, so there is nothing to delay. What it does is put the next
 * retro_run() back on the modeline's cadence, which is where the duplicates
 * were coming from. */
static void mister_skip_frame(uint8_t stall)
{
   retro_time_t now;

   mister_stall_set(stall);

   if (     !mister_st.connected
         ||  mister_pacing_mode() != MISTER_PACING_MISTER
         ||  mister_frame_period_usec <= 0)
      return;

   now = cpu_features_get_time_usec();

   if (mister_skip_pace_usec)
   {
      retro_time_t left = (mister_skip_pace_usec + mister_frame_period_usec) - now;

      /* Bounded by the period, so a timestamp left stale by a pause, a mode
       * change or a run of sent frames cannot turn into a long sleep - it
       * simply costs one unpaced iteration before the stamp below is fresh. */
      if (left > 0 && left < mister_frame_period_usec)
      {
         retro_sleep_us((unsigned)left);
         now = cpu_features_get_time_usec();
      }
   }

   mister_skip_pace_usec = now;
}

bool mister_pacing_active(void)
{
   /* Host pacing keeps RetroArch's own limiter in charge, so report the
    * MiSTer as not owning the clock even though the session is live. Must
    * agree with mister_sync(): if the two disagree the runloop either stands
    * its limiter down while nothing is pacing, or runs two limiters at once. */
   if (mister_pacing_mode() != MISTER_PACING_MISTER)
      return false;

   return mister_st.connected && mister_st.modeline_active;
}

void mister_set_menu_buffer(void *frame, unsigned width, unsigned height)
{
   mister_menu_frame  = frame;
   mister_menu_width  = width;
   mister_menu_height = height;
}

void mister_set_mode(const mister_modeline_t *mode)
{
   if (!mode)
      return;

   /* switchres briefly resolves degenerate modes while a core is starting;
    * sending one costs a full re-sync on the CRT for nothing. A zeroed mode
    * also arrives here when switchres failed to resolve anything at all, and
    * that is not transient - so say so, once per distinct size, rather than
    * dropping it in silence the way this did for four hardware runs. */
   if (mode->width < 200 || mode->height < 160)
   {
      if (      mister_st.rejected_width  != mode->width
            || mister_st.rejected_height != mode->height)
      {
         mister_st.rejected_width  = mode->width;
         mister_st.rejected_height = mode->height;
         MISTER_WARN("Ignoring %ux%u modeline: too small to be real. A 0x0 "
                     "means switchres resolved nothing - see the [switchres] "
                     "lines above for why.\n",
               mode->width, mode->height);
      }
      return;
   }

   mister_st.rejected_width  = 0;
   mister_st.rejected_height = 0;

   memcpy(&mister_mode, mode, sizeof(mister_mode));
   mister_st.mode_pending = true;
}

/* Grow the buffer the frame loop is currently writing.
 *
 * mister_stage is an alias into the staging ring, so a realloc that moves it
 * has to be written back into the array as well - otherwise the handoff picks
 * up the pointer this call has already freed. Only ever touches the write
 * slot, which by construction is neither queued nor in flight, so the sender
 * cannot be reading it. */
static bool mister_stage_reserve(size_t bytes)
{
   uint8_t *buf;

   if (mister_stage && mister_stage_size >= bytes)
      return true;

   buf = (uint8_t*)realloc(mister_stage, bytes);
   if (!buf)
      return false;

   mister_stage      = buf;
   mister_stage_size = bytes;

   mister_stage_buf[mister_stage_write]     = buf;
   mister_stage_bufsize[mister_stage_write] = bytes;
   return true;
}

/* Every failed attempt blocks the video thread inside the client for two 60 ms
 * ACK timeouts, so retrying at the initial cadence forever would cost a visible
 * hitch several times a minute against an address that is simply not there.
 *
 * But "no answer" is recoverable - the usual case is that the MiSTer has not
 * been switched on yet, or the Groovy core has not been started - so it must
 * not disable the module for the run. After this many attempts the backoff
 * stops doubling and settles at MISTER_SLOW_RETRY_USEC, which costs one 120 ms
 * hitch a minute and keeps working whenever the MiSTer does appear. */
#define MISTER_MAX_ATTEMPTS 5
#define MISTER_SLOW_RETRY_USEC (60 * 1000000)

/* How long frameEcho may stand still before it is worth mentioning. Long
 * enough not to fire on a core that hitches for a moment. */
#define MISTER_ECHO_STALL_USEC (3 * 1000000)

/* How much of the frame to leave for gmw_waitSync() to land on the raster.
 * It gets there by spinning, so this is time a core does not get; the point of
 * sleeping the rest away is to make it short. Wide enough to absorb ordinary
 * sleep overshoot - retro_sleep_us() uses a high resolution waitable timer on
 * Windows, so overshoot is tens of microseconds, not milliseconds. */
#define MISTER_SPIN_MARGIN_USEC 1500

/* Stop the whole module for the rest of the run, loudly and once. Used for
 * anything that cannot come right without the user changing a setting, so a
 * misconfiguration degrades to "MiSTer output is off" instead of taking
 * RetroArch with it. */
void mister_retry(void)
{
   if (mister_st.disabled)
      MISTER_INFO("Re-enabled by the user; will try to connect again.\n");

   mister_st.disabled     = false;
   mister_st.attempts     = 0;
   mister_st.last_attempt = 0;
}

static void mister_disable(const char *why, const char *osd)
{
   if (mister_st.disabled)
      return;

   mister_st.disabled = true;
   MISTER_ERR("MiSTer output disabled for this run: %s\n", why);

   if (osd)
      runloop_msg_queue_push(osd, strlen(osd), 2, 300, true, NULL,
            MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_ERROR);
}

/* The client resolves the address with a bare inet_addr() and never checks the
 * result, so a hostname, an empty string or a typo silently becomes
 * 255.255.255.255 and the failure is indistinguishable from an absent MiSTer.
 * Only dotted quads are usable, so anything else is refused here where it can
 * still be explained. */
static bool mister_ip_is_valid(const char *ip)
{
   unsigned octets = 0;
   const char *p   = ip;

   if (string_is_empty(ip))
      return false;

   while (*p && octets < 4)
   {
      unsigned value  = 0;
      unsigned digits = 0;

      while (*p >= '0' && *p <= '9')
      {
         value = (value * 10) + (unsigned)(*p - '0');
         if (++digits > 3 || value > 255)
            return false;
         p++;
      }
      if (!digits)
         return false;
      octets++;

      if (*p == '.')
         p++;
      else
         break;
   }

   return (octets == 4) && (*p == '\0');
}

static void mister_init(settings_t *settings, enum retro_pixel_format pix_fmt)
{
   unsigned codec    = settings->uints.mister_codec;
   unsigned rgb_mode = settings->uints.mister_rgb_mode;
   uint32_t rate     = mister_audio_rate(settings->uints.audio_output_sample_rate);
   retro_time_t now  = cpu_features_get_time_usec();
   retro_time_t wait;

   /* Back off rather than hammering: 2s, 4s, 8s ... so a machine that is off
    * costs progressively less, and the attempt count still reaches the limit
    * in well under a minute. */
   /* 2/4/8/16/32 s while the MiSTer might just be booting, then a quiet
    * once-a-minute poll for the rest of the run. */
   if (mister_st.attempts >= MISTER_MAX_ATTEMPTS)
      wait = MISTER_SLOW_RETRY_USEC;
   else
      wait = MISTER_RETRY_USEC << (mister_st.attempts < 4 ? mister_st.attempts : 4);
   if (      mister_st.last_attempt
         && (now - mister_st.last_attempt) < wait)
      return;
   mister_st.last_attempt = now;

   if (!mister_ip_is_valid(settings->arrays.mister_ip))
   {
      /* Sized to hold the whole message plus the longest address the setting
       * can carry: truncating the one diagnostic that says what to do next
       * would defeat the point of it. */
      char why[160 + sizeof(settings->arrays.mister_ip)];
      snprintf(why, sizeof(why),
            "\"%s\" is not an IPv4 address. Set MiSTer Address to the "
            "numeric address of the MiSTer, for example 192.168.0.2; "
            "host names are not supported.",
            settings->arrays.mister_ip);
      mister_disable(why, "MiSTer: address is not a valid IP - see mister.log");
      return;
   }

   /* Both of these ride CMD_INIT and tell the core how to read the stream, so a
    * value the client cannot honour is not a cosmetic problem: the core keeps
    * decoding to the declared format while we send something else. A codec the
    * client has no case for leaves cSize 0 and sends raw under a compressed
    * header; an RGB mode we do not share a bpp for makes the client size the
    * stream from a different pixel width than we staged. Nothing between here
    * and the wire range-checks either, and a config file is not obliged to
    * hold a value the menu could produce. */
   if (codec > MISTER_CODEC_NLC)
   {
      MISTER_WARN("Compression %u is not a codec this client implements; "
                  "using NLC (%d).\n", codec, MISTER_CODEC_NLC);
      codec = MISTER_CODEC_NLC;
   }

   if (rgb_mode != MISTER_RGB888 && rgb_mode != MISTER_RGB565)
   {
      MISTER_WARN("Colour depth %u is not supported; using RGB888. Only "
                  "RGB888 and RGB565 share a pixel size with this client.\n",
            rgb_mode);
      rgb_mode = MISTER_RGB888;
   }

   /* NLC has no RGB565 encoder and never will: nlc_encode() rejects the format,
    * CmdBlit sends a raw-size blit header while the core is still in compressed
    * mode, and the core then ends the frame on the first payload packet and
    * writes the remainder into its audio DDR zone. Not a bandwidth problem -
    * memory corruption on the FPGA.
    *
    * CmdInit refuses this pair outright now, so this is no longer the only line
    * of defence; it is the friendlier one. Coercing here rather than letting the
    * connect fail keeps the session working, and it has to happen here anyway,
    * where the staging format is chosen - coercing nearer the wire would leave
    * the buffer full of 565 pixels being read as 888.
    *
    * MUST run after the codec clamp above. Reversed, a hand-edited codec of 9
    * with RGB565 would pass this test (9 is not 7), then be clamped to 7 - and
    * arrive at gmw_init as exactly the pair this exists to prevent. */
   if (rgb_mode == MISTER_RGB565 && codec == MISTER_CODEC_NLC)
   {
      MISTER_WARN("RGB565 is not supported by the NLC codec; using RGB888. "
                 "Select a LZ4 compression mode if RGB565 is needed.\n");
      rgb_mode = MISTER_RGB888;
   }

   /* A core that reports RGB565 cannot fill an RGB888 stream with more colour
    * than it has, but the wire cost is what matters, so honour the setting
    * either way and simply convert. */
   if (pix_fmt == RETRO_PIXEL_FORMAT_RGB565 && rgb_mode != MISTER_RGB565)
      rgb_mode = MISTER_RGB888;

   /* Installed before gmw_init so the whole handshake, including a failure,
    * is captured; the client documents this ordering requirement. */
   gmw_set_log_callback(mister_log, (int)settings->uints.mister_log_level);
   mister_log_open();

   /* The switchres state is deliberately NOT logged here. mister_init runs from
    * mister_draw, which video_driver_frame calls before crt_switch_res_core,
    * so at this point switchres has not been asked for anything yet - and with
    * CRT SwitchRes off it never will be. Reported from mister_apply_mode
    * instead, where it describes the modeline actually in hand. */
   if (settings->uints.crt_switch_resolution == CRT_SWITCH_NONE)
      MISTER_WARN("CRT SwitchRes is off, so switchres will never resolve a "
                  "modeline and nothing can be streamed. Set it to the "
                  "monitor preset your CRT needs.\n");

   /* Subscribing to inputs and requesting capabilities both have to precede
    * CmdInit: the caps ride the init datagram, and the core only accepts an
    * input subscribe inside that handshake. Requesting against an older core
    * is safe - the client probes the version and negotiates down to v1 by
    * itself, which is why the granted set is read back rather than assumed. */
   if (settings->bools.mister_use_inputs)
   {
      gmw_bindInputs(settings->arrays.mister_ip);
      gmw_set_input_caps((uint8_t)(GMW_CAP_INPUTS_V2
            | (settings->bools.mister_rumble ? GMW_CAP_RUMBLE : 0)));
      mister_st.inputs_bound = true;
   }
   else
      gmw_set_input_caps(0);

   /* Permission for the core to end a silent session, not the mechanism: the
    * keepalives go out either way, since an older core times out regardless
    * and there is no way to tell the two apart from here.
    *
    * Kept out of gmw_set_input_caps, which would tie it to whether the user
    * wants MiSTer controllers. The client ORs both into one caps byte. */
   gmw_set_keepalive(settings->bools.mister_allow_idle_timeout ? 1 : 0);

   /* All of these ride CMD_INIT and must precede it. */
   if (codec == MISTER_CODEC_NLC)
   {
      gmw_set_nlc_pack((uint8_t)settings->uints.mister_nlc_pack);
      gmw_set_near_level((uint8_t)settings->uints.mister_nlc_near);
   }

   /* The core takes 22050, 44100 or 48000 and nothing else; a different output
    * rate would be mapped to "off" inside the client with no diagnostic. */
   if (!rate && settings->uints.audio_output_sample_rate)
      MISTER_WARN("Output sample rate %u is not one of 22050/44100/48000, "
                 "so audio will not be streamed. Change Audio Output Rate to "
                 "stream sound to the MiSTer.\n",
            settings->uints.audio_output_sample_rate);

   MISTER_INFO("Connecting to %s: codec %u, RGB%s, MTU %u, audio %u Hz.\n",
         settings->arrays.mister_ip, codec,
         (rgb_mode == MISTER_RGB565) ? "565" : "888",
         settings->uints.mister_mtu, rate);

   if (gmw_init(settings->arrays.mister_ip,
            (uint8_t)codec,
            rate, rate ? 2 : 0,
            (uint8_t)rgb_mode,
            (uint16_t)settings->uints.mister_mtu) < 0)
   {
      /* Loud while it might still be a real fault, then once on the way into
       * the slow cadence, then silent - a MiSTer that is simply switched off
       * must not fill the log for the rest of the session. */
      if (mister_st.attempts < MISTER_MAX_ATTEMPTS)
         MISTER_ERR("Could not open a session with %s (attempt %u of %d). If "
                   "the client gave a reason it is on the line above; "
                   "otherwise check the address, that the Groovy core is "
                   "running, and that UDP 32100 is open.\n",
               settings->arrays.mister_ip,
               mister_st.attempts + 1, MISTER_MAX_ATTEMPTS);
      else if (mister_st.attempts == MISTER_MAX_ATTEMPTS)
         MISTER_INFO("Still no answer from %s. Retrying quietly once a minute "
                    "for the rest of this run, so the stream starts on its own "
                    "if the MiSTer is switched on later.\n",
               settings->arrays.mister_ip);

      /* Tear the client down between attempts. gmw_bindInputs short-circuits on
       * a sticky "already bound" flag that only gmw_close() clears, so without
       * this the inputs socket stays pinned to whatever address the first
       * attempt used - correcting MiSTer Address would fix video and leave
       * input pointed at the old host. It also releases the socket and RIO
       * registrations the failed attempt left behind. */
      mister_st.inputs_bound = false;
      gmw_close();

      /* Saturate rather than latch off: this is a recoverable condition, and
       * mister_disable() is reserved for what the user has to fix by hand. */
      if (mister_st.attempts == MISTER_MAX_ATTEMPTS)
      {
         const char *_msg =
            "MiSTer not responding - still trying, see mister.log";
         runloop_msg_queue_push(_msg, strlen(_msg), 2, 300, true, NULL,
               MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_ERROR);
      }
      if (mister_st.attempts <= MISTER_MAX_ATTEMPTS)
         mister_st.attempts++;
      return;
   }

   mister_st.attempts        = 0;
   mister_st.connected       = true;


   /* input_caps is zeroed when MiSTer controllers are off, so the grant is
    * reported from the byte itself. */
   {
      uint8_t granted = gmw_get_input_caps();

      mister_st.input_caps   = settings->bools.mister_use_inputs ? granted : 0;

      if (!settings->bools.mister_allow_idle_timeout)
         MISTER_INFO("Allow Idle Timeout is off, so the core will hold this "
                    "session and the last frame on the CRT even if RetroArch "
                    "stops sending.\n");
      else if (granted & GMW_CAP_KEEPALIVE)
         MISTER_INFO("Idle timeout granted, so the core will free the CRT if "
                    "this session goes quiet.\n");
      else
         /* Pre-v2 cores drop the caps byte, and time out silent sessions
          * regardless. */
         MISTER_INFO("Idle timeout was requested but not granted; this core "
                    "is older than v2 and drops the capability byte.\n");
   }

   /* Name the core. A log covering several sessions is otherwise a set of
    * resolutions with nothing to attach them to, and which core produced which
    * numbers is usually the first thing worth knowing. */
   {
      runloop_state_t *rls = runloop_state_get_ptr();

      if (rls && rls->current_library_name[0])
         MISTER_INFO("Core: %s.\n", rls->current_library_name);
      else
         MISTER_INFO("No core loaded; this session is the menu.\n");
   }

   if (settings->bools.mister_use_inputs)
   {
      MISTER_INFO("Input capabilities negotiated: 0x%02x (inputs v2 %s, "
                "rumble %s).\n",
            mister_st.input_caps,
            (mister_st.input_caps & GMW_CAP_INPUTS_V2) ? "yes" : "no",
            (mister_st.input_caps & GMW_CAP_RUMBLE)    ? "yes" : "no");

      /* The pad datagrams are only read by the MiSTer joypad driver, and that
       * driver only runs when it is the selected one - RetroArch picks the
       * joypad driver by an exact name match, so any other choice leaves the
       * socket bound and nothing draining it. Worth one line, because the
       * symptom is silence rather than an error. */
      {
         const input_device_driver_t *jp = input_state_get_ptr()->primary_joypad;
         const char *ident = (jp && jp->ident) ? jp->ident : "(none)";

         if (string_is_equal(ident, "mister"))
            MISTER_INFO("Controller driver is \"mister\", so the MiSTer's pads "
                       "are being read.\n");
         else
            MISTER_WARN("Use MiSTer Controllers is on, but the active "
                       "controller driver is \"%s\". Only the \"mister\" "
                       "driver reads the MiSTer's pads - nothing will arrive "
                       "from them until Settings > Drivers > Controller is set "
                       "to it.\n", ident);
      }
   }

   if (codec == MISTER_CODEC_NLC)
      MISTER_INFO("NLC requested: pack %u (%s), near level %u.\n",
            settings->uints.mister_nlc_pack,
            (settings->uints.mister_nlc_pack == 2) ? "Rice" : "TILED",
            settings->uints.mister_nlc_near);
   mister_st.rgb_mode        = (uint8_t)rgb_mode;
   mister_st.codec           = (uint8_t)codec;
   mister_st.frame           = 0;
   mister_st.modeline_active = false;

   /* CMD_SWITCHRES is mandatory after every CMD_INIT: without a modeline the
    * core discards the stream. Replay whatever switchres last resolved. */
   if (mister_mode.width)
      mister_st.mode_pending = true;

   if (!mister_scaler)
      mister_scaler = (struct scaler_ctx*)calloc(1, sizeof(*mister_scaler));

   gmw_getStatus(&mister_status);

   /* A fresh session has no modeline, and switchres will not offer one unless
    * it thinks the geometry has changed. When a session opens under a core that
    * was already running - a core that loads twice, a reconnect - it has not,
    * so nothing would ever be resolved and the session would sit there sending
    * nothing at all. Ask for it explicitly rather than relying on a geometry
    * change that may never come. */
   if (!mister_st.mode_pending)
   {
      video_driver_state_t *vst = video_state_get_ptr();

      if (vst)
      {
         crt_switch_forget_resolved(&vst->crt_switch_st);
         MISTER_INFO("No modeline carried into this session; asking switchres "
                     "to resolve one again.\n");
      }
   }

   /* Last, and only once every other call above has been made: from here on the
    * sender thread owns the client, and this one must not touch it again
    * outside mister_sender_idle(). If it will not start there is no stream -
    * letting the frame loop keep talking to the client instead is exactly the
    * arrangement this replaced. */
   if (!mister_sender_start())
   {
      mister_st.connected = false;
      gmw_close();
      mister_disable("the sender thread could not be started.",
            "MiSTer output disabled - see mister.log");
   }
}

static void mister_apply_mode(video_driver_state_t *video_st)
{
   settings_t              *settings = config_get_ptr();
   const mister_modeline_t *srm      = &mister_mode;
   unsigned                 bpp      = (mister_st.rgb_mode == MISTER_RGB565) ? 2 : 3;
   uint8_t                  interlace;

   /* interlace 1 sends one half-height field per blit; interlace 2 keeps a
    * progressive framebuffer and lets the core scan fields out of it, which
    * is what a core producing whole frames at the field rate wants. */
   if (!srm->interlace)
      interlace = 0;
   else
      interlace = settings->bools.mister_interlaced_fb ? 1 : 2;

   /* Sized before anything is committed to mister_st: a mode that fails the
    * check below is never programmed, and leaving the state struct describing
    * it would make every later size calculation lie. */
   {
      unsigned cand_interlaced = (interlace == 1) ? 1 : 0;
      unsigned cand_fb_height  = cand_interlaced
         ? srm->height / 2 : srm->height;
      size_t   cand_bytes      = (size_t)srm->width * cand_fb_height * bpp;

      if (cand_bytes > MISTER_MAX_BYTES)
      {
         /* Latched on the mode itself, not on a "once per session" flag: the
          * next mode may well fit, and the user needs to be told again if a
          * different one does not. */
         if (     mister_st.refused_width  != srm->width
               || mister_st.refused_height != srm->height)
         {
            char _m[128];
            size_t _l;

            mister_st.refused_width  = srm->width;
            mister_st.refused_height = srm->height;

            MISTER_ERR("Refusing modeline %ux%u: %u bytes per blit exceeds the "
                       "%d byte buffer. %s\n",
                  srm->width, srm->height, (unsigned)cand_bytes,
                  MISTER_MAX_BYTES,
                  (srm->width > MISTER_MAX_WIDTH)
                  ? "The width is the problem - turn CRT Super Resolution off."
                  : "Use a lower core resolution or a smaller monitor preset.");

            _l = (size_t)snprintf(_m, sizeof(_m),
                  "MiSTer: %ux%u is too large to stream - see mister.log",
                  srm->width, srm->height);
            runloop_msg_queue_push(_m, _l, 2, 300, true, NULL,
                  MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_ERROR);
         }

         /* Deliberately leaves mode_pending set. switchres resolves a new
          * modeline whenever the core geometry changes, and the next one is
          * retried rather than the session latching off for good. */
         return;
      }

      mister_st.interlaced     = (uint8_t)cand_interlaced;
      mister_st.width          = srm->width;
      mister_st.height         = srm->height;
      mister_st.fb_height      = cand_fb_height;
      mister_st.refused_width  = 0;
      mister_st.refused_height = 0;
   }

   /* Report the frequencies rather than only the raw numbers: whether a
    * modeline is displayable is a question about hfreq, and "no picture on a
    * CRT" is nearly always a modeline the monitor cannot scan. */
   {
      double hfreq = (srm->htotal > 0)
         ? srm->pclock / (double)srm->htotal : 0.0;
      double vfreq = (srm->vtotal > 0 && hfreq > 0.0)
         ? hfreq / (double)srm->vtotal : 0.0;

      if (srm->interlace)
         vfreq *= 2.0;

      MISTER_INFO("Modeline %ux%u%s @ %.3f Hz, %.3f kHz horizontal, "
                "pclock %.4f MHz, scale %.4f x %.4f%s.\n",
            srm->width, srm->height, srm->interlace ? "i" : "p",
            vfreq, hfreq / 1000.0, srm->pclock / 1000000.0,
            srm->x_scale, srm->y_scale,
            srm->is_stretched ? ", stretched" : "");

      /* Why it is interlaced, in one line, because the usual reaction is that
       * the monitor can do 480p so it should have got 480p. */
      if (srm->interlace && hfreq > 0.0)
         MISTER_INFO("Interlaced because %ux%u progressive at %.3f Hz would "
                    "need about %.1f kHz, which is past what this monitor "
                    "preset can scan.\n",
               srm->width, srm->height, vfreq, hfreq * 2.0 / 1000.0);

      /* What the core asked for against what it got. A preset that cannot
       * scan the requested geometry is answered with the nearest mode it can,
       * which may be interlaced or at a different refresh - and that changes
       * how the core runs, because the modeline becomes the frame clock. It is
       * invisible otherwise: the picture is right and only the timing is off. */
      {
         video_driver_state_t *vst = video_state_get_ptr();
         double  want_hz           = vst ? (double)vst->core_hz : 0.0;

         if (want_hz > 0.0 && vfreq > 0.0)
         {
            double skew = fabs(1.0 - want_hz / vfreq);

            if (skew > 0.001)
            {
               settings_t *st  = config_get_ptr();
               float       cap = st ? st->floats.audio_max_timing_skew : 0.05f;

               MISTER_INFO("The core asked for %.3f Hz and the preset offered "
                          "%.3f Hz, a %.2f%% difference. The modeline is the "
                          "frame clock, so the core runs at the offered "
                          "rate and the audio is resampled to match.\n",
                     want_hz, vfreq, skew * 100.0);

               /* Ordinarily a gap this wide would make RetroArch give up on
                * adjusting the audio rate; under CRT SwitchRes the limit is
                * lifted, because the modeline was chosen for this core and the
                * gap is real rather than a misconfigured display. */
               if (skew > (double)cap)
                  MISTER_INFO("That is wider than Audio Max Timing Skew "
                             "(%.2f%%), which normally means no correction at "
                             "all. CRT SwitchRes lifts that limit, so the "
                             "sound is still being matched to %.3f Hz.\n",
                        cap * 100.0f, vfreq);

               /* Say it on screen too, not only in the log.
                *
                * The whole game is running this much slow, sound included, and
                * that is not something to discover from a log file after
                * wondering why a title feels wrong. Reached only when the mode
                * was kept anyway - either the speed limit is off, or no smaller
                * mode resolved any closer - so a default setup never sees it.
                *
                * Two percent is the floor: below that it is not worth
                * interrupting anyone over, and the common 59.94-against-60.000
                * case is 0.1%. */
               if (skew > 0.02)
               {
                  char   _m[160];
                  size_t _l = (size_t)snprintf(_m, sizeof(_m),
                        "MiSTer: running %.1f%% slow (%.2f Hz, core wants "
                        "%.2f Hz) - see Max Speed Loss",
                        skew * 100.0, vfreq, want_hz);

                  runloop_msg_queue_push(_m, _l, 2, 480, true, NULL,
                        MESSAGE_QUEUE_ICON_DEFAULT,
                        MESSAGE_QUEUE_CATEGORY_WARNING);
               }
            }
         }
      }


      /* 15 kHz tubes cannot scan much past ~16 kHz; handing one a 31 kHz mode
       * is the most common way to end up staring at a blank CRT. Only the
       * single-range presets are checked - a multi-sync one legitimately
       * covers both, and switchres itself rejects anything out of range. */
      if (hfreq > 0.0)
      {
         unsigned    crt_mode = config_get_ptr()->uints.crt_switch_resolution;
         const char *preset   = crt_switch_monitor_preset(crt_mode);
         double      lo_hz    = 0.0;
         double      hi_hz    = 0.0;

         /* The preset classification lives in video_crt_switch.c, where the
          * mode selection can also act on it - reaching here at all now means
          * something granted a mode its own preset should have refused. */
         crt_monitor_hfreq_bounds(crt_mode, &lo_hz, &hi_hz);

         if (hi_hz > 0.0 && hfreq > hi_hz)
            MISTER_WARN("%.3f kHz exceeds what the \"%s\" "
                       "monitor preset can scan. If the CRT stays blank "
                       "this is why: either the content needs an "
                       "interlaced mode or the display is a 31 kHz one "
                       "and CRT SwitchRes should say so.\n",
                  hfreq / 1000.0, preset ? preset : "");
         else if (lo_hz > 0.0 && hfreq < lo_hz)
            MISTER_WARN("%.3f kHz is below the 31 kHz range the "
                       "\"%s\" monitor preset expects; a VGA monitor "
                       "will likely not sync to it.\n",
                  hfreq / 1000.0, preset ? preset : "");
      }
   }

   {
      int sr_rc = gmw_switchres(srm->pclock / 1000000.0,
            (uint16_t)srm->width,  (uint16_t)srm->hbegin,
            (uint16_t)srm->hend,   (uint16_t)srm->htotal,
            (uint16_t)srm->height, (uint16_t)srm->vbegin,
            (uint16_t)srm->vend,   (uint16_t)srm->vtotal,
            interlace);

      /* A modeline the core never acknowledged leaves it discarding every frame
       * for the rest of the session while everything here still looks healthy -
       * no error, no dropped frames, just a picture that never arrives. Until
       * the client reported it we could only infer it, by watching the echo
       * counter fail to move across a hundred blits. It is answered directly
       * now, so say so at once and leave the modeline inactive rather than
       * streaming into a core that is throwing it all away. */
      if (sr_rc != 0)
      {
         const char *_m = "MiSTer did not accept the video mode";

         MISTER_WARN("The core did not acknowledge the modeline (rc %d). It "
                    "would discard every frame of this session, so the stream "
                    "is being held. The client already retried internally, so "
                    "the next attempt waits for a geometry change or a "
                    "reconnect rather than re-sending this every frame.\n",
               sr_rc);
         runloop_msg_queue_push(_m, strlen(_m), 2, 300, true, NULL,
               MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_WARNING);

         /* Cleared, not left pending: mister_apply_mode runs from the frame
          * loop whenever this is set, so leaving it would re-send CmdSwitchres
          * sixty times a second at a core that is already not answering.
          * modeline_active stays false, so nothing is streamed meanwhile. */
         mister_st.mode_pending    = false;
         mister_st.modeline_active = false;
         return;
      }
   }

   mister_st.field           = 0;
   /* A new modeline is a new raster, so the interlace phase is re-taken from
    * the FPGA on the next field rather than carried across the change. */
   mister_st.field_seeded    = 0;
   mister_st.last_field      = 0;
   /* Reached only once the core has acknowledged the modeline - checked above
    * from gmw_switchres's return, which upstream added at our request. */
   mister_st.modeline_active = true;

   /* The modeline is the frame clock, so its period is what the loop should be
    * spending per frame. Used by mister_sync() to know how much of the frame
    * is still ahead of it and can therefore be slept through rather than spun
    * away. Zero disables that and leaves the client to pace on its own. */
   {
      double hfreq = (srm->htotal > 0)
         ? srm->pclock / (double)srm->htotal : 0.0;
      double vfreq = (srm->vtotal > 0 && hfreq > 0.0)
         ? hfreq / (double)srm->vtotal : 0.0;

      if (srm->interlace)
         vfreq *= 2.0;

      mister_frame_period_usec = (vfreq > 1.0)
         ? (retro_time_t)(1000000.0 / vfreq) : 0;
      mister_frame_end_usec    = 0;

      /* The core's vga_frame is its vblank counter, and an interlaced mode
       * ticks it per field (rtl/vga.v) - the same rate the doubled vfreq above
       * carries, so one figure covers both scan modes. A new modeline means a
       * fresh FIFO prefill on the core side, so the servo starts over:
       * disarmed here, re-armed by the first flush the core accepts. */
      {
         uint32_t rate = mister_audio_rate(
               config_get_ptr()->uints.audio_output_sample_rate);

         if (mister_audio_lock)
            slock_lock(mister_audio_lock);

         mister_audio_spf   = (rate && vfreq > 1.0) ? rate / vfreq : 0.0;
         mister_audio_armed = false;
         mister_audio_ratio = 1.0;
         mister_audio_phase = 0.0;
         mister_audio_drift = 0.0;
         mister_audio_sent  = 0;
         mister_audio_hold_valid = false;

         /* See the note on the constant. */
         mister_audio_watermark = (mister_audio_spf > 1.0)
            ? (size_t)mister_audio_spf : MISTER_AUDIO_PACKET_FRAMES;
         if (mister_audio_watermark < MISTER_AUDIO_WATERMARK_MIN)
            mister_audio_watermark = MISTER_AUDIO_WATERMARK_MIN;
         if (mister_audio_watermark > MISTER_AUDIO_PACKET_FRAMES)
            mister_audio_watermark = MISTER_AUDIO_PACKET_FRAMES;

         if (mister_audio_lock)
            slock_unlock(mister_audio_lock);
      }
   }
   mister_st.blits_since_mode = 0;
   mister_st.warned_no_echo   = false;
   mister_st.echo_at_mode     = mister_status.frameEcho;
   mister_st.mode_pending    = false;
   mister_st.must_clear      = true;
   mister_st.trace_blit      = true;

   /* Pin the render viewport to the modeline so a GPU-rendered core reads back
    * exactly these pixels. Harmless for software cores, which never read back;
    * done here because it has to be redone on every mode change. */
   mister_st.hw_pinned   = mister_pin_viewport(video_st,
         mister_st.width, mister_st.height);
   mister_st.pinned_data = video_st->data;
}


/* RGUI hands over its own 16-bit framebuffer rather than an RGB frame.
 * Its platform format is (r << 12) | (g << 8) | (b << 4) | a; the 4-bit
 * channels expand to 8 by replication. */
/* scale replicates each pixel that many times in both directions. RGUI draws a
 * fixed 240-line bitmap whatever mode it is shown in, so with the high
 * resolution menu enabled it would otherwise sit in the middle of a 480-line
 * modeline surrounded by black. An integer factor is used rather than a
 * general scaler because it is exact - the result is indistinguishable from
 * the same menu at 240p - and because it costs one pass with no second
 * buffer. A mode that is not a whole multiple keeps the previous behaviour and
 * is centred by mister_place. */
static void mister_menu_to_bgr24(uint8_t *dst, const uint16_t *src,
      unsigned width, unsigned height, size_t src_pitch, unsigned scale)
{
   unsigned x, y, i;
   unsigned r, g, b;
   uint16_t pixel;
   size_t   dst_stride;

   if (!scale)
      scale = 1;

   dst_stride = (size_t)width * scale * 3;

   for (y = 0; y < height; y++)
   {
      const uint16_t *row = (const uint16_t*)((const uint8_t*)src + y * src_pitch);
      uint8_t        *out = dst + (size_t)y * scale * dst_stride;
      uint8_t        *p   = out;

      for (x = 0; x < width; x++)
      {
         pixel  = row[x];
         r      = (pixel >> 12) & 0xf;
         g      = (pixel >>  8) & 0xf;
         b      = (pixel >>  4) & 0xf;

         for (i = 0; i < scale; i++)
         {
            *p++ = (uint8_t)((b << 4) | b);
            *p++ = (uint8_t)((g << 4) | g);
            *p++ = (uint8_t)((r << 4) | r);
         }
      }

      /* Copy the finished row rather than converting it again. */
      for (i = 1; i < scale; i++)
         memcpy(out + (size_t)i * dst_stride, out, dst_stride);
   }
}

/* Copy one BGR24 image into the registered blit buffer, centred, clipped to
 * the modeline, rotated as the core asked, and reduced to the field being
 * sent when the framebuffer is interlaced. */
/* flip_v reverses the source row order as the copy is made. read_viewport
 * hands back bottom-up images (see tasks/task_screenshot.c) and the blit
 * buffer wants top-down; doing it here costs nothing, where a separate pass
 * over the whole staging buffer was the single most expensive step on the
 * hardware-rendered path. */
static void mister_place(const uint8_t *src, unsigned src_w, unsigned src_h,
      uint8_t *fb, unsigned field, bool flip_v)
{
   unsigned rotation = retroarch_get_rotation();
   unsigned rot_w    = (rotation & 1) ? src_h : src_w;
   unsigned rot_h    = (rotation & 1) ? src_w : src_h;
   unsigned dst_w    = mister_st.width;
   unsigned dst_h    = mister_st.height;
   unsigned bpp      = (mister_st.rgb_mode == MISTER_RGB565) ? 2 : 3;
   unsigned copy_w   = (rot_w < dst_w) ? rot_w : dst_w;
   unsigned copy_h   = (rot_h < dst_h) ? rot_h : dst_h;
   unsigned dst_x    = (dst_w - copy_w) / 2;
   unsigned dst_y    = (dst_h - copy_h) / 2;
   unsigned src_x    = (rot_w - copy_w) / 2;
   unsigned src_y    = (rot_h - copy_h) / 2;
   unsigned step     = mister_st.interlaced ? 2 : 1;
   unsigned x, y;

   for (y = 0; y < copy_h; y++)
   {
      unsigned line = dst_y + y;
      uint8_t *out;

      /* Interlaced framebuffers carry only this field's lines, packed. */
      if (mister_st.interlaced)
      {
         if ((line & 1) != field)
            continue;
         out = fb + ((line >> 1) * dst_w + dst_x) * bpp;
      }
      else
         out = fb + (line * dst_w + dst_x) * bpp;

      if (rotation == ORIENTATION_NORMAL && bpp == 3)
      {
         /* The overwhelmingly common case: one contiguous run per line. */
         unsigned srow = flip_v ? (src_h - 1 - (src_y + y)) : (src_y + y);
         memcpy(out, src + (srow * src_w + src_x) * 3, copy_w * 3);
         continue;
      }

      for (x = 0; x < copy_w; x++)
      {
         unsigned sx, sy;
         const uint8_t *in;

         switch (rotation)
         {
            case ORIENTATION_VERTICAL:
               sx = src_y + y;
               sy = src_h - 1 - (src_x + x);
               break;
            case ORIENTATION_FLIPPED:
               sx = src_w - 1 - (src_x + x);
               sy = src_h - 1 - (src_y + y);
               break;
            case ORIENTATION_FLIPPED_ROTATED:
               sx = src_w - 1 - (src_y + y);
               sy = src_x + x;
               break;
            default:
               sx = src_x + x;
               sy = src_y + y;
               break;
         }

         if (flip_v)
            sy = src_h - 1 - sy;

         in = src + (sy * src_w + sx) * 3;

         if (bpp == 2)
         {
            uint16_t *out16 = (uint16_t*)(out + x * 2);
            *out16 = (uint16_t)(((in[2] & 0xf8) << 8)
                              | ((in[1] & 0xfc) << 3)
                              | ( in[0]         >> 3));
         }
         else
         {
            out[x * 3 + 0] = in[0];
            out[x * 3 + 1] = in[1];
            out[x * 3 + 2] = in[2];
         }
      }
      (void)step;
   }
}

/* Scale and convert the source frame to a BGR24 staging image, returning its
 * dimensions. RetroArch's own scaler handles both, including the byte order
 * the MiSTer expects. */
static bool mister_stage_frame(const void *data, unsigned width,
      unsigned height, size_t pitch, enum scaler_pix_fmt in_fmt,
      double x_scale, double y_scale, bool smooth,
      unsigned *out_w, unsigned *out_h)
{
   unsigned sw = (unsigned)(width  * x_scale + 0.5);
   unsigned sh = (unsigned)(height * y_scale + 0.5);

   if (!sw || !sh)
      return false;

   if (!mister_stage_reserve((size_t)sw * sh * 3))
      return false;

   if (     mister_scaler->in_width    != (int)width
         || mister_scaler->in_height   != (int)height
         || mister_scaler->in_stride   != (int)pitch
         || mister_scaler->in_fmt      != in_fmt
         || mister_scaler->out_width   != (int)sw
         || mister_scaler->out_height  != (int)sh)
   {
      mister_scaler->in_fmt      = in_fmt;
      mister_scaler->in_width    = (int)width;
      mister_scaler->in_height   = (int)height;
      mister_scaler->in_stride   = (int)pitch;
      mister_scaler->out_fmt     = SCALER_FMT_BGR24;
      mister_scaler->out_width   = (int)sw;
      mister_scaler->out_height  = (int)sh;
      mister_scaler->out_stride  = (int)(sw * 3);
      mister_scaler->scaler_type = smooth
         ? SCALER_TYPE_BILINEAR : SCALER_TYPE_POINT;

      if (!scaler_ctx_gen_filter(mister_scaler))
      {
         MISTER_ERR("Failed to build the %ux%u scaler.\n", sw, sh);
         return false;
      }
   }

   scaler_ctx_scale_direct(mister_scaler, mister_stage, data);

   *out_w = sw;
   *out_h = sh;
   return true;
}

/* Wait until the sender has nothing in hand.
 *
 * Every structural change - opening a session, closing one, programming a
 * modeline - goes through here first. It is cruder than locking each shared
 * field, and that is the point: those things happen occasionally, so the cost
 * is nothing and the rule is easy to keep true. Safe before the thread
 * exists. */
static void mister_sender_loop(void *unused);

/* Start the sender. Called with a live session and nothing queued. */
static bool mister_sender_start(void)
{
   if (mister_sender)
      return true;

   mister_send_quit    = false;
   mister_send_working = false;
   mister_job_queued   = -1;
   mister_job_inflight = -1;
   mister_stage_write  = 0;

   if (!mister_send_lock)
      mister_send_lock = slock_new();
   if (!mister_send_ready)
      mister_send_ready = scond_new();
   if (!mister_send_idle)
      mister_send_idle = scond_new();
   if (!mister_audio_lock)
      mister_audio_lock = slock_new();

   if (!mister_send_lock || !mister_send_ready || !mister_send_idle
         || !mister_audio_lock)
   {
      MISTER_ERR("Could not create the sender thread's locks; the stream "
                 "cannot start.\n");
      return false;
   }

   if (!(mister_sender = sthread_create(mister_sender_loop, NULL)))
   {
      MISTER_ERR("Could not start the sender thread; the stream cannot "
                 "start.\n");
      return false;
   }

   return true;
}

/* Stop the sender and wait for it. After this returns the calling thread owns
 * the client again, which is what makes closing a session safe. */
static void mister_sender_stop(void)
{
   if (!mister_sender)
      return;

   slock_lock(mister_send_lock);
   mister_send_quit = true;
   scond_signal(mister_send_ready);
   scond_signal(mister_send_idle);
   slock_unlock(mister_send_lock);

   sthread_join(mister_sender);
   mister_sender       = NULL;
   mister_send_working = false;
   mister_job_queued   = -1;
   mister_job_inflight = -1;
}

static void mister_sender_idle(void)
{
   if (!mister_sender)
      return;

   slock_lock(mister_send_lock);
   while ((mister_job_queued >= 0 || mister_send_working) && !mister_send_quit)
      scond_wait(mister_send_idle, mister_send_lock);
   slock_unlock(mister_send_lock);
}

static void mister_send_job(const mister_job_t *job);

/* The frame's pacing wait. Runs on the sender thread, which is the only
 * caller of the client, and is what makes the raster the clock. */
static void mister_pace_frame(void)
{
   if (!mister_st.connected)
      return;

   /* Hand the rest of the frame back to the machine before waiting on the
    * raster.
    *
    * gmw_waitSync() lands on the raster by spinning - no sleep, no yield - so
    * on its own it holds a core for whatever is left of the frame. That is
    * affordable when the core is light and ruinous when it is not: a heavy
    * multi-threaded core wants every thread it can get, and a spinning loop is
    * a thread it cannot have. Sleeping until shortly before the deadline gives
    * that time back, and leaves the spin only the last stretch it needs to be
    * accurate. Costs nothing in latency - this frame was sent before we got
    * here, so the wait is idle time ahead of the next one.
    *
    * Skipped when the period is unknown, or when the frame already overran, in
    * which case there is nothing to give back. */
   /* Only when the raster is meant to be the clock.
    *
    * Under host pacing the frame loop is the clock, and this thread waiting on
    * the raster as well is a second one - the two beat against each other and
    * the newest-wins handoff throws away the loser. A hardware run discarded
    * 647 frames that way, which is what an occasional dropped frame on screen
    * looks like from here. gmw_waitSync() below still runs in both modes: it is
    * where the send queue drains and the ACK arrives, and with no sleep ahead
    * of it there is nothing left for it to wait on. */
   if (      mister_pacing_mode() == MISTER_PACING_MISTER
         && mister_frame_period_usec > MISTER_SPIN_MARGIN_USEC
         && mister_frame_end_usec)
   {
      retro_time_t now  = cpu_features_get_time_usec();
      retro_time_t left = (mister_frame_end_usec + mister_frame_period_usec)
                        - now - MISTER_SPIN_MARGIN_USEC;

      /* Bounded by the frame period so a stale timestamp - a long pause, a
       * mode change - cannot turn into a long sleep. */
      if (left > 0 && left < mister_frame_period_usec)
         retro_sleep_us((unsigned)left);
   }

   /* Always, in both pacing modes.
    *
    * This is not only the raster wait. It is also where the client drains its
    * send completion queue on Windows, and where it picks up the ACK that
    * moves the raster servo. Skipping it under host pacing - which this used
    * to do - let the completion queue fill until sends began failing silently,
    * which is why host pacing had to be disabled on Windows at all. */
   gmw_waitSync();

   mister_frame_end_usec = cpu_features_get_time_usec();
}

static void mister_sender_loop(void *unused)
{
   (void)unused;

   for (;;)
   {
      mister_job_t job;

      slock_lock(mister_send_lock);
      while (mister_job_queued < 0 && !mister_send_quit)
      {
         /* A timed wait, so an idle session still gets its keepalive. This is
          * also what holds the session open while a modal window has the frame
          * loop stopped: this thread is not on the message pump, so nothing
          * the window does can stop it.
          *
          * The housekeeping runs whether the wait timed out or was signalled,
          * because a signal with no frame queued is the audio path asking for a
          * flush before its buffer overruns. Only checking the timeout - which
          * is what this did at first - left that wake doing nothing at all. */
         scond_wait_timeout(mister_send_ready, mister_send_lock,
               MISTER_SENDER_IDLE_USEC);

         if (mister_job_queued >= 0 || mister_send_quit)
            break;

         slock_unlock(mister_send_lock);
         mister_keepalive();
         gmw_getACK(0);
         mister_audio_flush();
         slock_lock(mister_send_lock);
      }

      if (mister_send_quit)
      {
         slock_unlock(mister_send_lock);
         break;
      }

      /* Take the frame and free the slot straight away, before any of the work.
       * That is what lets the frame loop get on with the next frame while this
       * one is still being encoded and sent. */
      job                 = mister_job;
      mister_job_inflight = mister_job_queued;
      mister_job_queued   = -1;
      mister_send_working = true;
      scond_signal(mister_send_idle);
      slock_unlock(mister_send_lock);

      mister_send_job(&job);
      mister_audio_flush();
      mister_pace_frame();

      slock_lock(mister_send_lock);
      mister_job_inflight = -1;
      mister_send_working = false;
      scond_signal(mister_send_idle);
      slock_unlock(mister_send_lock);
   }
}

/* Hand one frame to the sender.
 *
 * Under MiSTer pacing this blocks until the sender is free, which is what keeps
 * the raster the clock - the wait that used to sit in the frame loop now sits
 * here, and the backpressure reaches the core the same way. Under host pacing
 * the newest frame wins: one the CRT has not shown yet is worth less than the
 * one just drawn, so drop it rather than grow a backlog.
 *
 * The staging buffers are swapped rather than copied, so the frame loop never
 * writes into the buffer the sender is reading. */
/* The raster line the FPGA is told to commit the frame at.
 *
 * Passing 0 here asks the client to derive it from its own measurements of
 * emulation and stream time. Those are taken between WaitSync calls, and
 * WaitSync runs on the sender thread, whose cadence is the send queue rather
 * than the core - so the figures do not mean what the heuristic assumes and
 * the line it returns wanders across the whole raster. Measured over a session
 * it settled around line 58 for two frames in three and in the bottom half of
 * the picture for one in five, which is a frame committed part-way through
 * scanout: a tear.
 *
 * So it is chosen here instead, and by default it is line 1 - the top of the
 * raster, before the beam reaches the picture. Frame delay remains the way to
 * trade that safety back for latency, exactly as it does for the host display,
 * which is also how the upstream integration drives this.
 *
 * Computed on the frame loop's thread, where the settings and the driver state
 * are already in hand, and carried to the sender in the job. */
static uint16_t mister_vsync_line(void)
{
   settings_t           *settings = config_get_ptr();
   video_driver_state_t *video_st = video_state_get_ptr();
   unsigned              height   = mister_st.height;
   unsigned              delay    = 0;

   if (settings->bools.video_frame_delay_auto && video_st->frame_delay_effective > 0)
      delay = video_st->frame_delay_effective;
   else
      delay = settings->uints.video_frame_delay;

   /* Frame delay is a slice of a ~16 ms frame, so 16/delay is how many slices
    * fit; past 16 ms there is no slice left and the division would trap. */
   if (delay == 0 || delay > 16 || height == 0)
      return 1;

   {
      unsigned line = height / (16 / delay);

      /* Never past the active area, and never 0 - 0 would hand the frame back
       * to the heuristic this exists to avoid. */
      if (line >= height)
         line = height - 1;
      return (uint16_t)(line ? line : 1);
   }
}

static void mister_submit(unsigned stage_w, unsigned stage_h, bool flip_v,
      unsigned src_w, unsigned src_h, unsigned src_pitch, int src_fmt,
      bool src_menu)
{
   int i;

   if (!mister_sender)
      return;

   slock_lock(mister_send_lock);

   if (mister_pacing_mode() == MISTER_PACING_MISTER)
   {
      /* Wait for the slot, not for the sender to finish. The raster still
       * paces this loop - the sender only takes the next frame once it has
       * come back round - but the encode and the send of the previous one
       * overlap the emulation of this one instead of blocking it. */
      while (mister_job_queued >= 0 && !mister_send_quit)
         scond_wait(mister_send_idle, mister_send_lock);
   }
   else if (mister_job_queued >= 0)
   {
      /* Host pacing: a frame the CRT has not shown yet is worth less than the
       * one just drawn, so let this one go rather than grow a backlog. */
      slock_unlock(mister_send_lock);
      mister_stall_set(MISTER_STALL_SENDER_BUSY);
      return;
   }

   if (mister_send_quit)
   {
      slock_unlock(mister_send_lock);
      return;
   }

   /* Hand the written buffer over and take a free one. With three buffers and
    * at most two spoken for, there is always exactly one left. */
   mister_stage_bufsize[mister_stage_write] = mister_stage_size;
   mister_job_queued                        = mister_stage_write;

   for (i = 0; i < MISTER_STAGE_BUFS; i++)
   {
      if (i != mister_job_queued && i != mister_job_inflight)
      {
         mister_stage_write = i;
         break;
      }
   }

   mister_stage      = mister_stage_buf[mister_stage_write];
   mister_stage_size = mister_stage_bufsize[mister_stage_write];

   mister_job.stage     = mister_stage_buf[mister_job_queued];
   mister_job.stage_w   = stage_w;
   mister_job.stage_h   = stage_h;
   mister_job.flip_v    = flip_v;
   mister_job.trace      = mister_st.trace_blit;
   mister_job.vsync_line = mister_vsync_line();
   mister_job.src_w     = src_w;
   mister_job.src_h     = src_h;
   mister_job.src_pitch = src_pitch;
   mister_job.src_fmt   = src_fmt;
   mister_job.src_menu  = src_menu;
   mister_job.src_hw    = flip_v;

   mister_st.trace_blit = false;
   scond_signal(mister_send_ready);
   slock_unlock(mister_send_lock);

   /* Cleared here rather than on the sender: the stall bookkeeping is the frame
    * loop's, and having one thread set it and another clear it would race on
    * the counters and mis-order the log lines. Handing a frame over is what
    * counts as "not stalled" from this side. */
   mister_stall_set(MISTER_STALL_NONE);
}

/* Everything that touches the client for one frame: fill its blit buffer,
 * read back its status, and put the frame on the wire. Runs on the sender
 * thread and nowhere else.
 *
 * The locals below carry the names the frame loop used, so this reads as it
 * did when it lived there. */
static void mister_send_job(const mister_job_t *job)
{
   uint8_t *fb;
   unsigned stage_w = job->stage_w;
   unsigned stage_h = job->stage_h;
   bool     hw_frame = job->flip_v;
   unsigned width   = job->src_w;
   unsigned height  = job->src_h;
   unsigned pitch   = job->src_pitch;
   bool     menu_on = job->src_menu;

   /* Fetched here, at the top, because the field decision below is the part
    * that needs it freshest - and the monotonic frame-number check further
    * down is unaffected by reading a status a few hundred microseconds older. */
   gmw_getStatus(&mister_status);

   /* The field is decided here, on the sender thread, and not back in the
    * frame loop.
    *
    * It used to be chosen in mister_draw, before mister_submit had decided
    * whether the frame would be sent at all. Under host pacing mister_submit
    * drops a frame whenever the previous one is still going out, so the field
    * that frame had claimed was spent without ever reaching the wire and the
    * next one sent repeated it. That is why the repeat rate tracked the pacing
    * mode rather than anything about interlacing: 1.98% where the queue blocks
    * against 11.77% where it drops, and about two repeats per drop - one for
    * the lost field and one more when the hysteresis below spent a frame
    * putting the phase back. Deciding it here means only a frame that is
    * actually going out can consume a field. */
   mister_st.field = 0;
   if (mister_st.interlaced)
   {
      /* Alternate unconditionally, and never correct the phase afterwards.
       *
       * Taking the field straight from the FPGA's vgaF1 each frame looks like
       * it keeps the two in step, but the status is a poll old: sample it
       * twice at the same phase and the same field goes out twice, writing one
       * half of the raster again and leaving the other half a frame stale.
       * Alternating from our own last field cannot do that.
       *
       * We used to still consult vgaF1 as slow evidence, holding a field once
       * after four consecutive disagreements to slip back into phase. Measured
       * from the core side, that was the only thing still producing repeats -
       * 0.60% of an interlaced session, matching the resync count exactly, and
       * every one a visible artifact. It was buying nothing: the core's own
       * measurement puts our field and vgaF1 in agreement 74.6% of the time
       * but flipping across 1,885 separate stretches in 6,688 blits, so a
       * vgaF1 sampled at whatever raster position a blit lands on has no
       * stable relationship to the field we are sending. There is no phase
       * there to correct towards.
       *
       * Nothing is lost by free-running: the interlaced framebuffer is
       * double-buffered per field (DDR_FB / DDR_FD alternating), so the field
       * number we send selects its own buffer and the content cannot land in
       * the wrong half. vgaF1 still seeds the phase once, which is all it is
       * good for. */
      if (!mister_st.field_seeded)
      {
         /* First interlaced frame of the session or the mode: take the FPGA's
          * phase as given, there is nothing yet to alternate from. */
         mister_st.field        = (uint8_t)(mister_status.vgaF1 ? 0 : 1);
         mister_st.field_seeded = 1;
      }
      else
         mister_st.field = mister_st.last_field ^ 1;

      mister_st.last_field = mister_st.field;
   }

   /* Fetched after the decision above, because which of the client's two blit
    * buffers this is depends on the field. */
   fb = (uint8_t*)gmw_get_pBufferBlit(mister_st.field);
   if (!fb)
   {
      mister_stall_set(MISTER_STALL_NO_BLIT_BUFFER);
      return;
   }

   if (mister_st.must_clear)
   {
      unsigned bpp = (mister_st.rgb_mode == MISTER_RGB565) ? 2 : 3;
      memset(fb, 0, (size_t)mister_st.width * mister_st.fb_height * bpp);
      /* Both fields carry their own buffer; clear the one we are not on too. */
      if (mister_st.interlaced)
      {
         uint8_t *other = (uint8_t*)gmw_get_pBufferBlit(mister_st.field ? 0 : 1);
         if (other)
            memset(other, 0, (size_t)mister_st.width * mister_st.fb_height * bpp);
      }
      mister_st.must_clear = false;
   }

   {
      bool         probe = mister_pace_logging();
      retro_time_t t0    = probe ? cpu_features_get_time_usec() : 0;

      mister_place(job->stage, stage_w, stage_h, fb, mister_st.field, hw_frame);

      if (probe)
      {
         mister_st.t_stage += (uint64_t)(cpu_features_get_time_usec() - t0);
         mister_st.t_samples++;
      }
   }

   /* One-shot trace of the whole pixel path, on the first blit after a mode
    * change. Per-frame logging would drown the log and perturb the timing;
    * once per mode is enough to see where a blank picture comes from. The
    * non-black count is the load-bearing number: it separates "we sent an
    * empty frame" from "we sent pixels and something downstream lost them". */
   if (job->trace)
   {
      size_t   fb_bytes = (size_t)mister_st.width * mister_st.fb_height
                        * ((mister_st.rgb_mode == MISTER_RGB565) ? 2 : 3);
      size_t   i;
      unsigned nonzero  = 0;

      for (i = 0; i < fb_bytes; i++)
         if (fb[i])
            nonzero++;


      MISTER_INFO("Source %ux%u pitch %u fmt %d%s -> staged %ux%u -> "
                "framebuffer %ux%u (%u lines/blit) %s, rotation %u.\n",
            width, height, (unsigned)pitch, job->src_fmt,
            menu_on ? " (menu)" : (hw_frame ? " (gpu readback)" : ""),
            stage_w, stage_h,
            mister_st.width, mister_st.height, mister_st.fb_height,
            (mister_st.rgb_mode == MISTER_RGB565) ? "RGB565" : "RGB888",
            retroarch_get_rotation());

      if (nonzero == 0)
         MISTER_WARN("The frame handed to the MiSTer is entirely "
                    "black. The picture is being lost before the wire, not "
                    "on it.\n");
      else
         MISTER_INFO("%u of %u framebuffer bytes are non-zero, so real "
                   "pixels are going out.\n",
               nonzero, (unsigned)fb_bytes);
   }

   /* Frame numbers must increase monotonically. If the core has run ahead of
    * us, pick up from where it is rather than sending a stale number. The
    * status this reads was fetched at the top of this function. */

   /* Surface the two core-side failures that otherwise look identical to a
    * dead cable. Said once each, because they persist for as long as the
    * cause does and a repeating notification helps nobody. */
   /* Gated on frameEcho: resetSessionState() zeroes the whole status struct at
    * every CmdInit and setFpgaStatus() only runs on a blit ACK, so before the
    * first one lands vramSynced reads 0 for reasons that have nothing to do
    * with sync. Firing here latched the flag as well, so the first blit of
    * every session raised a false alarm *and* consumed the one report a
    * genuine desync would have got. */
   if (      mister_status.frameEcho
         && !mister_status.vramSynced
         && !(mister_st.warned_sync & 1))
   {
      const char *_m = "MiSTer lost sync with the stream";
      mister_st.warned_sync |= 1;
      MISTER_WARN("vramSynced is 0: the core lost sync with the stream. "
                 "Usually bandwidth - raise NLC Near-Lossless Level, or drop "
                 "to a lower resolution. (Suggesting RGB565 here would be no "
                 "help on NLC, which cannot use it.)\n");
      runloop_msg_queue_push(_m, strlen(_m), 2, 300, true, NULL,
            MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_WARNING);
   }
   mister_st.frame++;
   if (mister_status.frame > mister_st.frame)
      mister_st.frame = mister_status.frame + 1;

   /* The sync scanline is chosen by mister_vsync_line() on the frame loop's
    * thread rather than left to the client's automatic frame delay - see the
    * note there for why the automatic figure cannot be trusted from a sender
    * thread. */
   {
      bool         probe = mister_pace_logging();
      retro_time_t t0    = probe ? cpu_features_get_time_usec() : 0;

      gmw_blit(mister_st.frame, mister_st.field, job->vsync_line, 0, 0);

      if (probe)
         mister_st.t_blit += (uint64_t)(cpu_features_get_time_usec() - t0);
   }
   mister_st.stat_blits++;
   /* A blit is client activity as far as the core's idle timeout is concerned,
    * so it defers the next keepalive. Recorded here rather than left for
    * mister_keepalive() to work out from a "did we blit" flag: such a flag can
    * only be maintained inside this function, so anything reading it from
    * outside the frame loop - the modal window timer, say - sees whatever the
    * last frame happened to leave, which is how a whole session was lost. */
   mister_st.last_keepalive = cpu_features_get_time_usec();

   /* Frames are going out but nothing is coming back.
    *
    * This used to be how a CmdSwitchres the core never accepted was inferred,
    * because the client discarded that failure. It is checked directly now, at
    * the point the mode is programmed, and nothing is streamed when it fails -
    * so reaching here means the core accepted the modeline and then stopped
    * answering: it died, it was replaced, or the return path broke.
    *
    * Compared against the echo count when this modeline was programmed, not
    * against zero: frameEcho runs for the life of the session, so a stall on
    * the second or later mode change could never trip a test for "still
    * zero". */
   if (++mister_st.blits_since_mode == 100 && !mister_st.warned_no_echo
         && mister_status.frameEcho == mister_st.echo_at_mode)
   {
      const char *_m = "MiSTer is not acknowledging frames";
      mister_st.warned_no_echo = true;
      MISTER_ERR("100 frames sent since the modeline was accepted and the core "
                 "has echoed none of them, so every frame is being discarded. "
                 "The core accepted the mode and then stopped answering - "
                 "check it is still running and still reachable.\n");
      runloop_msg_queue_push(_m, strlen(_m), 2, 300, true, NULL,
            MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_ERROR);
   }

   /* Liveness. frameEcho is the only signal that distinguishes a live session
    * from one the core has already torn down: the client's own connected flag
    * is set at CmdInit and never cleared by anything arriving (or failing to
    * arrive) on the wire, so it reads true forever against a core that walked
    * away. Once the core's idle timeout fires - which it does whenever a modal
    * window stops the runloop for longer than it allows - every frame after
    * that is being sent into nothing, which is what leaves the picture corrupt
    * until the core is restarted by hand.
    *
    * Only meaningful once the core has echoed at least one frame; before that
    * the no-echo check above is the right diagnosis.
    *
    * This reports and does not act. An earlier version rebuilt the session
    * here, and on a hardware run it did that to a session the core log showed
    * was in perfect step - frame and echo advancing together, sync held - so
    * the cure was worse than the disease. Until a log shows it firing on a
    * genuinely dead session, it stays a warning. The values it saw are printed
    * for exactly that reason: the previous version reported a blit count and
    * nothing else, which is why the false positive could not be explained
    * afterwards.
    *
    * Measured in time rather than blits: 45 blits is under a second at 60 fps
    * but well over two at 20, so a blit count means something different for
    * every core. */
   if (mister_status.frameEcho != mister_st.last_echo)
   {
      mister_st.last_echo      = mister_status.frameEcho;
      mister_st.echo_stall_at  = 0;
      mister_st.warned_echo_stall = false;
   }
   else if (mister_status.frameEcho)
   {
      retro_time_t now = cpu_features_get_time_usec();

      if (!mister_st.echo_stall_at)
         mister_st.echo_stall_at = now;
      else if (  !mister_st.warned_echo_stall
              && (now - mister_st.echo_stall_at) >= MISTER_ECHO_STALL_USEC)
      {
         mister_st.warned_echo_stall = true;
         MISTER_WARN("The core has not acknowledged a frame for %llu ms "
                    "(echo %u, last %u, our frame %u, core frame %u). The "
                    "session may be gone. Not acting on this - if the picture "
                    "is fine, this warning is the fault.\n",
               (unsigned long long)((now - mister_st.echo_stall_at) / 1000),
               mister_status.frameEcho, mister_st.last_echo,
               mister_st.frame, mister_status.frame);
      }
   }

   MISTER_TRACE("blit %u field %u staged %ux%u.\n",
         mister_st.frame, mister_st.field, stage_w, stage_h);

   mister_telemetry();
}

void mister_draw(video_driver_state_t *video_st, const void *data,
      unsigned width, unsigned height, size_t pitch)
{
   settings_t *settings = config_get_ptr();
   enum scaler_pix_fmt in_fmt = SCALER_FMT_ARGB8888;
   unsigned stage_w = 0;
   unsigned stage_h = 0;
   bool menu_on         = false;
   bool menu_composited = false;
   bool hw_frame        = false;
   unsigned menu_scale  = 1;
   double x_scale;
   double y_scale;

   /* read_viewport re-renders the cached frame to repopulate the back buffer,
    * which re-enters video_driver_frame and lands back here. For a core frame
    * the nested call carries no data and stops at the source checks below, but
    * with a composited menu it would take the readback branch again and
    * recurse without bound. There is no re-entrancy guard in
    * video_driver_frame, so it has to live here - and before anything else,
    * so the nested pass disturbs no state and no counter. */
   if (mister_st.in_readback)
      return;

   /* Latched off by mister_disable(). Everything else in this module gates on
    * mister_st.connected, which stays false, so this one check is what makes a
    * misconfigured session cost nothing per frame. */
   if (mister_st.disabled)
      return;

   mister_st.stat_frames++;

   /* Close off a no-frame stretch the sender reported. Stamped for every frame
    * the driver is given, sent or not: this times the core producing frames,
    * not us sending them. */
   {
      retro_time_t _now = cpu_features_get_time_usec();

      if (mister_st.draw_gap_open)
      {
         mister_st.draw_gap_open = false;
         MISTER_WARN("frames again, after %llu ms with none. Nothing was "
                     "asked of this driver for that stretch, so the gap is "
                     "upstream of it - the core, or the frame loop.\n",
               (unsigned long long)((_now - mister_st.last_draw_usec) / 1000));
      }
      mister_st.last_draw_usec = _now;
   }

   /* There is deliberately no automatic teardown-and-reconnect here. One used
    * to sit at this point, driven by the frameEcho stall test in the blit
    * path, and on hardware it rebuilt a session the core log showed was
    * perfectly healthy. Recovery from a genuinely dead session is a manual
    * toggle of Groovy MiSTer Output, which calls mister_retry(); that is a
    * worse experience than an automatic rebuild, but a great deal better than
    * one that fires on a session that was working. */

   if (!mister_st.connected)
   {
      mister_init(settings, video_st->pix_fmt);
      if (!mister_st.connected)
      {
         mister_skip_frame(MISTER_STALL_DISCONNECTED);
         return;
      }
   }

   if (mister_st.mode_pending)
   {
      /* Programming a modeline changes the geometry the sender reads while it
       * fills the blit buffer, so let it finish what it has first. */
      mister_sender_idle();
      mister_apply_mode(video_st);
   }

   /* The pin lives in the video driver's own state, so a driver rebuild - a
    * resolution change, a fullscreen toggle, or any settings change that
    * reinitialises the drivers - silently drops it while hw_pinned still reads
    * true here. The readback would then return the window's viewport instead
    * of the modeline. Re-apply whenever the instance we pinned has gone. */
   if (     mister_st.modeline_active
         && video_st->data
         && video_st->data != mister_st.pinned_data)
   {
      mister_st.hw_pinned   = mister_pin_viewport(video_st,
            mister_st.width, mister_st.height);
      mister_st.pinned_data = video_st->data;
      mister_st.must_clear  = true;
      MISTER_INFO("Video driver was rebuilt; viewport re-pinned to %ux%u.\n",
            mister_st.width, mister_st.height);
   }

   if (!mister_st.modeline_active)
   {
      mister_skip_frame(MISTER_STALL_NO_MODELINE);
      return;
   }

#ifdef HAVE_MENU
   if (menu_state_get_ptr()->flags & MENU_ST_FLAG_ALIVE)
   {
      if (mister_menu_frame)
      {
         /* RGUI hands over its own bitmap, which is far cheaper than reading
          * the composited frame back off the GPU. */
         menu_on = true;
         data    = mister_menu_frame;
         width   = mister_menu_width;
         height  = mister_menu_height;
         pitch   = (size_t)width * sizeof(uint16_t);
      }
      else
      {
         /* Every other menu driver composites on the GPU and never hands us a
          * bitmap, so without this the CRT keeps showing the last core frame -
          * or nothing at all before content is loaded. Read the composited
          * frame back instead, the same way a GPU-rendered core is handled. */
         menu_composited = true;
      }
   }
   else
      mister_menu_frame = NULL;
#endif
   mister_menu_active = menu_on || menu_composited;

   /* Toggling into or out of the menu changes the letterboxing, so anything
    * left in the untouched border would stay on screen. */
   if (mister_menu_prev != mister_menu_active)
   {
      mister_menu_prev     = mister_menu_active;
      mister_st.must_clear = true;
   }

   /* Hardware-rendered cores hand back a sentinel rather than pixels: the
    * frame lives on the GPU and has to be read back. The viewport is pinned to
    * the modeline, so this reads exactly the pixels that go on the wire, and
    * the driver already delivers them as BGR24. */
   if (!menu_on && (menu_composited || data == RETRO_HW_FRAME_BUFFER_VALID))
   {
      struct video_viewport vp;
      bool ok;

      hw_frame = true;

      if (!mister_st.hw_pinned)
      {
         if (!mister_st.warned_hw)
         {
            mister_st.warned_hw = true;
            MISTER_WARN("The \"%s\" video driver cannot pin its "
                       "viewport, so GPU-rendered content is not streamed. "
                       "Use gl, glcore, vulkan, d3d11 or d3d12, or set Menu "
                       "Driver to rgui for the menu.\n",
                  video_driver_get_ident());
         }
         mister_skip_frame(MISTER_STALL_NO_VIEWPORT_PIN);
         return;
      }

      if (     !video_st->current_video
            || !video_st->current_video->read_viewport)
      {
         mister_skip_frame(MISTER_STALL_NO_READ_VIEWPORT);
         return;
      }

      /* read_viewport writes the driver's *live* viewport, not the modeline.
       * The pin is re-applied per mode change but the drivers recompute their
       * viewport far more often than that, so sizing this buffer from the
       * modeline is a heap overflow waiting for the first frame where the two
       * disagree. Size from the viewport itself.
       *
       * Deliberately *not* clamped to the window the way the screenshot task
       * clamps: the drivers write a full viewport-sized image whatever the
       * window is, so shrinking these figures would both under-reserve the
       * buffer and leave the row stride below to disagree with the stride the
       * driver actually wrote - a torn picture on top of the overflow. A
       * viewport may legitimately exceed the window (overscan, integer scale),
       * and mister_place already crops whatever arrives down to the modeline.
       * The ceiling here is a sanity bound, not a fit. */
      memset(&vp, 0, sizeof(vp));
      video_driver_get_viewport_info(&vp);
      if (     vp.width  < 4    || vp.height < 4
            || vp.width  > 16384 || vp.height > 16384)
      {
         mister_skip_frame(MISTER_STALL_NO_FRAME);
         return;
      }

      if (!mister_stage_reserve((size_t)vp.width * vp.height * 3))
      {
         mister_skip_frame(MISTER_STALL_NO_MEMORY);
         return;
      }

      {
         bool         probe = mister_pace_logging();
         retro_time_t t0    = probe ? cpu_features_get_time_usec() : 0;

         mister_st.in_readback = true;
         ok = video_st->current_video->read_viewport(video_st->data,
               mister_stage,
               (runloop_get_flags() & RUNLOOP_FLAG_IDLE) ? true : false);
         mister_st.in_readback = false;

         if (probe)
            mister_st.t_readback += (uint64_t)(cpu_features_get_time_usec() - t0);
      }

      if (!ok)
      {
         mister_skip_frame(MISTER_STALL_READBACK_FAILED);
         return;
      }

      /* read_viewport returns bottom-up - RetroArch's own convention, stated
       * in tasks/task_screenshot.c and implemented with a negative stride in
       * the Vulkan driver. hw_frame carries that down to mister_place, which
       * reverses the rows as it copies rather than in a separate pass. */

      stage_w = vp.width;
      stage_h = vp.height;
   }
   else if (menu_on)
   {
      if (!data || !pitch || width < 4 || height < 4)
      {
         mister_skip_frame(MISTER_STALL_NO_FRAME);
         return;
      }

      /* The menu hands over its own 16-bit bitmap, sized for the framebuffer
       * RGUI drew it for rather than for the modeline. Fill the modeline with
       * it where a whole multiple fits - that is what the high resolution menu
       * option is asking for, and without this it is a small picture in the
       * middle of a large black frame. */
      {
         unsigned sx = width  ? mister_st.width  / width  : 1;
         unsigned sy = height ? mister_st.height / height : 1;

         menu_scale = (sx < sy) ? sx : sy;
         if (!menu_scale)
            menu_scale = 1;
      }

      if (!mister_stage_reserve((size_t)width * menu_scale
                              * height * menu_scale * 3))
      {
         mister_skip_frame(MISTER_STALL_NO_MEMORY);
         return;
      }

      mister_menu_to_bgr24(mister_stage, (const uint16_t*)data,
            width, height, pitch, menu_scale);
      stage_w = width  * menu_scale;
      stage_h = height * menu_scale;
   }
   else
   {
      if (!data || !pitch || width < 4 || height < 4)
      {
         mister_skip_frame(MISTER_STALL_NO_FRAME);
         return;
      }

      /* Two formats never reach the driver as the core declared them, so the
       * conversion video_driver_frame already performed has to be mirrored
       * here rather than trusting pix_fmt:
       *   0RGB1555   is scaled to RGB565 (video_driver.c, scaler_ptr)
       *   XRGB2101010 is narrowed to XRGB8888 unless the driver advertises a
       *               10-bit source, and pix_fmt still reads XRGB2101010 -
       *               which used to drop every frame from such a core. */
      switch (video_st->pix_fmt)
      {
         case RETRO_PIXEL_FORMAT_XRGB8888:
            in_fmt = SCALER_FMT_ARGB8888;
            break;
         case RETRO_PIXEL_FORMAT_RGB565:
         case RETRO_PIXEL_FORMAT_0RGB1555:
            in_fmt = SCALER_FMT_RGB565;
            break;
         case RETRO_PIXEL_FORMAT_XRGB2101010:
            /* Narrowed upstream unless the driver takes a 10-bit source
             * directly, and there is no scaler input format for the packed
             * one, so that case is genuinely unsupported rather than silent. */
            if (video_driver_test_all_flags(GFX_CTX_FLAGS_SCREEN_10BPC_SOURCE))
            {
               mister_skip_frame(MISTER_STALL_PIXEL_FORMAT);
               return;
            }
            in_fmt = SCALER_FMT_ARGB8888;
            break;
         default:
            mister_skip_frame(MISTER_STALL_PIXEL_FORMAT);
            return;
      }

      x_scale = mister_mode.x_scale;
      y_scale = mister_mode.y_scale;

      if ((retroarch_get_rotation() & 1))
      {
         double swap = x_scale;
         x_scale     = y_scale;
         y_scale     = swap;
      }

      if (x_scale <= 0.0)
         x_scale = 1.0;
      if (y_scale <= 0.0)
         y_scale = 1.0;

      if (!mister_stage_frame(data, width, height, pitch, in_fmt,
               x_scale, y_scale, mister_mode.is_stretched ? true : false,
               &stage_w, &stage_h))
      {
         mister_skip_frame(MISTER_STALL_SCALE_FAILED);
         return;
      }
   }

   /* Any change in what is being staged leaves the letterbox border holding
    * the previous image, so clear on the transition rather than only on a
    * mode or menu change. */
   if (     stage_w != mister_st.last_stage_w
         || stage_h != mister_st.last_stage_h
         || hw_frame != mister_st.last_hw_frame
         || retroarch_get_rotation() != mister_st.last_rotation)
   {
      mister_st.last_stage_w  = stage_w;
      mister_st.last_stage_h  = stage_h;
      mister_st.last_hw_frame = hw_frame;
      mister_st.last_rotation = (uint8_t)retroarch_get_rotation();
      mister_st.must_clear    = true;
   }

   /* Hand the frame over. Everything past this point - filling the client's
    * blit buffer, the encode and the sends - happens on the sender thread, so
    * the loop running the game gets on with the next frame. */
   mister_submit(stage_w, stage_h, hw_frame,
         width, height, (unsigned)pitch, (int)video_st->pix_fmt, menu_on);

   /* Stamped on every pass, not only the ones that give up, so the wait in
    * mister_skip_frame() always measures from the last time round the loop.
    * Without this a run of sent frames would leave the stamp stale and the
    * first duplicates after it would go unpaced. */
   mister_skip_pace_usec = cpu_features_get_time_usec();
}

/* Periodic counters, wall-clock spaced so the cost does not scale with frame
 * rate. Cumulative for the session, so the last line before a close is also
 * the session total. */
static void mister_telemetry(void)
{
   retro_time_t now = cpu_features_get_time_usec();
   unsigned i;
   char     drops[256];
   size_t   _len = 0;

   if (      mister_st.last_telemetry
         && (now - mister_st.last_telemetry) < MISTER_TELEMETRY_USEC)
      return;
   mister_st.last_telemetry = now;

   drops[0] = '\0';
   for (i = 1; i < MISTER_STALL_LAST; i++)
   {
      if (!mister_st.stat_dropped[i])
         continue;
      _len += (size_t)snprintf(drops + _len, sizeof(drops) - _len,
            "%s%s=%llu", _len ? ", " : "", mister_stall_str((uint8_t)i),
            (unsigned long long)mister_st.stat_dropped[i]);
      if (_len >= sizeof(drops) - 1)
         break;
   }

   MISTER_PACE("frames=%llu blits=%llu dropped=%llu audio=%llu discarded=%llu "

               "core frame=%u echo=%u vsync=%u eof=%u queue=%u skip=%u%s%s\n",
         (unsigned long long)mister_st.stat_frames,
         (unsigned long long)mister_st.stat_blits,
         /* Frames the newest-wins path let go because the sender still had the
          * previous one. A visible hitch with no wire error looks like this. */
         (unsigned long long)mister_st.stat_dropped[MISTER_STALL_SENDER_BUSY],
         (unsigned long long)mister_st.stat_audio_frames,
         (unsigned long long)(mister_st.stat_audio_drop_overrun
                            + mister_st.stat_audio_drop_nobuf),

         mister_status.frame, mister_status.frameEcho,
         mister_status.vramSynced, mister_status.vramEndFrame,
         mister_status.vramQueue, mister_status.vgaFrameskip,
         drops[0] ? " | not sent: " : "", drops);

   /* Where the rate servo has got to. drift is how many sample frames behind
    * the core's consumption we are; correction is what the resampler is doing
    * about it. Drift growing while the correction sits still means the servo
    * is not running; a correction pinned at the limit means the clocks are
    * further apart than it can cover. */
   if (mister_audio_armed)
   {
      double drift = mister_audio_drift;
      double ratio = mister_audio_ratio;

      MISTER_PACE("  audio clock: drift %+.1f frames (%+.1f ms), "
                  "correction %+.3f%%, %.1f expected per frame\n",
            drift,
            (mister_audio_spf > 1.0 && mister_frame_period_usec > 0)
               ? drift * (double)mister_frame_period_usec
                       / (mister_audio_spf * 1000.0) : 0.0,
            (ratio - 1.0) * 100.0,
            mister_audio_spf);
   }

   /* Mean microseconds per blit in each segment, so a frame rate that is not
    * keeping up can be attributed rather than guessed at. readback is the GPU
    * copy and its CPU conversion, stage is the copy into the blit buffer, and
    * blit is the codec plus the sends. Anything left over between these and
    * the frame period is the rest of RetroArch, including the core itself. */
   if (mister_st.t_samples)
   {
      MISTER_PACE("  per blit: readback %llu us (gpu wait %llu, convert %llu), "
                  "stage %llu us, encode+send %llu us (mean of %u)\n",
            (unsigned long long)(mister_st.t_readback / mister_st.t_samples),
            (unsigned long long)(mister_st.t_map      / mister_st.t_samples),
            (unsigned long long)(mister_st.t_convert  / mister_st.t_samples),
            (unsigned long long)(mister_st.t_stage    / mister_st.t_samples),
            (unsigned long long)(mister_st.t_blit     / mister_st.t_samples),
            mister_st.t_samples);

      mister_st.t_readback = 0;
      mister_st.t_map      = 0;
      mister_st.t_convert  = 0;
      mister_st.t_stage    = 0;
      mister_st.t_blit     = 0;
      mister_st.t_samples  = 0;
   }
}

bool mister_poll_pads(mister_pad_state_t *out)
{
   gmw_fpgaJoyInputs in;

   /* Gated on the subscription, not on the capability set. A core older than
    * v2 negotiates caps 0 but still sends v1 joystick packets, which the
    * client parses; gating here on caps threw away working input entirely. */
   if (!out || !mister_st.connected || !mister_st.inputs_bound)
      return false;

   gmw_pollInputs();
   gmw_getJoyInputs(&in);

   out->buttons[0] = in.joy1;
   out->buttons[1] = in.joy2;

   out->lx[0] = (int8_t)in.joy1LXAnalog;
   out->ly[0] = (int8_t)in.joy1LYAnalog;
   out->rx[0] = (int8_t)in.joy1RXAnalog;
   out->ry[0] = (int8_t)in.joy1RYAnalog;
   out->lx[1] = (int8_t)in.joy2LXAnalog;
   out->ly[1] = (int8_t)in.joy2LYAnalog;
   out->rx[1] = (int8_t)in.joy2RXAnalog;
   out->ry[1] = (int8_t)in.joy2RYAnalog;

   /* Triggers only exist on a v2 analog packet. */
   out->lt[0] = in.joy1LTAnalog;
   out->rt[0] = in.joy1RTAnalog;
   out->lt[1] = in.joy2LTAnalog;
   out->rt[1] = in.joy2RTAnalog;

   /* Sticks are all-zero until the core is sending analog packets; treating
    * that as "centred" would fight the d-pad. Latched rather than recomputed,
    * because a genuinely centred stick reads as all-zero too and would
    * otherwise switch the axes back off mid-session, freezing them at their
    * last off-centre value. */
   if (        in.joy1LXAnalog || in.joy1LYAnalog
            || in.joy1RXAnalog || in.joy1RYAnalog
            || in.joy2LXAnalog || in.joy2LYAnalog
            || in.joy2RXAnalog || in.joy2RYAnalog)
      mister_st.saw_analog = true;
   out->analog = mister_st.saw_analog;

   out->rumble = (mister_st.input_caps & GMW_CAP_RUMBLE) ? true : false;
   return true;
}

bool mister_readback_timing(void)
{
   return mister_pace_logging();
}

void mister_readback_split(unsigned map_usec, unsigned convert_usec)
{
   mister_st.t_map     += map_usec;
   mister_st.t_convert += convert_usec;
}

bool mister_set_rumble(unsigned player, uint8_t strong, uint8_t weak)
{
   if (!mister_st.connected)
      return false;
   if (!(mister_st.input_caps & GMW_CAP_RUMBLE))
      return false;
   if (player > 1)
      return false;

   gmw_send_rumble((uint8_t)player, strong, weak);
   return true;
}

bool mister_poll_ps2(mister_ps2_state_t *out)
{
   gmw_fpgaPS2Inputs in;

   /* Same reasoning as mister_poll_pads: the subscription is what matters,
    * not the negotiated capability set. */
   if (!out || !mister_st.connected || !mister_st.inputs_bound)
      return false;

   gmw_pollInputs();
   gmw_getPS2Inputs(&in);

   memcpy(out->keys, in.ps2Keys, sizeof(out->keys));
   out->mouse_buttons = in.ps2Mouse;

   /* The PS/2 mouse packet carries 9-bit signed deltas: the sign bits live in
    * the button byte (bit 4 for X, bit 5 for Y) and the magnitude in its own
    * byte, so a plain cast would lose every leftward or upward movement. */
   out->mouse_x = (in.ps2Mouse & 0x10)
      ? (int)in.ps2MouseX - 256 : (int)in.ps2MouseX;
   out->mouse_y = (in.ps2Mouse & 0x20)
      ? (int)in.ps2MouseY - 256 : (int)in.ps2MouseY;
   out->mouse_wheel = (int8_t)in.ps2MouseZ;

   return true;
}

/* Send whatever has been staged. Called once per blit so the cadence matches
 * the video frame, which is what the core expects. */
/* Runs on the sender thread; mister_audio_push() fills the staging buffer from
 * the frame loop, so the two are kept apart by mister_audio_lock. The lock is
 * held only for the copy out, not for the send. */
static void mister_audio_flush(void)
{
   uint16_t       bytes;
   char          *dst;
   size_t         take;
   gmw_fpgaStatus live;

   /* The client's own copy, not the shared mister_status: the same field
    * CmdAudio tests, read on the thread that is about to call it. */
   gmw_getStatus(&live);

   if (mister_audio_lock)
      slock_lock(mister_audio_lock);

   if (!mister_audio_frames)
   {
      if (mister_audio_lock)
         slock_unlock(mister_audio_lock);
      return;
   }

   /* Deliberately not gated on status.audio. CmdAudio checks the live flag
    * itself, so gating here only risks throwing the sound away on a stale
    * copy. The flag is read only to decide what counts as delivered.
    *
    * One packet carries at most the core's audio region. Any tail stays staged
    * for the next flush rather than following straight after: the core points
    * every CmdAudio at the same DDR offset, so a second packet arriving before
    * the FPGA has drained the first would overwrite it. */
   take = mister_audio_frames;
   if (take > MISTER_AUDIO_PACKET_FRAMES)
      take = MISTER_AUDIO_PACKET_FRAMES;

   bytes = (uint16_t)(take * 2 * sizeof(int16_t));
   dst   = gmw_get_pBufferAudio();

   /* Arm before the packet below is counted, not after: arming zeroes the
    * running total, so arming afterwards would discard the frames it had just
    * counted and start a packet behind. Zero drift from here holds the core's
    * FIFO at whatever its one-shot prefill left in it, which is the only level
    * we can know about and the only one that costs no latency. */
   if (live.audio && mister_audio_spf > 1.0 && !mister_audio_armed)
   {
      mister_audio_ref_frame  = live.frame;
      mister_audio_seen_frame = live.frame;
      mister_audio_sent       = 0;
      mister_audio_drift      = 0.0;
      mister_audio_ratio      = 1.0;
      mister_audio_armed      = true;
   }

   if (dst)
   {
      memcpy(dst, mister_audio_buf, bytes);

      /* Only count what the client will actually put on the wire. CmdAudio
       * returns without sending while the core has audio off, which it does
       * until the first blit lands. The servo below would believe it too. */
      if (live.audio)
      {
         mister_st.stat_audio_frames += take;
         mister_audio_sent           += take;
      }
   }
   else
      mister_st.stat_audio_drop_nobuf += take;

   /* Keep the tail, if any, for the next flush. */
   mister_audio_frames -= take;
   if (mister_audio_frames)
      memmove(mister_audio_buf, mister_audio_buf + take * 2,
            mister_audio_frames * 2 * sizeof(int16_t));

   /* Re-measure against the core's displayed-frame counter. Only when that
    * counter has moved: it stands still while video is stalled, and servoing
    * on a frozen clock drags the rate the wrong way. Both terms are running
    * totals, so the measurement is right again once blits resume. */
   if (mister_audio_armed && live.frame != mister_audio_seen_frame)
   {
      /* Unsigned difference, so a counter wrap cannot become a two-billion
       * frame correction. */
      uint32_t elapsed = live.frame - mister_audio_ref_frame;
      double   want    = (double)elapsed * mister_audio_spf;
      double   drift   = want - (double)mister_audio_sent;
      double   ratio;

      mister_audio_seen_frame = live.frame;
      mister_audio_drift      = drift;

      /* drift is already the integral of the rate error, so a proportional
       * term is enough. Two seconds leaves a few milliseconds of steady-state
       * lag against a 0.23% error - well inside the core's FIFO - and is slow
       * enough to ignore a core whose sound is not frame-locked. */
      ratio = 1.0 + drift / (mister_audio_spf * MISTER_AUDIO_SERVO_FRAMES);

      /* Hard limits, so a pause, a rewind or a jumping counter cannot turn
       * this into an audible pitch bend. */
      if (ratio > 1.0 + MISTER_AUDIO_SERVO_LIMIT)
         ratio = 1.0 + MISTER_AUDIO_SERVO_LIMIT;
      else if (ratio < 1.0 - MISTER_AUDIO_SERVO_LIMIT)
         ratio = 1.0 - MISTER_AUDIO_SERVO_LIMIT;

      mister_audio_ratio = ratio;
   }

   /* Released before the send: the copy out is all the frame loop has to wait
    * for, and holding it across a datagram would put network time on the
    * thread running the game. */
   if (mister_audio_lock)
      slock_unlock(mister_audio_lock);

   if (dst)
      gmw_audio(bytes);
}

void mister_audio_push(const float *samples, size_t frames)
{
   if (!samples || !frames)
      return;
   if (!mister_st.connected)
      return;
   if (!config_get_ptr()->bools.video_mister_enable)
      return;

   if (mister_audio_lock)
      slock_lock(mister_audio_lock);

   if (!mister_audio_buf)
   {
      mister_audio_buf = (int16_t*)malloc(MISTER_AUDIO_MAX_BYTES);
      if (!mister_audio_buf)
      {
         if (mister_audio_lock)
            slock_unlock(mister_audio_lock);
         return;
      }
   }

   /* This used to flush here when the buffer was about to overrun. It cannot
    * any more: flushing talks to the client, and the client belongs to the
    * sender thread - and the flush now takes the same lock this function is
    * already holding. Neither would end well.
    *
    * Nothing is lost by dropping it. The sender empties this buffer every time
    * round its loop, including while idle, so it only fills if the sender has
    * been blocked for longer than the buffer holds - about 170 ms of sound. If
    * that happens the truncation below is counted and the session summary says
    * so, which is the honest outcome rather than a hidden one. */

   /* Whatever will not fit is thrown away. Count it: the session summary
    * reporting "0 discarded" while sound went missing is worse than no counter
    * at all. */
   if (frames > MISTER_AUDIO_MAX_FRAMES)
   {
      mister_st.stat_audio_drop_overrun += frames - MISTER_AUDIO_MAX_FRAMES;
      frames = MISTER_AUDIO_MAX_FRAMES;
   }
   if (mister_audio_frames + frames > MISTER_AUDIO_MAX_FRAMES)
   {
      size_t fits = MISTER_AUDIO_MAX_FRAMES - mister_audio_frames;
      mister_st.stat_audio_drop_overrun += frames - fits;
      frames = fits;
   }

   /* The MiSTer takes signed 16-bit stereo whatever the host driver uses, so
    * convert here rather than depending on the host format. */
   if (!frames)
   {
      /* The clamp above took the lot. The resampling branch below would read
       * its scratch buffer before writing it. */
   }
   else if (mister_audio_ratio == 1.0)
   {
      /* Nothing to correct, and the case until the servo has a measurement. */
      convert_float_to_s16(mister_audio_buf + mister_audio_frames * 2,
            samples, frames * 2);
      mister_audio_frames += frames;
   }
   else if (!mister_audio_scratch
         && !(mister_audio_scratch = (int16_t*)malloc(MISTER_AUDIO_MAX_BYTES)))
   {
      /* No scratch, no correction: uncorrected is a slow drift, dropping the
       * samples is silence. */
      convert_float_to_s16(mister_audio_buf + mister_audio_frames * 2,
            samples, frames * 2);
      mister_audio_frames += frames;
   }
   else
   {
      /* Resample by the servo's ratio. Convert first, so clamping and rounding
       * stay identical to the straight path, then walk the result with a
       * fractional step, interpolating between neighbours.
       *
       * step is input frames per output frame: above 1:1 the walk occasionally
       * emits two samples from one interval, below it occasionally skips one.
       * mister_audio_hold and mister_audio_phase carry the position across
       * calls, so the interpolation does not restart every buffer. */
      const double step = 1.0 / mister_audio_ratio;
      double       phase = mister_audio_phase;
      int16_t      h0    = mister_audio_hold[0];
      int16_t      h1    = mister_audio_hold[1];
      int16_t     *out   = mister_audio_buf + mister_audio_frames * 2;
      size_t       room  = MISTER_AUDIO_MAX_FRAMES - mister_audio_frames;
      size_t       i     = 0;
      size_t       n     = 0;

      convert_float_to_s16(mister_audio_scratch, samples, frames * 2);

      /* First buffer through here has no previous frame to interpolate from,
       * and a zeroed hold would emit a sample of silence. Start on the
       * input. */
      if (!mister_audio_hold_valid)
      {
         h0    = mister_audio_scratch[0];
         h1    = mister_audio_scratch[1];
         phase = 0.0;
         mister_audio_hold_valid = true;
      }

      while (i < frames)
      {
         if (phase < 1.0)
         {
            const int16_t *in = mister_audio_scratch + i * 2;

            if (n >= room)
               break;

            out[n * 2]     = (int16_t)(h0 + (in[0] - h0) * phase);
            out[n * 2 + 1] = (int16_t)(h1 + (in[1] - h1) * phase);
            n++;
            phase += step;
         }
         else
         {
            h0     = mister_audio_scratch[i * 2];
            h1     = mister_audio_scratch[i * 2 + 1];
            phase -= 1.0;
            i++;
         }
      }

      /* Ran out of staging room mid-buffer; count the rest as lost, same as
       * the straight path's clamp. */
      if (i < frames)
         mister_st.stat_audio_drop_overrun += frames - i;

      mister_audio_phase   = phase;
      mister_audio_hold[0] = h0;
      mister_audio_hold[1] = h1;
      mister_audio_frames += n;
   }

   {
      bool wake = mister_audio_frames >= mister_audio_watermark;

      if (mister_audio_lock)
         slock_unlock(mister_audio_lock);

      /* This used to flush here, which was wrong twice over - the flush talks
       * to the client, which belongs to the sender thread, and it takes the
       * lock this function was holding. Waking the sender does the same job
       * from the right side of both. Without it the buffer only empties on the
       * sender's own cadence, and a hardware run lost a tenth of the audio
       * that way. */
      if (wake && mister_send_lock)
      {
         slock_lock(mister_send_lock);
         scond_signal(mister_send_ready);
         slock_unlock(mister_send_lock);
      }
   }
}

/* Called once per frame loop iteration. Since the sender thread took over the
 * client, there is nothing here that talks to it: draining acknowledgements,
 * flushing audio and waiting on the raster all happen over there, next to the
 * blit they belong with. What is left is the pacing hand-off, and under MiSTer
 * pacing that already happened inside mister_submit() - which blocks until the
 * sender is free, so the raster reaches the core through the queue.
 *
 * Kept as a call site rather than deleted: it is where the frame loop would
 * pick up anything that has to happen once per iteration on this thread. */
void mister_sync(void)
{
}

/* One iteration's worth of client housekeeping, for when the frame loop is not
 * running - a modal window on Windows owns the thread for as long as a menu is
 * open or the window is being dragged, and runloop_iterate does not complete
 * until it lets go.
 *
 * Both halves matter. The keepalive stops the core dropping the session on its
 * idle timeout; draining the ACKs stops the send completion queue filling,
 * which the client asks callers to keep doing through long pauses (see
 * CmdSendKeepAlive) and which otherwise makes later sends fail silently.
 *
 * Returns true if the tick did anything, so a caller can report whether it ran
 * at all. */
bool mister_idle_tick(void)
{
   /* Nothing to do here any more, and deliberately so.
    *
    * This existed because a modal window stops the frame loop, and with it the
    * keepalive, until the user closes the menu or lets go of the title bar. The
    * sender thread is not on the message pump, so it keeps running through all
    * of that and sends the keepalive itself on its idle timer. Talking to the
    * client from the window procedure would now be a second thread doing it,
    * which is the one thing the sender's ownership rule forbids.
    *
    * Kept so the window code can still count ticks and say in the log that the
    * modal loop was entered and left, which is what made this diagnosable. */
   return mister_st.connected;
}

static void mister_keepalive(void)
{
   retro_time_t now;

   if (!mister_st.connected)
      return;

   /* Telemetry has to run from here as well as from the blit path. When
    * nothing is being sent - which is exactly the situation worth reporting -
    * the blit path is never reached, and a counter that only prints while
    * things are working answers no question anyone has. */
   mister_telemetry();

   now = cpu_features_get_time_usec();

   /* The frame loop has stopped handing us anything. This runs on the sender
    * thread, so it is the only place that can notice. Once per stretch;
    * mister_draw() reports the end and the length. */
   if (      mister_st.modeline_active
         && mister_st.last_draw_usec
         && !mister_st.draw_gap_open
         && mister_frame_period_usec > 0
         && (now - mister_st.last_draw_usec)
               > MISTER_NOFRAME_PERIODS * mister_frame_period_usec)
   {
      mister_st.draw_gap_open = true;
      MISTER_WARN("no frame has reached this driver for %llu ms. The stream "
                  "is healthy and the sender is idle - whatever has stopped "
                  "is ahead of us.\n",
            (unsigned long long)((now - mister_st.last_draw_usec) / 1000));
   }

   /* Nothing here looks at whether a frame was drawn. Blits stamp
    * last_keepalive themselves, so a busy loop naturally never reaches the
    * send below, and a loop that has stopped - which is the whole reason this
    * function exists - still gets its keepalive on time. */
   if (      mister_st.last_keepalive
         && (now - mister_st.last_keepalive) < MISTER_KEEPALIVE_USEC)
      return;

   mister_st.last_keepalive = now;
   gmw_send_keepalive();
}

void mister_close(void)
{
   /* First, so that everything below - the summary, gmw_close(), freeing the
    * staging buffers - happens with this thread owning the client again. */
   mister_sender_stop();

   if (mister_st.connected)
   {
      unsigned i;

      MISTER_INFO("Session summary: %llu frames seen, %llu blitted, "
                  "%llu audio frames sent.\n",
            (unsigned long long)mister_st.stat_frames,
            (unsigned long long)mister_st.stat_blits,
            (unsigned long long)mister_st.stat_audio_frames);

      /* Named individually: they are three different faults and a single
       * "discarded" number could not say which had happened. */
      if (mister_st.stat_audio_drop_overrun)
         MISTER_WARN("  audio lost, staging buffer overran: %llu frames. The "
                     "sender is not emptying it fast enough.\n",
               (unsigned long long)mister_st.stat_audio_drop_overrun);
      if (mister_st.stat_audio_drop_nobuf)
         MISTER_WARN("  audio lost, the client had no buffer: %llu frames.\n",
               (unsigned long long)mister_st.stat_audio_drop_nobuf);

      for (i = 1; i < MISTER_STALL_LAST; i++)
         if (mister_st.stat_dropped[i])
            MISTER_INFO("  not sent, %s: %llu\n",
                  mister_stall_str((uint8_t)i),
                  (unsigned long long)mister_st.stat_dropped[i]);

      if (!mister_st.stat_blits)
         MISTER_ERR("Not one frame reached the MiSTer this session.\n");

      MISTER_INFO("Closing session.\n");
   }

   /* Unconditional. gmw_close() is what releases the client singleton, the
    * inputs socket and the sticky "inputs already bound" flag, and a run that
    * only ever failed to connect still created all three - gmw_bindInputs and
    * the pre-init setters construct the singleton before gmw_init is even
    * called. Skipping this left the socket open to process exit and pinned the
    * old address, so changing MiSTer Address did nothing to the input path
    * until RetroArch was restarted. Safe with no session: it null-checks and
    * its video teardown is idempotent. */
   gmw_close();

   if (mister_scaler)
   {
      scaler_ctx_gen_reset(mister_scaler);
      free(mister_scaler);
      mister_scaler = NULL;
   }

   /* The whole staging ring. Safe here because mister_sender_stop() above has
    * joined the thread, so nothing is reading any of it. mister_stage is only
    * an alias into this array, so it is cleared rather than freed separately. */
   {
      int i;
      for (i = 0; i < MISTER_STAGE_BUFS; i++)
      {
         free(mister_stage_buf[i]);
         mister_stage_buf[i]     = NULL;
         mister_stage_bufsize[i] = 0;
      }
   }
   mister_stage       = NULL;
   mister_stage_size  = 0;
   mister_stage_write = 0;

   memset(&mister_job, 0, sizeof(mister_job));

   free(mister_audio_buf);
   free(mister_audio_scratch);
   mister_audio_buf     = NULL;
   mister_audio_scratch = NULL;
   mister_audio_frames  = 0;

   /* The servo measures against the core's frame counter, which a new session
    * restarts, so none of it may be carried across one. */
   mister_audio_armed      = false;
   mister_audio_hold_valid = false;
   mister_audio_ratio      = 1.0;
   mister_audio_phase      = 0.0;
   mister_audio_drift      = 0.0;
   mister_audio_sent       = 0;
   mister_audio_spf        = 0.0;
   mister_audio_watermark  = MISTER_AUDIO_PACKET_FRAMES;

   mister_menu_frame = NULL;
   mister_menu_prev  = false;

   mister_frame_period_usec = 0;
   mister_frame_end_usec    = 0;

   memset(&mister_st, 0, sizeof(mister_st));
   memset(&mister_mode, 0, sizeof(mister_mode));

   mister_log_close();
}
