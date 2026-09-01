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

#ifndef __GFX_MISTER_H
#define __GFX_MISTER_H

#include <stddef.h>
#include <boolean.h>
#include <retro_common_api.h>

#include "video_driver.h"

/* Groovy MiSTer output.
 *
 * Frames are streamed to a MiSTer FPGA over UDP; the FPGA reprograms its PLL
 * to the modeline switchres picked and scans our pixels out to an analog CRT.
 * The MiSTer is the display, so the host video driver keeps presenting to its
 * own window and this runs alongside it as a second consumer.
 *
 * The client library header pulls in <winsock2.h>, and <windows.h> ahead of it
 * drags in the older Winsock 1.1 declarations that then collide, so it is
 * included only by gfx_mister.c and must not reach this header.
 *
 * Everything here runs on the thread that calls video_driver_frame() and
 * runloop_iterate(), which is the same thread; the client is not thread-safe
 * and that property is deliberate, not incidental.
 */

RETRO_BEGIN_DECLS

/* The modeline switchres resolved, in the terms the MiSTer needs.
 *
 * Deliberately not sr_mode: deps/switchres/switchres_wrapper.h carries no
 * include guard, so pulling it into a second header breaks any translation
 * unit that already included it directly. Copying the eleven fields we use
 * also keeps this module independent of the switchres ABI. */
typedef struct
{
   double   pclock;        /* Hz, as switchres reports it */
   double   vfreq;
   double   x_scale;
   double   y_scale;
   unsigned width;
   unsigned hbegin;
   unsigned hend;
   unsigned htotal;
   unsigned height;
   unsigned vbegin;
   unsigned vend;
   unsigned vtotal;
   bool     interlace;
   bool     is_stretched;
} mister_modeline_t;

/* True once CMD_INIT has been acknowledged. */
bool mister_is_connected(void);

/* True when a modeline is live and the MiSTer's raster is the frame clock.
 * While this holds, the runloop's own frame limiter must stand down. */
bool mister_pacing_active(void);

/* Hand over the modeline switchres resolved for the current content. Stashed
 * and applied on the next frame, because it arrives after the frame that
 * triggered it. */
void mister_set_mode(const mister_modeline_t *mode);

/* Hand over the menu's own framebuffer, which is composited separately from
 * core video. Called by the RGUI driver; NULL clears it. */
void mister_set_menu_buffer(void *frame, unsigned width, unsigned height);

/* Convert and stream one frame. Opens the session lazily on first use. */
void mister_draw(video_driver_state_t *video_st, const void *data,
      unsigned width, unsigned height, size_t pitch);

/* Hand over emulated audio: stereo float, as the audio path produces it just
 * before it is converted for the host driver. Staged and sent once per video
 * frame, because the client is not thread-safe and this is called from the
 * audio path rather than the frame path. No-op unless the core reports audio
 * enabled on its own side. */
void mister_audio_push(const float *samples, size_t frames);

/*
 * MiSTer-side inputs
 *
 * Controllers plugged into the MiSTer are streamed back to us on a second
 * socket, so a cabinet needs no host-side controller. This is a convenience,
 * not a latency win: it adds a network hop compared with a stick plugged into
 * the PC.
 */

/* One poll's worth of pad state, in wire terms. Socket-free so the joypad
 * driver can consume it without pulling in the client header. */
typedef struct
{
   /* Generic Button 1..12 plus d-pad, per player. Bit layout is the wire's:
    * 0-3 right/left/down/up, 4-15 buttons 1-12. */
   uint32_t buttons[2];

   int8_t   lx[2];
   int8_t   ly[2];
   int8_t   rx[2];
   int8_t   ry[2];
   uint8_t  lt[2];   /* analog triggers; inputs v2 sessions only */
   uint8_t  rt[2];

   bool     analog;  /* sticks/triggers are streaming (OSD Joysticks = Analog) */
   bool     rumble;  /* rumble capability was NEGOTIATED, not merely requested */
} mister_pad_state_t;

/* Drain the inputs socket and fill in the current pad state. Returns false
 * when no session is live, in which case out is untouched. */
bool mister_poll_pads(mister_pad_state_t *out);

/* Set a pad's motors. Send on state change only: the core repeats the last
 * value until replaced, and 0,0 stops. No-op unless rumble was negotiated. */
bool mister_set_rumble(unsigned player, uint8_t strong, uint8_t weak);

/**
 * mister_readback_timing:
 *
 * Whether the video driver should time its readback. False unless pace logging
 * is on, so the clock is never read during ordinary play.
 **/
bool mister_readback_timing(void);

/**
 * mister_readback_split:
 * @param map_usec      time spent blocked mapping the staging texture.
 * @param convert_usec  time spent converting the pixels.
 *
 * Report how a readback divided between waiting for the GPU and CPU work.
 * Called from the video driver, which is the only place the two are separable.
 **/
void mister_readback_split(unsigned map_usec, unsigned convert_usec);

/* Latest keyboard/mouse state from the MiSTer's own PS/2 inputs. keys is a
 * 256-bit array in SDL scancode numbering; bit n set means scancode n is
 * down. Returns false when no session is live. */
typedef struct
{
   uint8_t keys[32];
   uint8_t mouse_buttons;   /* PS/2 byte 0: [yo,xo,ys,xs,1,bm,br,bl] */
   int     mouse_x;
   int     mouse_y;
   int     mouse_wheel;
} mister_ps2_state_t;

bool mister_poll_ps2(mister_ps2_state_t *out);

/* Pace to the CRT raster and drain the send queue. Must be called every
 * runloop iteration while connected, streaming or not: on Windows this is the
 * only thing that drains the RIO completion queue, and a full queue makes
 * sends fail silently in a way that looks exactly like a dead core. */
void mister_sync(void);

/* Hold an idle session open against the core's idle timeout (5s by default).
 * No-op on iterations that already blitted. */
/* The keepalive is no longer public: it talks to the client, and the sender
 * thread is the only caller allowed to do that. A modal window on the host
 * used to need it from outside; the sender covers that case itself now,
 * because it is not on the message pump. */

/**
 * mister_idle_tick:
 *
 * Keepalive plus ACK drain, for callers outside the frame loop - a modal
 * window message timer, for instance. Safe with no session.
 *
 * @return true if a session was live and the tick ran.
 **/
bool mister_idle_tick(void);

/* Tear the session down. Idempotent, and safe with no session ever opened. */
void mister_close(void);

/* Clear the "disabled for this run" latch and the connect backoff, so a
 * user who has just corrected the address or toggled the output back on
 * gets a fresh attempt without restarting RetroArch. */
void mister_retry(void);

/* Record a line in the MiSTer diagnostics log, for the few decisions that are
 * made outside this module but belong in the same file as everything else the
 * user will be asked for. Also goes to RetroArch's log when that is enabled. */
void mister_log_note(const char *fmt, ...);

RETRO_END_DECLS

#endif
