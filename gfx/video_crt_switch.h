/* CRT SwitchRes Core
 * Copyright (C) 2018 Alphanu / Ben Templeman.
 *
 * RetroArch - A frontend for libretro.
 *  Copyright (C) 2010-2014 - Hans-Kristian Arntzen
 *  Copyright (C) 2011-2017 - Daniel De Matteis
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

#ifndef __VIDEO_CRT_SWITCH_H__
#define __VIDEO_CRT_SWITCH_H__

#include <stdint.h>

#include <boolean.h>
#include <retro_common_api.h>

RETRO_BEGIN_DECLS

typedef struct videocrt_switch
{
   double p_clock;

   unsigned ra_core_width;
   unsigned ra_core_height;
   unsigned ra_tmp_width;
   unsigned ra_tmp_height;
   unsigned ra_set_core_hz;
   unsigned index;
   unsigned int fb_width;
   unsigned int fb_height;

   float ra_core_hz;
   float sr_core_hz;
   float ra_tmp_core_hz;
   float fly_aspect;
   float fb_ra_core_hz;

   int center_adjust;
   int porch_adjust;
   int vert_adjust;
   int tmp_porch_adjust;
   int tmp_center_adjust;
   int tmp_vert_adjust;
   int rtn;
   int interlace;
   int doublescan;
   int hsync;
   int vsync;

   /* Part of drmModeModeInfo struct from xf86drmMode.h */
   uint32_t clock;
   uint32_t vrefresh;
   uint16_t hdisplay, hsync_start, hsync_end, htotal, hskew;
   uint16_t vdisplay, vsync_start, vsync_end, vtotal, vscan;
   bool sr2_active;
   /* What switchres was last initialised with. The monitor preset, the super
    * width and the display index are all applied once, inside the init block,
    * so a change to any of them means the live switchres is answering for the
    * wrong monitor until it is rebuilt. Remembered here rather than read back
    * from switchres: a switchres.ini beside the config is loaded after
    * sr_set_monitor() and may legitimately change the active monitor, so
    * comparing against what switchres reports would differ forever and rebuild
    * every frame. A copy set from the same inputs converges. */
   unsigned sr2_crt_mode;
   int      sr2_super_width;
   int      sr2_monitor_index;
   bool     sr2_init_state_valid;
   /* Scan mode is a per-resolve option rather than an init-time one, so it does
    * not belong in the rebuild guard above - but a change to it still has to
    * reach the resolver, which the geometry test cannot see on its own. */
   unsigned sr2_scan_mode;
   bool     sr2_scan_mode_valid;
   /* Whether this monitor can scan a progressive 640x480 menu. Answered by
    * asking switchres once and then remembered, because the menu resolves a
    * geometry every frame and each probe appends to switchres's mode list.
    * Cleared whenever switchres is rebuilt on a new monitor. */
   bool     menu_hires_ok;
   bool     menu_hires_valid;
   bool menu_active;
   bool hh_core;

   bool rotated;
   bool tmp_rotated;
   bool kms_ctx;
   bool khr_ctx;
} videocrt_switch_t;

void crt_switch_res_core(
      videocrt_switch_t *p_switch,
      unsigned naitive_width,
      unsigned width,
      unsigned height,
      float hz,
      bool rotated,
      unsigned crt_mode,
      int crt_switch_center_adjust,
      int crt_switch_porch_adjust,
      int monitor_index,
      bool dynamic,
      int super_width,
      bool hires_menu,
      unsigned video_aspect_ratio_idx,
      int crt_switch_vert_adjust);

void crt_destroy_modes(videocrt_switch_t *p_switch);

/**
 * crt_switch_forget_resolved:
 * @param p_switch  the CRT switching state.
 *
 * Forget that the current geometry has already been resolved, so the next frame
 * asks switchres for a modeline again. Used when a stream session restarts
 * under an unchanged core, which would otherwise leave it with no modeline and
 * nothing to send.
 **/
void crt_switch_forget_resolved(videocrt_switch_t *p_switch);

/* switchres monitor preset name for a crt_switch_type value, or NULL when
 * the value selects no preset (Off, or INI where switchres.ini supplies it). */
const char *crt_switch_monitor_preset(unsigned crt_mode);

/* Coarse horizontal frequency bounds for a monitor preset, in Hz. Either may
 * be 0, meaning unbounded on that side - which is what a multi-sync preset
 * reports, since it legitimately covers several ranges. */
void crt_monitor_hfreq_bounds(unsigned crt_mode, double *min_hz, double *max_hz);

/* The monitor switchres actually ended up with, and the super width in force.
 * Requested and active differ whenever a switchres.ini overrides the preset,
 * which is otherwise invisible. Kept here rather than exported as an sr_state
 * because switchres_wrapper.h has no include guard and must stay in one
 * translation unit. */
void crt_switch_monitor_state(char *s, size_t len, int *super_width);

RETRO_END_DECLS

#endif
