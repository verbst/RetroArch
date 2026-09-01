/* CRT SwitchRes Core
 *  Copyright (C) 2018 Alphanu / Ben Templeman.
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
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <libretro.h>
#include <math.h>

#include <retro_common_api.h>
#include <compat/strl.h>
#include <string/stdstring.h>
#include <features/features_cpu.h>

#include "gfx_display.h"
#include "video_crt_switch.h"
#include "video_display_server.h"
#include "../core_info.h"
#include "../verbosity.h"
#include "../file_path_special.h"
#include "../paths.h"
#include "../runloop.h"

#include "../deps/switchres/switchres_wrapper.h"
static sr_mode srm;

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

#ifdef HAVE_MISTER
#include "gfx_mister.h"
#endif

/* Forward declarations */
static void crt_adjust_sr_ini(videocrt_switch_t *p_switch);

/* Global local variables */
/* Whether switchres owns a live manager. Tracked here rather than read from
 * p_switch->sr2_active because the library's `swr` is a single process-wide
 * pointer, while sr2_active is per-instance, is still false during the window
 * where crt_sr2_init legitimately calls sr_* functions, and is false-but-
 * dangling after teardown. Nothing in switchres_wrapper.c checks `swr` before
 * dereferencing it - 16 of its 19 entry points fault on a null one - and
 * sr_deinit() deletes without nulling, so the caller has to know. */
static bool sr_manager_live      = false;
/* Whether the live switchres was built as a pure calculator. Recorded so a
 * mid-session change of MiSTer output can rebuild it: crt_sr2_init only runs
 * while !sr2_active, so otherwise a real host display would persist. */
static bool sr_manager_calc_only = false;
/* A failed resolve is retried rather than latched off, but not every frame:
 * the attempt is cheap and the log line is not. */
#define CRT_RESOLVE_RETRY_USEC (2 * 1000000)
static retro_time_t sr_last_fail_usec = 0;

/* video_refresh_rate as it stood before any modeline overwrote it, so the
 * value written to the configuration file at exit is the user's own and not
 * whichever core happened to run last. Without this a session that ended on,
 * say, a 56.842 Hz interlaced mode leaves every later launch believing the
 * display runs at 56.842 Hz, which is a rate no part of the next session has
 * anything to do with. Zero means nothing has been overwritten yet. */
static float sr_refresh_rate_saved = 0.0f;
static bool ini_overrides_loaded = false;
static char core_name[NAME_MAX_LENGTH]; /* Same size as library_name on retroarch_data.h */
static char content_dir[DIR_MAX_LENGTH];
static char current_content_name[256];
static char content_name[256];
static char _hSize[12];
static char _hShift[12];
static char _vShift[12];

#if defined(HAVE_VIDEOCORE) /* Need to add video core to SR2 */
#include <interface/vmcs_host/vc_vchi_gencmd.h>
static void crt_rpi_switch(videocrt_switch_t *p_switch,int width, int height, float hz, int xoffset, int native_width);
#endif

/* Whether switchres is only being asked for the arithmetic, with nothing to be
 * applied to this PC's display.
 *
 * True for the MiSTer, which scans the modeline out itself, and true by default
 * otherwise: generating modelines is useful on its own, but programming them
 * into the desktop is not something to do unless asked. On hardware where
 * switchres can genuinely set modes, applying the 320x240 menu modeline would
 * leave the user in front of an out-of-range monitor with no way back. */
/* Whether the user would rather have the core's own refresh rate than its exact
 * line count, when a monitor cannot give both. */
static bool crt_keep_refresh_rate(void)
{
   settings_t *settings = config_get_ptr();
   return settings
       && settings->uints.crt_switch_mode_priority
             == CRT_SWITCH_MODE_KEEP_REFRESH;
}

/* Whether interlaced modelines may be chosen. Defaults to letting switchres
 * decide when the settings are not readable, which is what it did before this
 * option existed. */
static unsigned crt_scan_mode(void)
{
   settings_t *settings = config_get_ptr();
   return settings ? settings->uints.crt_switch_scan_mode
                   : (unsigned)CRT_SCAN_MODE_AUTO;
}

/* Far enough from the requested rate to be worth trading resolution for.
 *
 * tolerance is a fraction, not a percent. One percent is comfortably below
 * anything audible or visible - 59.94 against 60.000 is 0.1% and must not
 * trigger - while still catching the case this exists for, which measured
 * 5.45% on hardware and slowed the emulation by exactly that much. */
static bool crt_refresh_differs(float granted, float wanted, float tolerance)
{
   if (granted <= 1.0f || wanted <= 1.0f || tolerance <= 0.0f)
      return false;

   return fabsf(1.0f - (granted / wanted)) > tolerance;
}

/* How far the granted rate may sit from the core's own before the resolution
 * is given up for it.
 *
 * Two different questions, so two different thresholds. Having asked to keep
 * the refresh rate, the user wants it whenever it is available, so anything
 * past a percent is worth a smaller mode. Having asked to keep the resolution,
 * they have said the opposite - and overriding that is only defensible when
 * the cost is large, because past a few percent the game is running
 * measurably slow and its audio with it. Zero there means never override.
 *
 * Returns a fraction. Zero disables the trade entirely. */
static float crt_refresh_tolerance(void)
{
   settings_t *settings = config_get_ptr();

   if (crt_keep_refresh_rate())
      return 0.01f;

   if (!settings)
      return 0.0f;

   return (float)settings->uints.crt_switch_refresh_tolerance / 100.0f;
}

static bool switchres_calc_only(void)
{
   settings_t *settings = config_get_ptr();

   if (!settings)
      return true;

#ifdef HAVE_MISTER
   if (settings->bools.video_mister_enable)
      return true;
#endif

   return !settings->bools.crt_switch_host_modeswitch;
}

#ifdef HAVE_MISTER
/* switchres explains a refused mode through its own log, and its reasons -
 * "could not find a video mode that meets your specs" above all - are the ones
 * that matter when the CRT stays blank. Routed into the MiSTer log too, because
 * RARCH_ERR alone is discarded unless the user has turned on global logging,
 * which is exactly the state every failing run so far has been in. */
static void crt_sr_log_mister(const char *fmt, ...)
{
   char    line[512];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(line, sizeof(line), fmt, ap);
   va_end(ap);

   RARCH_LOG("[CRT] %s", line);
   if (switchres_calc_only())
      mister_log_note("[switchres] %s", line);
}

static void crt_sr_err_mister(const char *fmt, ...)
{
   char    line[512];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(line, sizeof(line), fmt, ap);
   va_end(ap);

   RARCH_ERR("[CRT] %s", line);
   if (switchres_calc_only())
      mister_log_note("[switchres] %s", line);
}
#endif

void crt_switch_monitor_state(char *s, size_t len, int *super_width)
{
   sr_state state;

   memset(&state, 0, sizeof(state));

   /* Callers reach this before switchres has been initialised: MiSTer output
    * connects from mister_draw(), which video_driver_frame() runs before
    * crt_switch_res_core(), and with CRT SwitchRes off the latter never runs
    * at all. Reporting zeroes is the honest answer; asking switchres would
    * fault. */
   if (sr_manager_live)
      sr_get_state(&state);

   if (s && len)
   {
      /* monitor[] is not guaranteed terminated when switchres never ran. */
      state.monitor[sizeof(state.monitor) - 1] = '\0';
      strlcpy(s, state.monitor, len);
   }
   if (super_width)
      *super_width = state.super_width;
}

const char *crt_switch_monitor_preset(unsigned crt_mode)
{
   switch (crt_mode)
   {
      case CRT_SWITCH_15KHZ:
         return "arcade_15";
      case CRT_SWITCH_31KHZ:
         return "arcade_31";
      case CRT_SWITCH_32_120:
         return "pc_31_120";
      case CRT_SWITCH_ARCADE_15_25_31:
         return "arcade_15_25_31";
      case CRT_SWITCH_ARCADE_15_31:
         return "arcade_15_31";
      case CRT_SWITCH_ARCADE_15_25:
         return "arcade_15_25";
      case CRT_SWITCH_ARCADE_15EX:
         return "arcade_15ex";
      case CRT_SWITCH_ARCADE_25:
         return "arcade_25";
      case CRT_SWITCH_GENERIC_15:
         return "generic_15";
      case CRT_SWITCH_NTSC:
         return "ntsc";
      case CRT_SWITCH_PAL:
         return "pal";
      case CRT_SWITCH_D9800:
         return "d9800";
      case CRT_SWITCH_D9200:
         return "d9200";
      case CRT_SWITCH_K7000:
         return "k7000";
      case CRT_SWITCH_K7131:
         return "k7131";
      case CRT_SWITCH_M3129:
         return "m3129";
      case CRT_SWITCH_M2929:
         return "m2929";
      case CRT_SWITCH_H9110:
         return "h9110";
      case CRT_SWITCH_PSTAR:
         return "pstar";
      case CRT_SWITCH_MS2930:
         return "ms2930";
      case CRT_SWITCH_MS929:
         return "ms929";
      case CRT_SWITCH_R666B:
         return "r666b";
      case CRT_SWITCH_PC_70_120:
         return "pc_70_120";
      case CRT_SWITCH_VESA_480:
         return "vesa_480";
      case CRT_SWITCH_VESA_600:
         return "vesa_600";
      case CRT_SWITCH_VESA_768:
         return "vesa_768";
      case CRT_SWITCH_VESA_1024:
         return "vesa_1024";
      default:
         break;
   }
   return NULL;
}

/* The horizontal frequencies a preset can actually scan.
 *
 * Only the single-range presets have bounds worth stating; a multi-sync one
 * legitimately covers several, so it reports none and nothing is checked. Zero
 * on either side means unbounded there.
 *
 * Deliberately loose. switchres holds the exact ranges (see monitor.cpp) and
 * enforces them when it resolves; these are a coarse sanity net for the case
 * where a mode arrives that its own preset should never have allowed, which is
 * what a stale switchres produced before it was rebuilt on a preset change. */
void crt_monitor_hfreq_bounds(unsigned crt_mode, double *min_hz, double *max_hz)
{
   double lo = 0.0;
   double hi = 0.0;

   switch (crt_mode)
   {
      case CRT_SWITCH_15KHZ:
      case CRT_SWITCH_ARCADE_15EX:
      case CRT_SWITCH_GENERIC_15:
      case CRT_SWITCH_NTSC:
      case CRT_SWITCH_PAL:
      case CRT_SWITCH_K7000:
      case CRT_SWITCH_K7131:
      case CRT_SWITCH_H9110:
         hi = 16700.0;
         break;
      case CRT_SWITCH_31KHZ:
      case CRT_SWITCH_32_120:
         lo = 20000.0;
         break;
      default:
         break;
   }

   if (min_hz)
      *min_hz = lo;
   if (max_hz)
      *max_hz = hi;
}

/* Make the next check see a change, so switchres resolves again even though the
 * core's geometry has not moved.
 *
 * The modeline is resolved once per geometry and then remembered as handled. A
 * MiSTer session that opens afterwards has no modeline of its own and no reason
 * to ask for one - it simply never blits. That is normally invisible because a
 * session opens when a core loads, which changes the geometry; but a core that
 * loads twice, or any session that restarts under an unchanged core, lands in
 * it and stays there. One hardware run sat at 1226 frames and zero blits with
 * no way back.
 *
 * Clearing the remembered geometry rather than the modeline itself keeps this
 * to one idea: what is stored is "this geometry has been dealt with", and after
 * a session change it has not been. */
void crt_switch_forget_resolved(videocrt_switch_t *p_switch)
{
   if (!p_switch)
      return;

   p_switch->ra_tmp_width   = 0;
   p_switch->ra_tmp_height  = 0;
   p_switch->ra_tmp_core_hz = 0.0f;
}

static bool crt_check_for_changes(videocrt_switch_t *p_switch)
{
   if (   (p_switch->ra_core_height != p_switch->ra_tmp_height)
       || (p_switch->ra_core_width  != p_switch->ra_tmp_width)
       || (p_switch->center_adjust  != p_switch->tmp_center_adjust)
       || (p_switch->porch_adjust   != p_switch->tmp_porch_adjust)
       || (p_switch->vert_adjust   != p_switch->tmp_vert_adjust)
       || (p_switch->ra_core_hz     != p_switch->ra_tmp_core_hz)
       || (p_switch->rotated        != p_switch->tmp_rotated))
      return true;
   return false;
}

static void crt_store_temp_changes(videocrt_switch_t *p_switch)
{
   p_switch->ra_tmp_height     = p_switch->ra_core_height;
   p_switch->ra_tmp_width      = p_switch->ra_core_width;
   p_switch->tmp_center_adjust = p_switch->center_adjust;
   p_switch->tmp_porch_adjust  = p_switch->porch_adjust;
   p_switch->ra_tmp_core_hz    = p_switch->ra_core_hz;
   p_switch->tmp_rotated       = p_switch->rotated;
   p_switch->tmp_vert_adjust   = p_switch->vert_adjust;
}

static void crt_aspect_ratio_switch(
      videocrt_switch_t *p_switch,
      unsigned width, unsigned height,
      float srm_width, float srm_height,
      unsigned video_aspect_ratio_idx)
{
   float fly_aspect               = (float)width / (float)height;
   p_switch->fly_aspect           = fly_aspect;
   video_driver_state_t *video_st = video_state_get_ptr();

   /* We only force aspect ratio for the core provided setting */
   if (video_aspect_ratio_idx != ASPECT_RATIO_CORE)
   {
      RARCH_LOG("[CRT] Aspect ratio forced by user: %f.\n", video_st->aspect_ratio);
      return;
   }

   /* Send aspect float to video_driver */
   video_st->aspect_ratio         = fly_aspect;
   RARCH_LOG("[CRT] Setting aspect ratio: %f.\n", fly_aspect);

   RARCH_LOG("[CRT] Setting screen size: %dx%d.\n",
         width, height);
   video_driver_set_output_size(width, height);
   if (video_st->current_video && video_st->current_video->set_viewport)
      video_st->current_video->set_viewport(
            video_st->data, width, height, true, true);

   command_event(CMD_EVENT_VIDEO_APPLY_STATE_CHANGES, NULL);
}

static void crt_switch_set_aspect(
      videocrt_switch_t *p_switch,
      unsigned int width, unsigned int height,
      unsigned int srm_width, unsigned srm_height,
      float srm_xscale, float srm_yscale,
      bool srm_isstretched )
{
   /* Zeroed because the sr_get_state() below is conditional now; state.super_width
    * is read either way. */
   sr_state state;
   unsigned int patched_width  = 0;
   unsigned int patched_height = 0;
   int scaled_width            = 0;
   int scaled_height           = 0;

   memset(&state, 0, sizeof(state));

   /* used to fix aspect should SR not find a resolution */
   if (srm_width == 0)
   {
      video_driver_get_output_size(&patched_width, &patched_height);
      srm_xscale               = 1;
      srm_yscale               = 1;
   }
   else
   {
      /* use native values as we will be multiplying by srm scale later. */
      patched_width            = width;
      patched_height           = height;
   }

#if !defined(HAVE_VIDEOCORE)
   /* Reached with switchres torn down: crt_sr2_init() calls sr_deinit() on its
    * failure path and switch_res_crt() then falls straight into this function,
    * as does crt_switch_res_core() unconditionally after it. sr_deinit deletes
    * the manager without nulling the pointer, so this was reading through freed
    * memory whenever display init failed. */
   if (sr_manager_live)
      sr_get_state(&state);

   if ((int)srm_width >= state.super_width && !srm_isstretched)
      RARCH_LOG("[CRT] Super resolution detected. Fractal scaling @ X:%f Y:%f.\n", srm_xscale, srm_yscale);
   else if (srm_isstretched && srm_width > 0 )
      RARCH_LOG("[CRT] Resolution is stretched. Fractal scaling @ X:%f Y:%f.\n", srm_xscale, srm_yscale);
#endif

   scaled_width  = roundf(patched_width  * srm_xscale);
   scaled_height = roundf(patched_height * srm_yscale);

   crt_aspect_ratio_switch(p_switch, scaled_width, scaled_height,
         srm_width, srm_height,
         config_get_ptr()->uints.video_aspect_ratio_idx);
}

#if !defined(HAVE_VIDEOCORE)
static bool crt_sr2_init(videocrt_switch_t *p_switch,
      int monitor_index, unsigned int crt_mode, unsigned int super_width)
{
   char index[10];
   gfx_ctx_ident_t gfxctx;
   char ra_config_path[PATH_MAX_LENGTH];
   char sr_ini_file[PATH_MAX_LENGTH];

   if (monitor_index+1 >= 0 && monitor_index+1 < 10)
      snprintf(index, sizeof(index), "%d", monitor_index);
   else
      strlcpy(index, "0", sizeof(index));

   video_context_driver_get_ident(&gfxctx);

   p_switch->kms_ctx = (gfxctx.ident && strncmp(gfxctx.ident, "kms", 3) == 0);
   p_switch->khr_ctx = (gfxctx.ident && strncmp(gfxctx.ident, "khr_display", 11) == 0);

   RARCH_LOG("[CRT] Video context is: %s.\n", gfxctx.ident);

   /* A live switchres is only built once. If MiSTer output was toggled since
    * then the display kind is now wrong - a real host display where a
    * calculator is wanted, or the reverse - so rebuild rather than run with a
    * display that cannot do the job. */
   if (      p_switch->sr2_active
         && (sr_manager_calc_only != (switchres_calc_only() || p_switch->kms_ctx)))
   {
      RARCH_LOG("[CRT] MiSTer output changed; reinitialising switchres.\n");
      sr_deinit();
      sr_manager_live      = false;
      p_switch->sr2_active = false;
   }

   /* The same for the monitor itself.
    *
    * sr_set_monitor(), the super width option and sr_init_disp() are all inside
    * the init block below, so without this the preset picked when switchres
    * first came up holds for the rest of the session. Choosing a 15 kHz monitor
    * mid-session then changed nothing at all: switchres carried on answering
    * for the multi-sync preset it started with and kept granting 31 kHz
    * modelines, which is exactly the blank CRT the setting exists to avoid. */
   if (      p_switch->sr2_active
         &&  p_switch->sr2_init_state_valid
         && (   p_switch->sr2_crt_mode      != crt_mode
             || p_switch->sr2_super_width   != (int)super_width
             || p_switch->sr2_monitor_index != monitor_index))
   {
      RARCH_LOG("[CRT] Monitor selection changed (preset %u -> %u, super width "
                "%d -> %u, index %d -> %d); reinitialising switchres.\n",
            p_switch->sr2_crt_mode, crt_mode,
            p_switch->sr2_super_width, super_width,
            p_switch->sr2_monitor_index, monitor_index);
      sr_deinit();
      sr_manager_live      = false;
      p_switch->sr2_active = false;
   }

   if (!p_switch->sr2_active)
   {
#ifdef HAVE_MISTER
      void (*logp)(const char *, ...) = &crt_sr_log_mister;
      void (*errp)(const char *, ...) = &crt_sr_err_mister;
#else
      void (*logp)(const char *, ...) = &RARCH_LOG;
      void (*errp)(const char *, ...) = &RARCH_ERR;
#endif
      void (*dbgp)(const char *, ...) = &RARCH_DBG;
      sr_init();
      sr_manager_live = true;
      /* Callbacks first, then the level: set_log_verbose() only installs into
       * the live pointer when the current level already permits it, and the
       * manager's constructor leaves it at SR_INFO - so without raising it here
       * switchres's per-candidate reasoning is routed to log_dummy and lost no
       * matter what RetroArch's own verbosity is. */
      sr_set_log_callback_info(*(void **)(&logp));
      sr_set_log_callback_debug(*(void **)(&dbgp));
      sr_set_log_callback_error(*(void **)(&errp));
      sr_set_log_level(3);

      {
         const char *preset = crt_switch_monitor_preset(crt_mode);

         if (preset)
         {
            sr_set_monitor(preset);
            RARCH_LOG("[CRT] CRT mode: %d - %s.\n", crt_mode, preset);
         }
         else if (crt_mode == CRT_SWITCH_INI)
            RARCH_LOG("[CRT] CRT mode: %d - Selected from ini.\n", crt_mode);
      }

      /* Recorded on the attempt rather than on success, so a switchres that
       * fails to initialise is retried on the next pass - sr2_active stays
       * false and brings us back here - without the guard above reading a
       * change that has already been applied and tearing it down again. */
      p_switch->sr2_crt_mode         = crt_mode;
      p_switch->sr2_super_width      = (int)super_width;
      p_switch->sr2_monitor_index    = monitor_index;
      p_switch->sr2_init_state_valid = true;
      /* A different monitor may answer the menu question differently. */
      p_switch->menu_hires_valid     = false;

      if (super_width > 2)
      {
         char sw[16];
         sr_set_user_mode(super_width, 0, 0);
         snprintf(sw, sizeof(sw), "%d", super_width);
         sr_set_option(SR_OPT_SUPER_WIDTH, sw);
      }

      /* "dummy" makes switchres a pure modeline calculator: no host display is
       * opened and no custom video backend is created, so display_manager::caps()
       * reports CUSTOM_VIDEO_CAPS_ADD and timings are computed from the monitor
       * preset's frequency ranges alone.
       *
       * That is what both the KMS path and MiSTer output want, for the same
       * reason - neither switches the host's mode. It also sidesteps a hard
       * dependency the real path carries: on Windows switchres only has custom
       * video backends for ATI/AMD and PowerStrip, and for any other vendor
       * custom_video::make() returns the *base* object rather than NULL. That
       * stub reports caps 0, which suppresses modeline generation entirely -
       * so on an NVIDIA or Intel GPU the real display path can never produce a
       * modeline at all. */
      sr_manager_calc_only = (p_switch->kms_ctx || switchres_calc_only());

      if (sr_manager_calc_only)
      {
         RARCH_LOG("[CRT] Modeline calculator only (no host mode switch).\n");
         p_switch->rtn = sr_init_disp("dummy", NULL);
      }
      else if (monitor_index + 1 > 0)
      {
         RARCH_LOG("[CRT] Monitor index manual: %s.\n", &index[0]);
         p_switch->rtn = sr_init_disp(index, NULL);
      }
      else
      {
         RARCH_LOG("[CRT] Monitor index auto: %s.\n", "auto");
         p_switch->rtn = sr_init_disp("auto", NULL);
      }

      RARCH_LOG("[CRT] SR rtn %d.\n", p_switch->rtn);

#ifdef HAVE_MISTER
      /* Reported here, not once a modeline exists: when switchres cannot
       * resolve one this is the only place the state is observable at all.
       * switchres accepts an unknown preset name silently and then echoes it
       * back as if it were valid, so comparing requested against active is the
       * only check available. */
      if (switchres_calc_only())
      {
         const char *want = crt_switch_monitor_preset(crt_mode);
         char        active[32];
         int         super = 0;

         crt_switch_monitor_state(active, sizeof(active), &super);
         mister_log_note("switchres %s: display \"%s\", monitor requested "
                         "\"%s\", active \"%s\", super width %d.\n",
               p_switch->rtn >= 0 ? "ready" : "FAILED TO INITIALISE",
               sr_manager_calc_only ? "dummy (calculator)" : "host",
               want ? want : (crt_mode == CRT_SWITCH_INI
                     ? "from switchres.ini" : "off"),
               active, super);
      }
#endif

      if (p_switch->rtn >= 0)
      {
         core_name[0]   = '\0';
         content_dir[0] = '\0';
         /* For Lakka, check a switchres.ini next to user's retroarch.cfg */
         fill_pathname_application_data(ra_config_path, PATH_MAX_LENGTH);
         fill_pathname_join(sr_ini_file,
               ra_config_path, "switchres.ini", sizeof(sr_ini_file));
         if (path_is_valid(sr_ini_file))
         {
            RARCH_LOG("[CRT] Loading switchres.ini override file from \"%s\".\n", sr_ini_file);
            sr_load_ini(sr_ini_file);
         }
      }
   }

   if (p_switch->rtn >= 0)
   {
      if (!p_switch->kms_ctx)
      {
         p_switch->sr2_active = true;
         return true;
      }
      else if (p_switch->kms_ctx)
      {
         p_switch->sr2_active = true;
         RARCH_LOG("[CRT] KMS context detected, keeping SR alive.\n");
         return true;
      }
      else if (p_switch->khr_ctx)
      {
         p_switch->sr2_active = true;
         RARCH_LOG("[CRT] Vulkan context detected, keeping SR alive.\n");
         return true;
      }
   }

   RARCH_ERR("[CRT] Error at init, CRT modeswitching disabled.\n");
   sr_deinit();
   sr_manager_live      = false;
   p_switch->sr2_active = false;

   return false;
}

/* Can this monitor actually show a 640x480 menu?
 *
 * High Resolution Menu is a preference, not a demand. Asking for it on a
 * preset that cannot scan 480 progressive lines used to hand back an
 * interlaced mode - a menu that flickers on every line of text - and left the
 * user to work out why. So the preset is asked first, and the answer decides.
 *
 * Progressive is the whole point of the test. A 15 kHz preset answers a
 * 640x480 request perfectly happily with an *interlaced* mode, because 480
 * sits inside its interlaced line range; it is only unusable as a menu. So a
 * candidate is accepted only when it comes back progressive and at about the
 * rate asked for.
 *
 * switchres has to be up before it can be asked, and the menu geometry is
 * decided before anything else would have started it - hence the init here.
 * It is idempotent once active, and its own rebuild guard handles a monitor
 * changing underneath us. */
static bool crt_menu_hires_available(videocrt_switch_t *p_switch,
      int monitor_index, unsigned crt_mode, unsigned super_width)
{
   sr_mode probe;

   if (!p_switch)
      return false;

   if (p_switch->menu_hires_valid)
      return p_switch->menu_hires_ok;

   if (!crt_sr2_init(p_switch, monitor_index, crt_mode, super_width))
      return false;   /* Not cached: switchres may come up on a later pass. */

   memset(&probe, 0, sizeof(probe));

   p_switch->menu_hires_ok    =
            sr_add_mode(640, 480, 60.0, 0, &probe)
         && probe.vfreq > 1.0
         && !probe.interlace
         && !crt_refresh_differs((float)probe.vfreq, 60.0f, 0.05f);
   p_switch->menu_hires_valid = true;

   RARCH_LOG("[CRT] Menu at 640x480: %s.\n",
         p_switch->menu_hires_ok
            ? "available"
            : "not scannable progressively by this monitor preset, using 320x240");

   return p_switch->menu_hires_ok;
}

static void get_modeline_for_kms(videocrt_switch_t *p_switch, sr_mode* srm)
{
   p_switch->clock       = srm->pclock / 1000;
   p_switch->hdisplay    = srm->width;
   p_switch->hsync_start = srm->hbegin;
   p_switch->hsync_end   = srm->hend;
   p_switch->htotal      = srm->htotal;
   p_switch->vdisplay    = srm->height;
   p_switch->vsync_start = srm->vbegin;
   p_switch->vsync_end   = srm->vend;
   p_switch->vtotal      = srm->vtotal;
   p_switch->vrefresh    = srm->refresh;
   p_switch->hskew       = 0;
   p_switch->vscan       = 0;
   p_switch->interlace   = srm->interlace;
   p_switch->doublescan  = srm->doublescan;
   p_switch->hsync       = srm->hsync;
   p_switch->vsync       = srm->vsync;
}

/* Returns whether switchres actually resolved a modeline. The caller must not
 * record the geometry as "handled" when it did not, or crt_check_for_changes()
 * latches the failure off for the rest of the session and it is never retried. */
static bool switch_res_crt(
      videocrt_switch_t *p_switch,
      unsigned width, unsigned height,
      unsigned crt_mode, unsigned native_width,
      int monitor_index, int super_width)
{
   int w                   = native_width;
   int h                   = height;
   bool got_mode           = false;

   /* Check if SR2 is loaded, if not, load it */
   if (crt_sr2_init(p_switch, monitor_index, crt_mode, super_width))
   {
      int ret;
      int flags = 0;
      int temph = 640;
      int tempw = 480;
      char current_core_name[NAME_MAX_LENGTH];
      char current_content_dir[DIR_MAX_LENGTH];
      double rr              = p_switch->ra_core_hz;
      const char *_core_name = (const char*)runloop_state_get_ptr()->system.info.library_name;



      const char* hSize = (const char*)_hSize;
      const char* hShift = (const char*)_hShift;
      const char* vShift = (const char*)_vShift;

      if (p_switch->rotated)
         flags |= SR_MODE_ROTATED;

      /* Check for core and content changes in case we need
         to make any adjustments */
      if (!_core_name || !*_core_name)
         current_core_name[0] = '\0';
      else
         strlcpy(current_core_name, _core_name, sizeof(current_core_name));

      fill_pathname_parent_dir_name(current_content_dir,
            path_get(RARCH_PATH_CONTENT),
            sizeof(current_content_dir));

      if (     !string_is_equal(core_name,   current_core_name)
            || !string_is_equal(content_dir, current_content_dir)
            || !string_is_equal(current_content_name ,content_name))
      {
         /* A core or content change was detected,
            we update the current values and make adjustments */
         strlcpy(core_name,   current_core_name,   sizeof(core_name));
         strlcpy(content_dir, current_content_dir, sizeof(content_dir));
         strlcpy(content_name, current_content_name, sizeof(current_content_name));
         RARCH_LOG("[CRT] Current running core: %s.\n", core_name);
         crt_adjust_sr_ini(p_switch);
         p_switch->hh_core = false;
      }

      /* This drops the desktop into a throwaway 320x240 or 640x400 mode so the
       * geometry trims can be applied against it. Skipped when the modelines
       * are not for this display: there is nothing to trim here, and switching
       * the desktop to a 15 kHz mode purely to adjust centering for a MiSTer
       * would blank the monitor being worked on. */
      #if defined(_WIN32)
      if (!switchres_calc_only()
         && (p_switch->center_adjust  != p_switch->tmp_center_adjust ||
         p_switch->vert_adjust   != p_switch->tmp_vert_adjust))
      {

         if (w > 320 || h > 240)
         {
            temph = 240;
            tempw = 320;
            RARCH_LOG("[CRT] SR temporary mode for windows geometry adjustment (320x240).\n");
         }else{

            RARCH_LOG("[CRT] SR temporary mode for windows geometry adjustment (640x400).\n");
         }

         ret = sr_add_mode(tempw, temph, rr, flags, &srm);

         if (!ret)
            RARCH_ERR("[CRT] SR failed to add temporary mode for windows geometry adjustment.\n");
         else
         {
            ret = sr_set_mode(srm.id);
            RARCH_LOG("[CRT] SR added temporary mode for windows geometry adjustment.\n");
         }

      }
      #endif

      sr_set_option(SR_OPT_H_SIZE, hSize);
      sr_set_option(SR_OPT_H_SHIFT, hShift);
      sr_set_option(SR_OPT_V_SHIFT, vShift);

      /* Whether an interlaced modeline is an acceptable answer.
       *
       * Written on every resolve, both ways round. switchres enables interlace
       * once when it starts (switchres.cpp, set_interlace(true)) and this
       * option mutates that state permanently, so setting only the "off" case
       * would leave it off for the rest of the session - the setting would
       * appear to be one-way. */
      {
         unsigned scan = crt_scan_mode();

         sr_set_option(SR_OPT_INTERLACE,
               (scan == CRT_SCAN_MODE_PROGRESSIVE) ? "0" : "1");
         p_switch->sr2_scan_mode       = scan;
         p_switch->sr2_scan_mode_valid = true;
      }

      RARCH_DBG("[CRT] %dx%d rotation: %d rotated: %d core rotation:%d\n", w, h, p_switch->rotated, flags & SR_MODE_ROTATED, retroarch_get_rotation());
      ret = sr_add_mode(w, h, rr, flags, &srm);
      if (!ret)
      {
         RARCH_ERR("[CRT] SR failed to add mode for %dx%d@%f.\n", w, h, rr);
#ifdef HAVE_MISTER
         if (config_get_ptr()->bools.video_mister_enable)
            mister_log_note("switchres could not resolve a modeline for "
                            "%dx%d@%.3f Hz. The monitor preset cannot produce "
                            "this geometry, or switchres refused it - its own "
                            "reason is logged just above.\n", w, h, rr);
#endif
      }
      /* Refuse a mode the selected monitor cannot scan.
       *
       * switchres enforces its own preset's ranges when it resolves, so this
       * should never fire. It did: a preset changed mid-session left switchres
       * still answering for the previous monitor, and a 15 kHz tube was handed
       * 31.5 kHz - a blank CRT with the reason buried in a log. That cause is
       * fixed above, in the rebuild guard; this is the net under it, and the
       * point of a monitor setting is that the picture ends up scannable.
       *
       * ret is cleared rather than only got_mode, because the MiSTer branch
       * below is gated on ret and would otherwise still send the mode. The
       * caller widens its fallback ladder to cover the refusal. */
      if (ret)
      {
         double hi_hz = 0.0;

         crt_monitor_hfreq_bounds(crt_mode, NULL, &hi_hz);

         if (hi_hz > 0.0 && srm.hfreq > hi_hz)
         {
            const char *preset = crt_switch_monitor_preset(crt_mode);
            char        _m[160];
            size_t      _l;

            RARCH_ERR("[CRT] %dx%d@%.3f resolved to %.3f kHz, past what the "
                      "\"%s\" preset can scan (%.3f kHz). Refusing it and "
                      "looking for a mode this monitor can display.\n",
                  w, h, rr, srm.hfreq / 1000.0,
                  preset ? preset : "", hi_hz / 1000.0);

            _l = (size_t)snprintf(_m, sizeof(_m),
                  "CRT: %.1f kHz is past what \"%s\" can scan - finding a "
                  "lower mode", srm.hfreq / 1000.0, preset ? preset : "");
            runloop_msg_queue_push(_m, _l, 2, 300, true, NULL,
                  MESSAGE_QUEUE_ICON_DEFAULT, MESSAGE_QUEUE_CATEGORY_WARNING);

            ret = 0;
         }
      }

      got_mode = (ret != 0);

      if (p_switch->kms_ctx)
      {
         get_modeline_for_kms(p_switch, &srm);
         video_driver_set_video_mode(srm.width, srm.height, true);
      }
#ifdef HAVE_MISTER
      /* The MiSTer is the display: it takes the modeline over the wire and
       * reprograms its own PLL, so the host must not switch mode.
       *
       * Gated on ret: srm is a file-static shared with the Win32 geometry
       * adjustment above, so on a failed add it holds either zeroes or the
       * 320x240 temporary mode - and handing either to the MiSTer produces a
       * blank CRT with no indication of why. */
      else if (ret && config_get_ptr()->bools.video_mister_enable)
      {
         mister_modeline_t mode;

         mode.pclock       = (double)srm.pclock;
         mode.vfreq        = srm.vfreq;
         mode.x_scale      = srm.x_scale;
         mode.y_scale      = srm.y_scale;
         mode.width        = (unsigned)srm.width;
         mode.hbegin       = (unsigned)srm.hbegin;
         mode.hend         = (unsigned)srm.hend;
         mode.htotal       = (unsigned)srm.htotal;
         mode.height       = (unsigned)srm.height;
         mode.vbegin       = (unsigned)srm.vbegin;
         mode.vend         = (unsigned)srm.vend;
         mode.vtotal       = (unsigned)srm.vtotal;
         mode.interlace    = srm.interlace    ? true : false;
         mode.is_stretched = srm.is_stretched ? true : false;

         /* Reported rather than refused. switchres accumulates modes across
          * calls, so a previously added, resolution-locked one can win for a
          * later geometry and come back scaled - which is worth seeing. Unlike
          * the reference implementations we do scale the source by the mode's
          * x_scale/y_scale before blitting, so a differing active area is
          * handled rather than fatal. */
         if ((unsigned)srm.width != (unsigned)w || (unsigned)srm.height != (unsigned)h)
            mister_log_note("switchres answered %dx%d for a %dx%d request "
                            "(scale %.4f x %.4f).\n",
                  srm.width, srm.height, w, h, srm.x_scale, srm.y_scale);

         mister_set_mode(&mode);
      }
#endif
      else if (p_switch->khr_ctx)
         RARCH_WARN("[CRT] Vulkan -> Can't modeswitch for now.\n");
      else if (switchres_calc_only())
         /* The modeline was wanted for the arithmetic, not for this display.
          * Report success: the mode resolved, it is simply not ours to apply. */
         ret = 1;
      else
         ret = sr_set_mode(srm.id);
      if (!p_switch->kms_ctx && !ret && !switchres_calc_only())
         RARCH_ERR("[CRT] SR failed to switch mode.\n");
      p_switch->sr_core_hz = (float)srm.vfreq;

      crt_switch_set_aspect(p_switch,
            p_switch->rotated ? h : w,
            p_switch->rotated ? w : h,
            srm.width, srm.height,
            (float)srm.x_scale,
            (float)srm.y_scale,
            srm.is_stretched);
   }
   else
   {
      crt_switch_set_aspect(p_switch,
            width, height,
            width, height,
            1.0f,
            1.0f,
            false);
      video_driver_set_output_size(width , height);
      command_event(CMD_EVENT_VIDEO_APPLY_STATE_CHANGES, NULL);
   }

   return got_mode;
}
#endif

void crt_destroy_modes(videocrt_switch_t *p_switch)
{
   if (p_switch->sr2_active)
   {
      p_switch->sr2_active = false;
      sr_deinit();
      sr_manager_live      = false;
   }

   /* Hand the refresh rate back before the configuration is written out. The
    * modelines are per core and per game; the setting they were published
    * through is global and permanent, so without this the last mode of the
    * session becomes the starting assumption for every session after it. */
   if (sr_refresh_rate_saved != 0.0f)
   {
      float hz              = sr_refresh_rate_saved;
      sr_refresh_rate_saved = 0.0f;
      driver_ctl(RARCH_DRIVER_CTL_SET_REFRESH_RATE, &hz);
   }
}

void crt_switch_res_core(
      videocrt_switch_t *p_switch,
      unsigned native_width, unsigned width, unsigned height,
      float hz, bool rotated, unsigned crt_mode,
      int crt_switch_center_adjust,
      int crt_switch_porch_adjust,
      int monitor_index, bool dynamic,
      int super_width, bool hires_menu,
      unsigned video_aspect_ratio_idx,
      int crt_switch_vert_adjust)
{


   if (height <= 4)
   {
      hz              = 60;
      if (      hires_menu
            &&  crt_menu_hires_available(p_switch, monitor_index - 1,
                  crt_mode, super_width))
      {
         native_width = 640;
         height       = 480;
      }
      else
      {
         native_width = 320;
         height       = 240;
      }
      width           = native_width;
   }

   if (height != 4 )
   {
      p_switch->menu_active           = false;
      p_switch->porch_adjust          = crt_switch_porch_adjust;
      p_switch->vert_adjust           = crt_switch_vert_adjust;
      p_switch->ra_core_height        = height;
      p_switch->ra_core_hz            = hz;

      p_switch->ra_core_width         = width;

      p_switch->center_adjust         = crt_switch_center_adjust;
      p_switch->index                 = monitor_index;
      p_switch->rotated               = rotated;

      /* A different monitor is a reason to resolve again, and the geometry
       * test below cannot see one.
       *
       * crt_check_for_changes() compares geometry, the adjusts, the refresh
       * and rotation - nothing about which monitor the modelines are being
       * calculated for. So picking a new preset would sit unhandled until the
       * core happened to change resolution, which for a running game is never.
       * Forgetting the geometry we have already handled makes the test fire,
       * and switch_res_crt() then rebuilds switchres on the new monitor before
       * resolving against it. */
      /* monitor_index - 1 because that is what every switch_res_crt() call
       * below passes down, and so what crt_sr2_init() recorded. Comparing the
       * undecremented value here would differ on every single frame and turn
       * this into a re-resolve on every frame. */
      if (      p_switch->sr2_init_state_valid
            && (   p_switch->sr2_crt_mode      != crt_mode
                || p_switch->sr2_super_width   != super_width
                || p_switch->sr2_monitor_index != monitor_index - 1))
         crt_switch_forget_resolved(p_switch);

      /* Scan mode the same way, for the same reason. Kept separate because it
       * is applied per resolve rather than at init, so it has nothing to do
       * with whether switchres needs rebuilding - only with whether the
       * modeline in hand was resolved under the rule now in force. */
      if (      p_switch->sr2_scan_mode_valid
            &&  p_switch->sr2_scan_mode != crt_scan_mode())
         crt_switch_forget_resolved(p_switch);

      /* Detect resolution change and switch */
      if (crt_check_for_changes(p_switch))
      {
         bool got_mode      = false;
         retro_time_t now   = cpu_features_get_time_usec();

         /* The geometry is unchanged since the last attempt failed - the only
          * reason we are here again is that the failure was deliberately not
          * recorded. Space the retries out. */
         if (      sr_last_fail_usec
               && (now - sr_last_fail_usec) < CRT_RESOLVE_RETRY_USEC)
            return;

         RARCH_LOG("[CRT] Requested resolution: %dx%d@%f, orientation: %s.\n",
                  native_width, height, hz, rotated? "rotated" : "normal");
#if defined(HAVE_VIDEOCORE)
         crt_rpi_switch(p_switch, width, height, hz, 0, native_width);
#else

         snprintf(_hSize, sizeof(_hSize), "%lf", 1+
               ((float)crt_switch_porch_adjust/100.0));
         snprintf(_hShift, sizeof(_hShift), "%d",
               crt_switch_center_adjust);
         snprintf(_vShift, sizeof(_vShift), "%d",
               crt_switch_vert_adjust);
         if (p_switch->hh_core)
         {
            int corrected_width  = 320;
            int corrected_height = 240;
            got_mode = switch_res_crt(p_switch, corrected_width, corrected_height,
                  crt_mode, corrected_width, monitor_index-1, super_width);
            crt_switch_set_aspect(p_switch, native_width, height, native_width,
                  height ,(float)1,(float)1, false);
            video_driver_set_output_size(native_width , height);
         }
         else
         {
            got_mode = switch_res_crt(p_switch, p_switch->ra_core_width,
                  p_switch->ra_core_height, crt_mode,
                  native_width, monitor_index-1, super_width);

            /* Keeping the refresh rate instead of the resolution.
             *
             * switchres fits a picture into a monitor's ranges by scanning it
             * slower when it will not fit at the rate asked for - it can scale
             * a small picture up into a range, but it never scales one down,
             * so a tall frame ends up correct-sized and slow rather than
             * correct-speed and scaled. For a core that is a 5% slowdown, and
             * everything downstream inherits it: the emulation, the audio, and
             * the frame budget.
             *
             * So walk down a few standard CRT line counts and take the first
             * that resolves close to the rate the core wanted. The picture is
             * then scaled into the smaller mode by the render viewport, which
             * costs nothing because the GPU is doing it either way.
             *
             * This runs in both modes, on different thresholds - see
             * crt_refresh_tolerance(). Having asked to keep the refresh rate,
             * a percent is enough. Having asked to keep the resolution, the
             * user has said the opposite and only a large cost overrides them,
             * because a monitor that has to scan the picture slower slows the
             * game and its sound by the same amount; on a 15 kHz preset that
             * was 5.45% on GameCube content. Setting the limit to zero says
             * never override, and turns this off for that mode entirely.
             *
             * If none of the rungs do better, the original stands. This can
             * lose detail but it can never lose the picture. */
            /* Also entered when nothing resolved at all.
             *
             * A refused mode and an outright failure land in the same place:
             * the geometry the core asked for cannot be shown on this monitor.
             * Walking down to one that can is the answer to both, and trying
             * is free - if no rung resolves either, nothing has been lost. */
            if (      p_switch->ra_core_hz > 1.0f
                  && (   !got_mode
                      || crt_refresh_differs(p_switch->sr_core_hz,
                            p_switch->ra_core_hz, crt_refresh_tolerance())))
            {
               static const unsigned ladder[] = { 480, 400, 288, 240 };
               float    wanted = p_switch->ra_core_hz;
               float    was    = p_switch->sr_core_hz;
               unsigned i;
               unsigned chosen = 0;
               double   hi_hz  = 0.0;
               /* Whether we arrived with a mode in hand that is merely the
                * wrong speed, or with none at all. The messages below say
                * different things in the two cases, and `was` only means
                * anything in the first. */
               bool     had_mode = got_mode;

               /* A rung has to be scannable as well as the right speed - on a
                * 15 kHz preset the 400-line rung is neither progressive nor
                * interlaced range, so it must not be taken just because
                * switchres offered something at the right refresh. */
               crt_monitor_hfreq_bounds(crt_mode, NULL, &hi_hz);

               /* Resolve each candidate without applying it.
                *
                * sr_add_mode computes a modeline and hands it back; it is
                * switch_res_crt that goes on to set the aspect, the render
                * size and the modeline the MiSTer is given. Trying rungs with
                * switch_res_crt therefore applied every one of them on the way
                * past, which churned all three during core startup. Probe
                * first, apply once. */
               for (i = 0; i < ARRAY_SIZE(ladder) && !chosen; i++)
               {
                  sr_mode probe;

                  if (ladder[i] >= (unsigned)p_switch->ra_core_height)
                     continue;

                  memset(&probe, 0, sizeof(probe));

                  /* A percent here whatever the mode's own threshold is. That
                   * threshold decides whether to look for a smaller mode at
                   * all; this decides whether a candidate is actually worth
                   * having. Reusing a loose one would let a rung three percent
                   * out satisfy a fallback taken because five percent was too
                   * much, which is most of the cost for none of the point. */
                  if (      sr_add_mode(p_switch->ra_core_width, (int)ladder[i],
                              (double)wanted, 0, &probe)
                        && probe.vfreq > 1.0
                        && !crt_refresh_differs((float)probe.vfreq, wanted, 0.01f)
                        && (hi_hz <= 0.0 || probe.hfreq <= hi_hz))
                     chosen = ladder[i];
               }

               if (chosen)
               {
                  got_mode = switch_res_crt(p_switch, p_switch->ra_core_width,
                        (int)chosen, crt_mode, native_width,
                        monitor_index-1, super_width);

                  if (got_mode && had_mode)
                     RARCH_LOG("[CRT] %dx%d could only be scanned at %.3f Hz; "
                               "using %dx%u at %.3f Hz instead so the core "
                               "keeps its own speed.\n",
                           p_switch->ra_core_width, p_switch->ra_core_height,
                           was, p_switch->ra_core_width, chosen,
                           p_switch->sr_core_hz);
                  else if (got_mode)
                     RARCH_LOG("[CRT] %dx%d could not be shown on this monitor "
                               "at all; using %dx%u at %.3f Hz instead.\n",
                           p_switch->ra_core_width, p_switch->ra_core_height,
                           p_switch->ra_core_width, chosen,
                           p_switch->sr_core_hz);
                  else
                     /* The probe resolved but applying it did not. Put the
                      * original back rather than leave nothing applied. */
                     got_mode = switch_res_crt(p_switch,
                           p_switch->ra_core_width, p_switch->ra_core_height,
                           crt_mode, native_width, monitor_index-1,
                           super_width);
               }
               else if (had_mode)
                  RARCH_LOG("[CRT] No smaller mode reached %.3f Hz either; "
                            "keeping %dx%d at %.3f Hz.\n",
                        wanted, p_switch->ra_core_width,
                        p_switch->ra_core_height, was);
               else
                  RARCH_ERR("[CRT] No mode this monitor can scan was found for "
                            "%dx%d@%.3f Hz, at any of the sizes tried. Check "
                            "the CRT SwitchRes monitor preset matches the "
                            "display.\n",
                        p_switch->ra_core_width, p_switch->ra_core_height,
                        wanted);
            }
         }
#endif
         /* Only publish a refresh rate that actually came from a modeline;
          * sr_core_hz is 0 after a failed resolve.
          *
          * Through driver_ctl rather than video_monitor_set_refresh_rate():
          * that only moves the configuration float, leaving the audio
          * resampler still converting for the rate the previous modeline ran
          * at. The driver_ctl case resets the resampler ratio, re-runs
          * driver_adjust_system_rates() and recomputes the dynamic rate
          * control threshold, which is what a mode change actually needs. */
         if (got_mode)
         {
            float hz             = p_switch->sr_core_hz;
            settings_t *settings = config_get_ptr();

            if (settings && sr_refresh_rate_saved == 0.0f)
               sr_refresh_rate_saved = settings->floats.video_refresh_rate;

            driver_ctl(RARCH_DRIVER_CTL_SET_REFRESH_RATE, &hz);
         }

         /* Recording the geometry as handled is what makes crt_check_for_changes()
          * false on every later frame. Do it only on success, so a resolve that
          * failed is retried rather than latched off for the rest of the run. */
         if (got_mode)
         {
            sr_last_fail_usec = 0;
            crt_store_temp_changes(p_switch);
         }
         else
            sr_last_fail_usec = now;
      }

      if (  (video_aspect_ratio_idx == ASPECT_RATIO_CORE)
         &&  video_driver_get_aspect_ratio() != p_switch->fly_aspect)
      {
         video_driver_state_t *video_st = video_state_get_ptr();
         float fly_aspect               = (float)p_switch->fly_aspect;
         RARCH_LOG("[CRT] Restoring aspect ratio: %f.\n", fly_aspect);
         video_st->aspect_ratio         = fly_aspect;
         command_event(CMD_EVENT_VIDEO_APPLY_STATE_CHANGES, NULL);
      }
   }
}

static char *get_game_name(char *full_path)
{
   unsigned i;
   size_t _len        = strlen(full_path);
   char* rom_filename = full_path + _len;
   char delim         = (char)  path_get(RARCH_PATH_BASENAME)[0];

   for (i = 0; i < _len; i++)
   {
      if (full_path[i] == '/' || full_path[i] =='\\')
      {
         delim = full_path[i];
         break;
      }
   }

   while (0 < _len && (full_path[--_len] != delim));
   if (full_path[_len] == delim)
      rom_filename = full_path + _len + 1;
   return rom_filename;
}

void crt_adjust_sr_ini(videocrt_switch_t *p_switch)
{
   char config_directory[DIR_MAX_LENGTH];
   char switchres_ini_override_file[PATH_MAX_LENGTH];

   char* rom_filename = get_game_name((char*) path_get(RARCH_PATH_BASENAME));

   strlcpy(content_name, rom_filename, sizeof(current_content_name));

   RARCH_LOG("[CRT] Game info \"%s\".\n", rom_filename);

   if (p_switch->sr2_active)
   {
      /* First we reload the base switchres.ini file
         to undo any overrides that might have been
         loaded for another core */
      if (ini_overrides_loaded)
      {
         RARCH_LOG("[CRT] Loading default switchres.ini...\n");
         sr_load_ini((char *)"switchres.ini");
         ini_overrides_loaded = false;
      }

      if (core_name[0] != '\0')
      {
         /* Then we look for config/Core Name/Core Name.switchres.ini
            and load it, overriding any variables it specifies */
         config_directory[0] = '\0';
         fill_pathname_application_special(config_directory,
               sizeof(config_directory),
               APPLICATION_SPECIAL_DIRECTORY_CONFIG);

         fill_pathname_join_special_ext(switchres_ini_override_file,
               config_directory, core_name, core_name,
               ".switchres.ini", sizeof(switchres_ini_override_file));

         if (path_is_valid(switchres_ini_override_file))
         {
            RARCH_LOG("[CRT] Loading switchres.ini core override file from \"%s\".\n", switchres_ini_override_file);
            sr_load_ini(switchres_ini_override_file);
            ini_overrides_loaded = true;
         }

         /* Next up we load directory overrides, if any */
         fill_pathname_join_special_ext(switchres_ini_override_file,
               config_directory, core_name, content_dir,
               ".switchres.ini", sizeof(switchres_ini_override_file));

         if (path_is_valid(switchres_ini_override_file))
         {
            RARCH_LOG("[CRT] Loading switchres.ini content directory override file from \"%s\".\n", switchres_ini_override_file);
            sr_load_ini(switchres_ini_override_file);
            ini_overrides_loaded = true;
         }

         /* Next up we load game overrides, if any */
         fill_pathname_join_special_ext(switchres_ini_override_file,
               config_directory, core_name, content_name,
               ".switchres.ini", sizeof(switchres_ini_override_file));

         if (path_is_valid(switchres_ini_override_file))
         {
            RARCH_LOG("[CRT] Loading switchres.ini game override file from \"%s\".\n", switchres_ini_override_file);
            sr_load_ini(switchres_ini_override_file);
            ini_overrides_loaded = true;
         }
      }
   }
}

/* only used for RPi3 */
#if defined(HAVE_VIDEOCORE)
static void crt_rpi_switch(videocrt_switch_t *p_switch,
      int width, int height, float hz,
      int xoffset, int native_width)
{
   int w;
   char buffer[1024];
   VCHI_INSTANCE_T vchi_instance;
   VCHI_CONNECTION_T *vchi_connection  = NULL;
   static char output1[250]            = {0};
   static char output2[250]            = {0};
   static char set_hdmi[250]           = {0};
   static char set_hdmi_timing[250]    = {0};
   int i                               = 0;
   int hfp                             = 0;
   int hsp                             = 0;
   int hbp                             = 0;
   int vfp                             = 0;
   int vsp                             = 0;
   int vbp                             = 0;
   int hmax                            = 0;
   int vmax                            = 0;
   int pdefault                        = 8;
   int pwidth                          = 0;
   int ip_flag                         = 0;
   float roundw                        = 0.0f;
   float roundh                        = 0.0f;
   float pixel_clock                   = 0.0f;
   int xscale                          = 1;
   int yscale                          = 1;

   if (height > 300)
      height /= 2;

   /* set core refresh from hz */
   video_monitor_set_refresh_rate(hz);

   crt_switch_set_aspect(p_switch, width,
      height, width, height,
      (float)1, (float)1, false);

   w = width;
   while (w < 1920)
      w = w+width;

   if (w > 2000)
      w = w - width;

   width = w;

   crt_aspect_ratio_switch(p_switch, width, height, width, height,
         config_get_ptr()->uints.video_aspect_ratio_idx);

   /* following code is the mode line generator */
   hfp      = ((width * 0.044f) + (width / 112));
   hbp      = ((width * 0.172f) + (width /64));

   hsp      = (width * 0.117f);

   if (height < 241)
      vmax = 261;
   if (height < 241 && hz > 56 && hz < 58)
      vmax = 280;
   if (height < 241 && hz < 55)
      vmax = 313;
   if (height > 250 && height < 260 && hz > 54)
      vmax = 296;
   if (height > 250 && height < 260 && hz > 52 && hz < 54)
      vmax = 285;
   if (height > 250 && height < 260 && hz < 52)
      vmax = 313;
   if (height > 260 && height < 300)
      vmax = 318;

   if (height > 400 && hz > 56)
      vmax = 533;
   if (height > 520 && hz < 57)
      vmax = 580;

   if (height > 300 && hz < 56)
      vmax = 615;
   if (height > 500 && hz < 56)
      vmax = 624;
   if (height > 300)
      pdefault = pdefault * 2;

   vfp = (height + ((vmax - height) / 2) - pdefault) - height;

   if (height < 300)
      vsp = vfp + 3; /* needs to be 3 for progressive */
   if (height > 300)
      vsp = vfp + 6; /* needs to be 6 for interlaced */

   vsp  = 3;
   vbp  = (vmax - height) - vsp - vfp;
   hmax = width + hfp + hsp + hbp;

   if (height < 300)
      pixel_clock = (hmax * vmax * hz);

   if (height > 300)
   {
      pixel_clock = (hmax * vmax * (hz/2)) / 2;
      ip_flag     = 1;
   }

   /* above code is the modeline generator */
   snprintf(set_hdmi_timing, sizeof(set_hdmi_timing),
         "hdmi_timings %d 1 %d %d %d %d 1 %d %d %d 0 0 0 %f %d %f 1 ",
         width, hfp, hsp, hbp, height, vfp,vsp, vbp,
         hz, ip_flag, pixel_clock);

   vcos_init();
   vchi_initialise(&vchi_instance);
   vchi_connect(NULL, 0, vchi_instance);
   vc_vchi_gencmd_init(vchi_instance, &vchi_connection, 1);
   vc_gencmd(buffer, sizeof(buffer), set_hdmi_timing);
   vc_gencmd_stop();
   vchi_disconnect(vchi_instance);
   snprintf(output1,  sizeof(output1),
         "tvservice -e \"DMT 87\" > /dev/null");
   system(output1);
   snprintf(output2,  sizeof(output2),
         "fbset -g %d %d %d %d 24 > /dev/null",
         width, height, width, height);
   system(output2);
   video_driver_reinit(DRIVER_VIDEO_MASK);
}
#endif
