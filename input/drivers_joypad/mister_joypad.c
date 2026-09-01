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

/* Joypad driver for controllers plugged into the MiSTer.
 *
 * The MiSTer streams the state of its own pads back to us on a second socket,
 * so a cabinet needs no host-side controller at all.
 *
 * RetroArch runs exactly one joypad driver, and losing every host controller
 * the moment MiSTer output is switched on would be a poor trade.  So this
 * driver *wraps* the platform's normal one: calls for pads below the MiSTer
 * range are forwarded to it untouched, and the MiSTer's two pads are served on
 * their own indices above it.  Both sets are then live simultaneously, each
 * with its own bindings, and neither knows about the other.
 *
 * The MiSTer pads sit ABOVE the host range rather than below because host
 * drivers announce themselves to the autoconfig system with their own pad
 * indices, which a wrapper cannot renumber.  Leaving the host numbering alone
 * is what keeps existing controller setups working unchanged.
 */

#include <stdint.h>
#include <string.h>

#include <boolean.h>
#include <libretro.h>
#include <retro_inline.h>
#include <string/stdstring.h>

#include "../input_driver.h"

#include "../../configuration.h"
#include "../../gfx/gfx_mister.h"
#include "../../tasks/tasks_internal.h"
#include "../../verbosity.h"

/* The protocol carries two pads. */
#define MISTER_MAX_PADS 2

/* Wire button positions, matching the GM_JOY_* bit layout: 0-3 are the d-pad,
 * 4-15 are Button 1..12.  Generic on purpose - the MiSTer normalises whatever
 * is physically plugged in via its own per-device .map files, so mapping is
 * done once here against positions rather than per controller. */
enum
{
   MISTER_BTN_RIGHT = 0,
   MISTER_BTN_LEFT,
   MISTER_BTN_DOWN,
   MISTER_BTN_UP,
   MISTER_BTN_B1,   /* Cross    / A  */
   MISTER_BTN_B2,   /* Circle   / B  */
   MISTER_BTN_B3,   /* Square   / X  */
   MISTER_BTN_B4,   /* Triangle / Y  */
   MISTER_BTN_B5,   /* L1       / LB */
   MISTER_BTN_B6,   /* R1       / RB */
   MISTER_BTN_B7,   /* Select   / Back  */
   MISTER_BTN_B8,   /* Start    / Start */
   MISTER_BTN_B9,   /* L2       / LT */
   MISTER_BTN_B10,  /* R2       / RT */
   MISTER_BTN_B11,  /* L3       / LS */
   MISTER_BTN_B12,  /* R3       / RS */
   MISTER_BTN_COUNT
};

/* Analog axes, in the order the wire delivers them. */
enum
{
   MISTER_AXIS_LX = 0,
   MISTER_AXIS_LY,
   MISTER_AXIS_RX,
   MISTER_AXIS_RY,
   MISTER_AXIS_COUNT
};

typedef struct
{
   uint32_t buttons;
   int16_t  axes[MISTER_AXIS_COUNT];
   uint8_t  rumble[2];        /* strong/weak most recently asked for  */
   uint8_t  rumble_sent[2];   /* strong/weak most recently put on the wire */
   bool     rumble_valid;     /* rumble_sent means something yet */
   bool     connected;
} mister_pad_t;

static mister_pad_t mister_pads[MISTER_MAX_PADS];

/* Rumble Gain, as a percentage. RetroArch hands it to the driver rather than
 * applying it, once, whenever the setting changes. */
static unsigned mister_rumble_gain = 100;

static void mister_rumble_flush(mister_pad_t *pad, unsigned player);

/* The host driver we forward to, and where our own pads start. */
static const input_device_driver_t *mister_host_joypad;
static unsigned mister_pad_base;

static INLINE bool mister_is_ours(unsigned port)
{
   return port >= mister_pad_base
       && port <  mister_pad_base + MISTER_MAX_PADS;
}

static INLINE unsigned mister_local(unsigned port)
{
   return port - mister_pad_base;
}

static const char *mister_joypad_name(unsigned port)
{
   if (mister_is_ours(port))
      return "MiSTer Pad";
   if (mister_host_joypad && mister_host_joypad->name)
      return mister_host_joypad->name(port);
   return NULL;
}

/* Try one named driver.  Deliberately does not consult
 * input_joypad_init_driver(), which would find this driver again and recurse. */
static const input_device_driver_t *mister_try_host(const char *ident,
      void *data)
{
   unsigned i;

   if (string_is_empty(ident))
      return NULL;

   for (i = 0; joypad_drivers[i]; i++)
   {
      if (joypad_drivers[i] == &mister_joypad)
         continue;
      if (!string_is_equal(ident, joypad_drivers[i]->ident))
         continue;
      if (!joypad_drivers[i]->init)
         return NULL;
      return joypad_drivers[i]->init(data) ? joypad_drivers[i] : NULL;
   }
   return NULL;
}

/* Resolve the driver to wrap.  Named drivers are tried in order of authority:
 * the MiSTer-specific override, then whatever the user actually configured as
 * their joypad driver.  Only if neither names a driver that starts does this
 * fall back to probing.
 *
 * The probe is a last resort on purpose: init() is not free of side effects,
 * so walking the whole table initialises drivers that are then abandoned.
 * Doing that first was taking ownership of the user's configured driver -
 * a run with input_joypad_driver = "xinput" ended up wrapping dinput, and
 * RetroArch then reported xinput as having failed to initialise. */
static const input_device_driver_t *mister_find_host(const char *ident,
      void *data)
{
   settings_t                     *settings = config_get_ptr();
   const input_device_driver_t    *found;
   unsigned i;

   if ((found = mister_try_host(ident, data)))
      return found;

   if (!string_is_empty(ident))
      RARCH_WARN("[MiSTer] Wrapped Controller Driver \"%s\" is not available; "
                 "falling back.\n", ident);

   if (settings)
      if ((found = mister_try_host(settings->arrays.input_joypad_driver, data)))
         return found;

   for (i = 0; joypad_drivers[i]; i++)
   {
      if (joypad_drivers[i] == &mister_joypad)
         continue;
      if (!joypad_drivers[i]->init)
         continue;
      if (joypad_drivers[i]->init(data))
         return joypad_drivers[i];
   }
   return NULL;
}

static void mister_pad_connect(unsigned port)
{
   /* Announced so the autoconfig system can bind it.  The name matches the
    * built-in profile in input_autodetect_builtin.c, so the pads arrive
    * already mapped and remain remappable like any other controller. */
   input_autoconfigure_connect("MiSTer Pad", NULL, NULL,
         mister_joypad.ident, port, 0, 0);
   mister_pads[mister_local(port)].connected = true;
}

static void *mister_joypad_init(void *data)
{
   settings_t *settings = config_get_ptr();
   unsigned i;

   memset(mister_pads, 0, sizeof(mister_pads));

   mister_pad_base    = settings->uints.mister_joypad_port_base;
   mister_host_joypad = mister_find_host(
         settings->arrays.mister_joypad_host_driver, data);

   /* Through the MiSTer sink rather than RARCH_LOG: whether this driver is
    * running at all is the first thing to establish when the MiSTer's pads do
    * nothing, and RARCH_LOG is discarded unless the user has turned logging on
    * and pointed it at a file. mister.log is written whenever MiSTer output is
    * enabled, which is exactly when this matters. */
   if (mister_host_joypad)
      mister_log_note("Joypad driver active, wrapping \"%s\". MiSTer pads are "
                      "on device indices %u-%u.",
            mister_host_joypad->ident, mister_pad_base,
            mister_pad_base + MISTER_MAX_PADS - 1);
   else
      mister_log_note("Joypad driver active, but no host joypad driver could "
                      "be initialised - only the MiSTer's own pads will be "
                      "available. MiSTer pads are on device indices %u-%u.",
            mister_pad_base, mister_pad_base + MISTER_MAX_PADS - 1);

   /* Which player, if any, is actually pointed at them. Getting this wrong is
    * silent otherwise: the pads enumerate, the bindings look right, and no
    * input arrives because every player is still on its own index. */
   {
      unsigned p;
      for (p = 0; p < settings->uints.input_max_users && p < MAX_USERS; p++)
      {
         unsigned idx = settings->uints.input_joypad_index[p];
         if (idx >= mister_pad_base && idx < mister_pad_base + MISTER_MAX_PADS)
            mister_log_note("  player %u -> MiSTer pad %u (device index %u).",
                  p + 1, idx - mister_pad_base + 1, idx);
      }
   }

   /* The pads are announced whether or not a session is live: they have to be
    * bindable in the menu before the MiSTer is ever reachable. */
   for (i = 0; i < MISTER_MAX_PADS; i++)
      mister_pad_connect(mister_pad_base + i);

   return (void*)-1;
}

static void mister_joypad_destroy(void)
{
   unsigned i;

   for (i = 0; i < MISTER_MAX_PADS; i++)
   {
      if (!mister_pads[i].connected)
         continue;
      /* Stop the motors before the device goes away.  The core also force-
       * stops them on session close, so this covers the case where inputs are
       * switched off with the session still up.  Sent directly rather than
       * through mister_rumble_flush: the cached pair may already read 0,0 from
       * an earlier stop, and this one has to go out regardless. */
      mister_set_rumble(i, 0, 0);
      mister_pads[i].rumble[0]      = 0;
      mister_pads[i].rumble[1]      = 0;
      mister_pads[i].rumble_valid   = false;
      input_autoconfigure_disconnect(mister_pad_base + i, "MiSTer Pad");
   }

   memset(mister_pads, 0, sizeof(mister_pads));

   if (mister_host_joypad && mister_host_joypad->destroy)
      mister_host_joypad->destroy();
   mister_host_joypad = NULL;
}

static bool mister_joypad_query_pad(unsigned port)
{
   if (mister_is_ours(port))
      return true;
   if (mister_host_joypad && mister_host_joypad->query_pad)
      return mister_host_joypad->query_pad(port);
   return false;
}

static int32_t mister_pad_button(unsigned port, uint16_t joykey)
{
   const mister_pad_t *pad = &mister_pads[mister_local(port)];

   if (joykey >= MISTER_BTN_COUNT)
      return 0;
   return (pad->buttons & (1u << joykey)) ? 1 : 0;
}

static int32_t mister_joypad_button(unsigned port, uint16_t joykey)
{
   if (mister_is_ours(port))
      return mister_pad_button(port, joykey);
   if (mister_host_joypad && mister_host_joypad->button)
      return mister_host_joypad->button(port, joykey);
   return 0;
}

static int16_t mister_pad_axis(unsigned port, uint32_t joyaxis)
{
   const mister_pad_t *pad = &mister_pads[mister_local(port)];

   if (AXIS_NEG_GET(joyaxis) < MISTER_AXIS_COUNT)
   {
      int16_t v = pad->axes[AXIS_NEG_GET(joyaxis)];
      return (v < 0) ? v : 0;
   }
   if (AXIS_POS_GET(joyaxis) < MISTER_AXIS_COUNT)
   {
      int16_t v = pad->axes[AXIS_POS_GET(joyaxis)];
      return (v > 0) ? v : 0;
   }
   return 0;
}

static int16_t mister_joypad_axis(unsigned port, uint32_t joyaxis)
{
   if (mister_is_ours(port))
      return mister_pad_axis(port, joyaxis);
   if (mister_host_joypad && mister_host_joypad->axis)
      return mister_host_joypad->axis(port, joyaxis);
   return 0;
}

static int16_t mister_joypad_state(
      rarch_joypad_info_t *joypad_info,
      const struct retro_keybind *binds,
      unsigned port)
{
   unsigned i;
   int16_t  ret      = 0;
   uint16_t port_idx = joypad_info->joy_idx;

   if (!mister_is_ours(port_idx))
   {
      if (mister_host_joypad && mister_host_joypad->state)
         return mister_host_joypad->state(joypad_info, binds, port);
      return 0;
   }

   for (i = 0; i < RARCH_FIRST_CUSTOM_BIND; i++)
   {
      /* Auto-binds are per joypad, not per user. */
      const uint64_t joykey  = (binds[i].joykey != NO_BTN)
         ? binds[i].joykey  : joypad_info->auto_binds[i].joykey;
      const uint32_t joyaxis = (binds[i].joyaxis != AXIS_NONE)
         ? binds[i].joyaxis : joypad_info->auto_binds[i].joyaxis;

      if (     (uint16_t)joykey != NO_BTN
            && mister_pad_button(port_idx, (uint16_t)joykey))
         ret |= (1 << i);
      else if (joyaxis != AXIS_NONE
            && ((float)abs(mister_pad_axis(port_idx, joyaxis)) / 0x8000)
                  > joypad_info->axis_threshold)
         ret |= (1 << i);
   }

   return ret;
}

static void mister_joypad_get_buttons(unsigned port, input_bits_t *state)
{
   if (mister_is_ours(port))
   {
      BIT256_CLEAR_ALL_PTR(state);
      return;
   }
   if (mister_host_joypad && mister_host_joypad->get_buttons)
      mister_host_joypad->get_buttons(port, state);
   else
      BIT256_CLEAR_ALL_PTR(state);
}

static void mister_joypad_poll(void)
{
   mister_pad_state_t snap;
   unsigned i;

   if (mister_host_joypad && mister_host_joypad->poll)
      mister_host_joypad->poll();

   if (!mister_poll_pads(&snap))
      return;

   for (i = 0; i < MISTER_MAX_PADS; i++)
   {
      mister_pad_t *pad = &mister_pads[i];

      pad->buttons = snap.buttons[i];

      /* Sticks only stream when the MiSTer's own menu has Joysticks = Analog.
       * Feeding a permanently centred stick would fight the d-pad, so leave
       * the axes alone until an analog packet has actually been seen. */
      if (!snap.analog)
         continue;

      pad->axes[MISTER_AXIS_LX] = (int16_t)(snap.lx[i] * 256);
      pad->axes[MISTER_AXIS_LY] = (int16_t)(snap.ly[i] * 256);
      pad->axes[MISTER_AXIS_RX] = (int16_t)(snap.rx[i] * 256);
      pad->axes[MISTER_AXIS_RY] = (int16_t)(snap.ry[i] * 256);
   }

   /* Both motors have had their say for this frame by now, so the pair can go
    * out as one value. Doing it here rather than from set_rumble is what stops
    * a two-call stop passing through a still-buzzing intermediate state. */
   for (i = 0; i < MISTER_MAX_PADS; i++)
      mister_rumble_flush(&mister_pads[i], i);
}

/* Put a pad's current pair on the wire, if it has moved since the last send.
 *
 * Called once per poll rather than once per motor, and that ordering is the
 * whole point. The MiSTer collapses the pair into a single 16-bit value,
 * (strong << 8) | weak, and treats any non-zero as "buzz for five seconds".
 * RetroArch hands rumble over one motor at a time, so a stop arrives as two
 * calls - and sending after each would put (0, weak) on the wire in between,
 * which is non-zero, which re-arms the motor for another five seconds. Waiting
 * until both calls have landed means a stop goes out as a single (0, 0).
 *
 * Stops are sent three times. UDP does not promise delivery and this link has
 * been measured losing packets; a lost (0,0) leaves a motor running for the
 * rest of that five seconds. The core ignores a value it already holds, so the
 * repeats cost nothing. Non-zero values go once - the game re-drives them. */
static void mister_rumble_flush(mister_pad_t *pad, unsigned player)
{
   const int stop_resends = 3;
   int       sends;
   int       i;

   if (      pad->rumble_valid
         && pad->rumble_sent[0] == pad->rumble[0]
         && pad->rumble_sent[1] == pad->rumble[1])
      return;

   sends = (pad->rumble[0] == 0 && pad->rumble[1] == 0) ? stop_resends : 1;

   mister_log_note("rumble: player %u -> strong %u weak %u (was %u/%u), %d "
                   "datagram%s.",
         player + 1, pad->rumble[0], pad->rumble[1],
         pad->rumble_valid ? pad->rumble_sent[0] : 0,
         pad->rumble_valid ? pad->rumble_sent[1] : 0,
         sends, sends == 1 ? "" : "s");

   pad->rumble_sent[0] = pad->rumble[0];
   pad->rumble_sent[1] = pad->rumble[1];
   pad->rumble_valid   = true;

   for (i = 0; i < sends; i++)
      if (!mister_set_rumble(player, pad->rumble[0], pad->rumble[1]))
      {
         mister_log_note("rumble: the send was refused - no session, or the "
                         "core did not negotiate rumble.");
         break;
      }
}

static bool mister_joypad_set_rumble(unsigned port,
      enum retro_rumble_effect effect, uint16_t strength)
{
   mister_pad_t *pad;
   uint8_t       level;
   unsigned      motor;
   bool          changed;

   if (!mister_is_ours(port))
   {
      if (mister_host_joypad && mister_host_joypad->set_rumble)
         return mister_host_joypad->set_rumble(port, effect, strength);
      return false;
   }

   pad = &mister_pads[mister_local(port)];

   /* RetroArch only applies its own software gain when a driver has no
    * set_rumble_gain, and ours has one, so the scaling has to happen here or
    * the setting does nothing at all for these pads. */
   if (mister_rumble_gain != 100)
      strength = (uint16_t)(((unsigned)strength * mister_rumble_gain) / 100);

   level = (uint8_t)(strength >> 8);

   if (effect == RETRO_RUMBLE_STRONG)
      motor = 0;
   else if (effect == RETRO_RUMBLE_WEAK)
      motor = 1;
   else
      return false;

   changed            = (pad->rumble[motor] != level);
   pad->rumble[motor] = level;

   /* Recorded, not sent. mister_joypad_poll() puts it on the wire once both
    * motors have been given their value for this frame - see
    * mister_rumble_flush() for why sending here would keep the motor alive.
    *
    * Only the changes are logged. Cores call this every frame for every motor
    * whether or not anything moved, which at 60 Hz buried the rest of the log
    * under thousands of identical lines. */
   if (changed)
      mister_log_note("rumble: core asked player %u for %s = %u (level %u).",
            mister_local(port) + 1,
            motor ? "weak" : "strong",
            (unsigned)strength, level);

   return true;
}

/* Applied to our own pads in mister_joypad_set_rumble rather than here, because
 * the MiSTer takes an absolute level per motor and has nothing to scale after
 * the fact. Advertising this function at all is what stops RetroArch applying
 * its own software gain, so ignoring the value - which is what this used to do
 * - left the Rumble Gain setting doing nothing whatsoever for MiSTer pads. */
static bool mister_joypad_set_rumble_gain(unsigned port, unsigned gain)
{
   if (mister_is_ours(port))
   {
      mister_rumble_gain = gain;
      return true;
   }

   if (mister_host_joypad && mister_host_joypad->set_rumble_gain)
      return mister_host_joypad->set_rumble_gain(port, gain);

   return false;
}

static bool mister_joypad_set_sensor_state(unsigned port,
      enum retro_sensor_action action, unsigned rate)
{
   if (!mister_is_ours(port) && mister_host_joypad
         && mister_host_joypad->set_sensor_state)
      return mister_host_joypad->set_sensor_state(port, action, rate);
   return false;
}

static bool mister_joypad_get_sensor_input(unsigned port, unsigned id,
      float *value)
{
   if (!mister_is_ours(port) && mister_host_joypad
         && mister_host_joypad->get_sensor_input)
      return mister_host_joypad->get_sensor_input(port, id, value);
   return false;
}

input_device_driver_t mister_joypad = {
   mister_joypad_init,
   mister_joypad_query_pad,
   mister_joypad_destroy,
   mister_joypad_button,
   mister_joypad_state,
   mister_joypad_get_buttons,
   mister_joypad_axis,
   mister_joypad_poll,
   mister_joypad_set_rumble,
   mister_joypad_set_rumble_gain,
   mister_joypad_set_sensor_state,
   mister_joypad_get_sensor_input,
   mister_joypad_name,
   "mister"
};
