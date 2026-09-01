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

/* Keyboard and mouse plugged into the MiSTer.
 *
 * Unlike the joypad driver, this one does NOT wrap the platform's input
 * driver: the keyboard lookup table it installs is process-wide, so two
 * keyboard sources cannot be live at once.  Selecting this driver therefore
 * replaces the host keyboard and mouse for as long as it is active, which is
 * the right trade for a cabinet and the wrong one for a desk.  The MiSTer's
 * pads are unaffected either way - those come through mister_joypad, which
 * does wrap.
 *
 * Requires PS2 = Keyboard (or Keyboard & Mouse) under Server -> Send inputs
 * in the MiSTer's own menu; without it the core simply sends nothing.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <boolean.h>
#include <libretro.h>

#include "../input_driver.h"
#include "../input_keymaps.h"

#include "../../configuration.h"
#include "../../gfx/gfx_mister.h"
#include "../../retroarch.h"

typedef struct
{
   mister_ps2_state_t ps2;
   int mouse_abs_x;
   int mouse_abs_y;
} mister_input_t;

static void *mister_input_init(const char *joypad_driver)
{
   mister_input_t *mister;

   if (!config_get_ptr()->bools.video_mister_enable)
      return NULL;

   if (!(mister = (mister_input_t*)calloc(1, sizeof(*mister))))
      return NULL;

   /* The wire carries SDL scancodes, so the lookup table has to match before
    * any key is translated. */
   input_keymaps_init_keyboard_lut(rarch_key_map_mister);

   return mister;
}

static void mister_input_free(void *data)
{
   if (data)
      free(data);
}

static void mister_input_poll(void *data)
{
   mister_input_t *mister = (mister_input_t*)data;

   if (!mister)
      return;

   if (!mister_poll_ps2(&mister->ps2))
      return;

   /* The mouse reports movement since the last packet; RetroArch's pointer
    * abstraction wants a position, so integrate here. */
   mister->mouse_abs_x += mister->ps2.mouse_x;
   mister->mouse_abs_y -= mister->ps2.mouse_y;   /* PS/2 Y counts upward */
}

static bool mister_key_pressed(mister_input_t *mister, int key)
{
   unsigned sym;

   if (key >= RETROK_LAST)
      return false;

   sym = rarch_keysym_lut[(enum retro_key)key];
   if (sym >= 256)
      return false;

   return (mister->ps2.keys[sym >> 3] >> (sym & 7)) & 1;
}

static int16_t mister_mouse_state(mister_input_t *mister, unsigned id)
{
   switch (id)
   {
      case RETRO_DEVICE_ID_MOUSE_X:
         return (int16_t)mister->ps2.mouse_x;
      case RETRO_DEVICE_ID_MOUSE_Y:
         return (int16_t)-mister->ps2.mouse_y;
      case RETRO_DEVICE_ID_MOUSE_LEFT:
         return (mister->ps2.mouse_buttons & 0x01) ? 1 : 0;
      case RETRO_DEVICE_ID_MOUSE_RIGHT:
         return (mister->ps2.mouse_buttons & 0x02) ? 1 : 0;
      case RETRO_DEVICE_ID_MOUSE_MIDDLE:
         return (mister->ps2.mouse_buttons & 0x04) ? 1 : 0;
      case RETRO_DEVICE_ID_MOUSE_WHEELUP:
         return (mister->ps2.mouse_wheel > 0) ? 1 : 0;
      case RETRO_DEVICE_ID_MOUSE_WHEELDOWN:
         return (mister->ps2.mouse_wheel < 0) ? 1 : 0;
      default:
         break;
   }
   return 0;
}

static int16_t mister_input_state(
      void *data,
      const input_device_driver_t *joypad,
      const input_device_driver_t *sec_joypad,
      rarch_joypad_info_t *joypad_info,
      const retro_keybind_set *binds,
      bool keyboard_mapping_blocked,
      unsigned port,
      unsigned device,
      unsigned idx,
      unsigned id)
{
   mister_input_t *mister = (mister_input_t*)data;

   if (!mister)
      return 0;

   switch (device)
   {
      case RETRO_DEVICE_JOYPAD:
         /* Keyboard-driven pad binds, as any keyboard input driver provides. */
         if (id == RETRO_DEVICE_ID_JOYPAD_MASK)
         {
            unsigned i;
            int16_t  ret = 0;

            for (i = 0; i < RARCH_FIRST_CUSTOM_BIND; i++)
               if (     binds[port][i].valid
                     && mister_key_pressed(mister, binds[port][i].key))
                  ret |= (1 << i);

            return ret;
         }

         if (id < RARCH_BIND_LIST_END)
            if (     binds[port][id].valid
                  && mister_key_pressed(mister, binds[port][id].key))
               return 1;
         break;

      case RETRO_DEVICE_KEYBOARD:
         return mister_key_pressed(mister, id) ? 1 : 0;

      case RETRO_DEVICE_MOUSE:
      case RARCH_DEVICE_MOUSE_SCREEN:
         return mister_mouse_state(mister, id);

      default:
         break;
   }

   return 0;
}

static uint64_t mister_input_get_capabilities(void *data)
{
   return   (1 << RETRO_DEVICE_JOYPAD)
          | (1 << RETRO_DEVICE_KEYBOARD)
          | (1 << RETRO_DEVICE_MOUSE);
}

input_driver_t input_mister = {
   mister_input_init,
   mister_input_poll,
   mister_input_state,
   mister_input_free,
   NULL,                            /* set_sensor_state */
   NULL,                            /* get_sensor_input */
   mister_input_get_capabilities,
   "mister",
   NULL,                            /* grab_mouse  */
   NULL,                            /* grab_stdin  */
   NULL                             /* keypress_vibrate */
};
