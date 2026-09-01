/* Single-source definitions: Groovy MiSTer output group.
 * Grammar identical to settings_def_crt_switchres.h; the descriptor
 * argument span matches SDESC_<kind>_ROW; h2json.py parses these rows for
 * the Crowdin source upload.
 *
 * Row order here is NOT menu display order. It sets the order settings are
 * registered, and so the order they land in retroarch.cfg. Which settings
 * appear on which page, and in what order, is decided by the hand-written
 * build_list tables in menu/menu_displaylist.c - and a setting absent from
 * every one of those is unreachable in the menu no matter what this file
 * says. Two were, for two rounds, before anyone noticed.
 *
 * Frames are streamed to a MiSTer FPGA over UDP and scanned out to an
 * analog CRT, so the modeline comes from switchres and the MiSTer, not
 * from the host display. */

S_BOOL(video_mister_enable, VIDEO_MISTER_ENABLE,
      "video_mister_enable",
      DEFAULT_VIDEO_MISTER_ENABLE, SD_FLAG_ADVANCED, 0, CMD_EVENT_NONE,
      "Groovy MiSTer Output",
      "Stream video to a MiSTer FPGA over the network instead of presenting to a local display. Requires CRT SwitchRes for modeline generation.")
/* The generated string row cannot carry a default (the configuration pass
 * emits SETTING_ARRAY with default_enable false), and this one needs one,
 * so the configuration.c row stays literal for this setting. */
#ifndef SETTINGS_DEF_CONFIG_PASS
S_STRING(mister_ip, MISTER_IP,
      "mister_ip",
      DEFAULT_MISTER_IP, SD_FLAG_ALLOW_INPUT | SD_FLAG_ADVANCED, CMD_EVENT_NONE, NULL, NULL, setting_generic_action_start_default, NULL, NULL, NULL, ST_UI_TYPE_STRING_LINE_EDIT,
      "MiSTer Address",
      "IP address of the MiSTer. Select this entry to type it in. A direct gigabit link with static addresses on both ends is strongly recommended; the address is read when the stream connects, so set it before turning Groovy MiSTer Output on.")
#endif
S_UINT_EX(crt_switch_mode_priority, CRT_SWITCH_MODE_PRIORITY,
      "crt_switch_mode_priority",
      DEFAULT_CRT_SWITCH_MODE_PRIORITY, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, 0, 0, 1, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_crt_switch_mode_priority, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Mode Priority",
      "What to give up when a core's picture cannot be scanned at the rate it asks for. Keeping the resolution sends every line the core drew, at whatever refresh the monitor can manage - which slows the emulation to match. Keeping the refresh rate runs the core at its proper speed and scales the picture to a size the monitor can scan at that rate. Only has any effect when the two cannot both be had; a core whose resolution already fits is untouched either way.")
S_UINT_EX(crt_switch_scan_mode, CRT_SWITCH_SCAN_MODE,
      "crt_switch_scan_mode",
      DEFAULT_CRT_SWITCH_SCAN_MODE, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, 0, 0, 1, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_crt_switch_scan_mode, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Scan Mode",
      "Whether an interlaced mode is an acceptable answer. Automatic lets the monitor preset decide, which is what you want unless you know otherwise. Progressive Only refuses interlace outright: on a 15 kHz preset that forces the picture down into the progressive line range, so a 480-line core is scanned at 240 lines instead of 480 interlaced - steadier, at half the vertical detail. No effect on a preset that has no interlaced range to begin with.")
S_UINT_EX(crt_switch_refresh_tolerance, CRT_SWITCH_REFRESH_TOLERANCE,
      "crt_switch_refresh_tolerance",
      DEFAULT_CRT_SWITCH_REFRESH_TOLERANCE, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, 0, 0, 10, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_crt_switch_refresh_tolerance, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_SPINBOX,
      "Keep Resolution Limit",
      "Has no effect unless Mode Priority is set to Keep Resolution. It is how much of the game's speed you will give up to keep every line the core drew. A monitor that cannot scan the picture at the rate the core asks for scans it slower instead, and the modeline is the frame clock, so the game and its sound slow by the same amount - on a 15 kHz preset that measured 5.45% with GameCube content. Past this limit the picture is scaled into a mode that runs at the right speed instead, and a message says so. Unlimited keeps the resolution whatever it costs.")
S_UINT_EX(mister_pacing, MISTER_PACING,
      "mister_pacing",
      DEFAULT_MISTER_PACING, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 1, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_mister_pacing, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Frame Clock",
      "Who decides when a frame starts. The MiSTer raster gives the lowest and most deterministic latency, and stands RetroArch's own frame limiter down while it is in charge; the RetroArch limiter leaves that in place so a network stall cannot hold up emulation. Either way the stream waits for the raster before starting the next frame, sleeping through most of the wait so a demanding core keeps the processor time it needs.")
S_UINT_EX(mister_codec, MISTER_CODEC,
      "mister_codec",
      DEFAULT_MISTER_CODEC, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 7, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_mister_codec, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Compression",
      "How frames are compressed before being sent. NLC gives the best quality per byte and holds a locked frame rate on heavy 3D, but needs a core whose bitstream carries the matching decoder. If the picture is noise or garbage, choose LZ4.")
S_UINT_EX(mister_nlc_pack, MISTER_NLC_PACK,
      "mister_nlc_pack",
      DEFAULT_MISTER_NLC_PACK, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 1, 2, 1.0, 1, setting_action_ok_uint, setting_get_string_representation_uint_mister_nlc_pack, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "NLC Entropy Pack",
      "Which entropy coder NLC uses. Tiled suits flat 2D content and works on any NLC core; Rice suits photographic and 3D content but needs a Rice-capable core. There is no way to ask the core which it is, so an older one misreads the stream and shows a garbled picture with no error anywhere - if that is what you see, switch to Tiled.")
S_UINT_EX(mister_nlc_near, MISTER_NLC_NEAR,
      "mister_nlc_near",
      DEFAULT_MISTER_NLC_NEAR, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 3, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_mister_nlc_near, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "NLC Level",
      "How much NLC may quantise. Lossless can peak high enough to saturate the link on heavy scenes; near-lossless +/-1 is invisible on a CRT and is what keeps the stream inside the ingest ceiling.")
S_BOOL(mister_use_inputs, MISTER_USE_INPUTS,
      "mister_use_inputs",
      DEFAULT_MISTER_USE_INPUTS, SD_FLAG_ADVANCED, 0, CMD_EVENT_NONE,
      "Use MiSTer Controllers",
      "Read the controllers, keyboard and mouse plugged into the MiSTer, so a cabinet needs no host-side controller. Requires the matching Send inputs options in the MiSTer's own menu - a fresh core install enables joysticks but leaves PS2 off, so the keyboard and mouse need turning on there by hand.")
S_BOOL(mister_rumble, MISTER_RUMBLE,
      "mister_rumble",
      DEFAULT_MISTER_RUMBLE, SD_FLAG_ADVANCED, 0, CMD_EVENT_NONE,
      "MiSTer Rumble",
      "Send force feedback to MiSTer-attached controllers. Needs a core new enough to negotiate it, and the per-controller Rumble option on the MiSTer side.")
/* Same as mister_ip: the generated string row is emitted into the int table by
 * the configuration pass, so every S_STRING in settings/ is excluded from it
 * and registered literally in configuration.c instead. */
#ifndef SETTINGS_DEF_CONFIG_PASS
S_STRING(mister_joypad_host_driver, MISTER_JOYPAD_HOST_DRIVER,
      "mister_joypad_host_driver",
      DEFAULT_MISTER_JOYPAD_HOST_DRIVER, SD_FLAG_ALLOW_INPUT | SD_FLAG_ADVANCED, CMD_EVENT_NONE, NULL, NULL, setting_generic_action_start_default, NULL, NULL, NULL, ST_UI_TYPE_STRING_LINE_EDIT,
      "Wrapped Controller Driver",
      "The MiSTer controller driver is a wrapper: it adds the MiSTer pads on top of an ordinary joypad driver so controllers plugged into this PC keep working. Names which driver to wrap, by its identifier - \"xinput\", \"dinput\", \"sdl2\", \"udev\". Leave empty and it wraps whatever Settings > Drivers > Controller is set to, which is what you want unless a specific driver is needed.")
#endif
S_UINT_EX(mister_joypad_port_base, MISTER_JOYPAD_PORT_BASE,
      "mister_joypad_port_base",
      DEFAULT_MISTER_JOYPAD_PORT_BASE, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 14, 1.0, 0, setting_action_ok_uint, NULL, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_SPINBOX,
      "First MiSTer Device Index",
      "The MiSTer's two pads appear to RetroArch as controllers at this device index and the next one. Kept above the PC's own controllers so their numbering never shifts; the default of 8 puts them at indices 8 and 9. Point a player at one under Settings > Input > Port 1 Controls > Device Index.")
S_UINT_EX(mister_rgb_mode, MISTER_RGB_MODE,
      "mister_rgb_mode",
      DEFAULT_MISTER_RGB_MODE, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 2, 2.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_mister_rgb_mode, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Colour Depth",
      "Colour depth on the wire. RGB565 halves bandwidth at the cost of banding, but NLC has no RGB565 encoder and that combination is forced back to RGB888. If you wanted RGB565 to save bandwidth, NLC Near-Lossless Level is the better control - it quantises the prediction residual instead of laying a fixed 5/6/5 grid across every gradient, and comes out below uncompressed RGB565 on the wire.")
S_UINT_EX(mister_mtu, MISTER_MTU,
      "mister_mtu",
      DEFAULT_MISTER_MTU, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 1500, 3800, 2300.0, 1500, setting_action_ok_uint, setting_get_string_representation_uint_mister_mtu, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "MTU",
      "Datagram size. Standard is safe everywhere. Jumbo frames have to be raised at both ends: the option in the MiSTer's own menu, and the MTU of this PC's network adapter - the client sets the do-not-fragment bit, so an adapter still at 1500 fails every send outright rather than fragmenting.")
S_BOOL(mister_interlaced_fb, MISTER_INTERLACED_FB,
      "mister_interlaced_fb",
      DEFAULT_MISTER_INTERLACED_FB, SD_FLAG_ADVANCED, 0, CMD_EVENT_NONE,
      "Interlaced Framebuffer",
      "Recommended on. For an interlaced mode, send one field per blit - half the lines each frame, alternating - which is what a core drawing whole frames wants, and half the data over the wire. Turn it off only for a core that draws at the field rate, roughly double the usual frame rate: the whole frame is then sent every time and the MiSTer picks the fields out of it, at twice the bandwidth. This has no bearing on which mode is chosen - that is Scan Mode, under CRT SwitchRes.")
S_UINT_EX(mister_log_level, MISTER_LOG_LEVEL,
      "mister_log_level",
      DEFAULT_MISTER_LOG_LEVEL, SD_FLAG_ADVANCED, SDESC_RANGE_MINMAX, CMD_EVENT_NONE, 0, 2, 1.0, 0, setting_action_ok_uint, setting_get_string_representation_uint_mister_log_level, NULL, NULL, NULL, NULL, ST_UI_TYPE_UINT_COMBOBOX,
      "Log Verbosity",
      "How much detail goes into mister.log, written to the log directory whenever Groovy MiSTer Output is on. Connection details, the modeline and every reason a frame was not sent are always recorded; the higher levels add frame pacing telemetry and then per-frame tracing, which costs time and should be used for short debugging runs rather than normal play.")
