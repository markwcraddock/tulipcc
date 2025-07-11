/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2021 Damien P. George and 2017, 2018 Rami Ali
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "py/builtin.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "extmod/vfs.h"
#include "extmod/vfs_posix.h"
#include "shared/runtime/pyexec.h"

#include "emscripten.h"
#include "lexer_dedent.h"
#include "library.h"
#include "proxy_c.h"
#include "py/ringbuf.h"
#include "SDL.h"
#include "emscripten/html5.h"
#include "tsequencer.h"
#include "amy_connector.h"
extern void setup_lvgl();
void tulip_start();

#define TULIP_FS_SYNC_DIVIDER 120
#define TULIP_FRAME_SCHEDULER_EATS 4

STATIC uint8_t stdin_ringbuf_array[260];
ringbuf_t stdin_ringbuf = {stdin_ringbuf_array, sizeof(stdin_ringbuf_array), 0, 0};


// This counter tracks the current depth of calls into C code that originated
// externally, ie from JavaScript.  When the counter is 0 that corresponds to
// the top-level call into C.
static size_t external_call_depth = 0;

#if MICROPY_GC_SPLIT_HEAP_AUTO
static void gc_collect_top_level(void);
#endif

void external_call_depth_inc(void) {
    ++external_call_depth;
    #if MICROPY_GC_SPLIT_HEAP_AUTO
    if (external_call_depth == 1) {
        gc_collect_top_level();
    }
    #endif
}

void external_call_depth_dec(void) {
    --external_call_depth;
}
extern void mp_schedule_lv();
extern void unix_display_init();
extern int unix_display_draw();
uint8_t tulip_ready = 0;
extern uint8_t display_size_changed;
extern SDL_Window * window;
extern void force_rescale();
uint32_t frame_count = 0;
uint32_t last_tick = 0;
void main_loop__tulip() {
    if(tulip_ready) {
        // We seem to be stuck at 60fps (even if we change it in the loop setup), so let's halve that to be more "Tulip-y"
        if(frame_count++ % 4 == 0)
            unix_display_draw();
        
        //mp_schedule_lv();

        // Why run this more than once? Tulip on web updates all its stuff twice a frame. 
        // We schedule at least 1 thing a frame - the LVGL updater
        // So this ensures that anything else scheduled (MIDI, sequencer, frame_callback, touch_callback) still can run
        //for(uint8_t i=0;i<TULIP_FRAME_SCHEDULER_EATS;i++) 
        MICROPY_EVENT_POLL_HOOK
        //mp_handle_pending(true);

        if(frame_count % TULIP_FS_SYNC_DIVIDER == 0)
            EM_ASM(
                FS.syncfs(false, function (err) {});
            );

        if(frame_count == 10) display_size_changed = 1;

        if (display_size_changed) {
            double w, h;
            int iw,ih;
            emscripten_get_canvas_element_size("#canvas", &iw, &ih);
            emscripten_get_element_css_size( "#canvas", &w, &h );
            //fprintf(stderr, "size change event --> CSS %d,%d DOM %d,%d\n", (int)w, (int) h ,iw,ih);
            SDL_SetWindowSize( window, (int)w, (int) h );
            force_rescale();
            unix_display_draw();
            display_size_changed = 0;
        }
        if(frame_count%1000==0) {
            uint32_t ticks = mp_hal_ticks_ms() - last_tick;
            //fprintf(stderr, "frame %d took %d ticks %2.2f FPS\n", frame_count, ticks, 1000.0/(ticks/1000.0));
            last_tick = mp_hal_ticks_ms();
        }

    }
}

void tulip_tick(uint32_t tick) {
    if(tulip_ready) tulip_amy_sequencer_hook(tick);
}

void setup_fs() {
    EM_ASM(
        try {
            FS.mkdir('/tulip4');
        } catch (err) {
            // OK, already exists
        }
        try {
            FS.mkdir('/tulip4/user');
        } catch (err) {
            // OK, already exists
        }

        // Then mount with IDBFS type
        // NB: autoPersist:true does not seem to work with the current tulip setup -- unsure why
        // So we use a divider on the tulip frame drawing to sync FS manually every couple seconds
        FS.mount(IDBFS,  {autoPersist:true}, '/tulip4/user');
        // Then sync
        FS.syncfs(true, function (err) {
            // callback
        });
    );
}
extern void unix_display_init();

void mp_js_init(int pystack_size, int heap_size) {

    setup_fs();
    emscripten_set_main_loop(main_loop__tulip, 0, 0);

    #if MICROPY_ENABLE_PYSTACK
    mp_obj_t *pystack = (mp_obj_t *)malloc(pystack_size * sizeof(mp_obj_t));
    mp_pystack_init(pystack, pystack + pystack_size);
    #endif

    #if MICROPY_ENABLE_GC
    char *heap = (char *)malloc(heap_size * sizeof(char));
    gc_init(heap, heap + heap_size);
    #endif

    #if MICROPY_GC_SPLIT_HEAP_AUTO
    // When MICROPY_GC_SPLIT_HEAP_AUTO is enabled, set the GC threshold to a low
    // value so that a collection is triggered before the heap fills up.  The actual
    // garbage collection will happen later when control returns to the top-level,
    // via the `gc_collect_pending` flag and `gc_collect_top_level()`.
    MP_STATE_MEM(gc_alloc_threshold) = 16 * 1024 / MICROPY_BYTES_PER_GC_BLOCK;
    #endif

    mp_init();

    #if MICROPY_VFS_POSIX
    {
        // Mount the host FS at the root of our internal VFS
        mp_obj_t args[2] = {
            MP_OBJ_TYPE_GET_SLOT(&mp_type_vfs_posix, make_new)(&mp_type_vfs_posix, 0, 0, NULL),
            MP_OBJ_NEW_QSTR(qstr_from_str("/")),
        };
        mp_vfs_mount(2, args, (mp_map_t *)&mp_const_empty_map);
        MP_STATE_VM(vfs_cur) = MP_STATE_VM(vfs_mount_table);
    }
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR__slash_lib));
    #endif

    sysex_buffer = malloc_caps(MAX_SYSEX_BYTES, MALLOC_CAP_SPIRAM);

    tsequencer_init();
    unix_display_init();
    setup_lvgl();
    tulip_ready = 1;
}

void mp_js_register_js_module(const char *name, uint32_t *value) {
    mp_obj_t module_name = MP_OBJ_NEW_QSTR(qstr_from_str(name));
    mp_obj_t module = proxy_convert_js_to_mp_obj_cside(value);
    mp_map_t *mp_loaded_modules_map = &MP_STATE_VM(mp_loaded_modules_dict).map;
    mp_map_lookup(mp_loaded_modules_map, module_name, MP_MAP_LOOKUP_ADD_IF_NOT_FOUND)->value = module;
}

void mp_js_do_import(const char *name, uint32_t *out) {
    external_call_depth_inc();
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t ret = mp_import_name(qstr_from_str(name), mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
        // Return the leaf of the import, eg for "a.b.c" return "c".
        const char *m = name;
        const char *n = name;
        for (;; ++n) {
            if (*n == '\0' || *n == '.') {
                if (m != name) {
                    ret = mp_load_attr(ret, qstr_from_strn(m, n - m));
                }
                m = n + 1;
                if (*n == '\0') {
                    break;
                }
            }
        }
        nlr_pop();
        external_call_depth_dec();
        proxy_convert_mp_to_js_obj_cside(ret, out);
    } else {
        // uncaught exception
        external_call_depth_dec();
        proxy_convert_mp_to_js_exc_cside(nlr.ret_val, out);
    }
}

void mp_js_do_exec(const char *src, size_t len, uint32_t *out) {
    external_call_depth_inc();
    mp_parse_input_kind_t input_kind = MP_PARSE_FILE_INPUT;
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len_dedent(MP_QSTR__lt_stdin_gt_, src, len, 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, false);
        mp_obj_t ret = mp_call_function_0(module_fun);
        nlr_pop();
        external_call_depth_dec();
        proxy_convert_mp_to_js_obj_cside(ret, out);
    } else {
        // uncaught exception
        external_call_depth_dec();
        proxy_convert_mp_to_js_exc_cside(nlr.ret_val, out);
    }
}
#include "py/frozenmod.h"

void mp_js_frozen_exec(const char *name) {
    pyexec_file_if_exists(name);
}

void mp_js_do_exec_async(const char *src, size_t len, uint32_t *out) {
    mp_compile_allow_top_level_await = true;
    mp_js_do_exec(src, len, out);
    mp_compile_allow_top_level_await = false;
}

void mp_js_repl_init(void) {
    pyexec_event_repl_init();
}

int mp_js_repl_process_char(int c) {
    mp_compile_allow_top_level_await = true;
    external_call_depth_inc();
    int ret = pyexec_event_repl_process_char(c);
    external_call_depth_dec();
    mp_compile_allow_top_level_await = false;
    return ret;
}

#if MICROPY_GC_SPLIT_HEAP_AUTO

static bool gc_collect_pending = false;

// The largest new region that is available to become Python heap.
size_t gc_get_max_new_split(void) {
    return 128 * 1024 * 1024;
}

// Don't collect anything.  Instead require the heap to grow.
void gc_collect(void) {
    gc_collect_pending = true;
}

// Collect at the top-level, where there are no root pointers from stack/registers.
static void gc_collect_top_level(void) {
    if (gc_collect_pending) {
        gc_collect_pending = false;
        gc_collect_start();
        gc_collect_end();
    }
}

#else

static void gc_scan_func(void *begin, void *end) {
    gc_collect_root((void **)begin, (void **)end - (void **)begin + 1);
}

void gc_collect(void) {
    gc_collect_start();
    emscripten_scan_stack(gc_scan_func);
    emscripten_scan_registers(gc_scan_func);
    gc_collect_end();
}

#endif

#if !MICROPY_VFS
mp_lexer_t *mp_lexer_new_from_file(qstr filename) {
    mp_raise_OSError(MP_ENOENT);
}

mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);
#endif

void nlr_jump_fail(void *val) {
    while (1) {
        ;
    }
}

void NORETURN __fatal_error(const char *msg) {
    while (1) {
        ;
    }
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif
