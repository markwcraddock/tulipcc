// lvgl_u8g2.c
// render u8g2 fonts in lvgl 
#include "lvgl.h"
#include "u8g2_fonts.h"


// will make this less nasty asap
#define MAX_FONT_W 50
#define MAX_FONT_H 80
uint8_t databuf[MAX_FONT_W*MAX_FONT_H];


/* Get info about glyph of `unicode_letter` in `font` font.
 * Store the result in `dsc_out`.
 * The next letter (`unicode_letter_next`) might be used to calculate the width required by this glyph (kerning)
 */
bool my_get_glyph_dsc_cb(const lv_font_t * font, lv_font_glyph_dsc_t * dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next)
{
    // Even though this callback is just to get font/glyph info, u8g2 can't really know everything about the glyph
    // until u8g2 draws it. so i draw it, save the bitmap to a temp part of RAM , extract the info, and the
    // drawing CB will use the same bit of ram without having to re-draw. 
    // unfortunately this function seems to be called 3 times per glyph, so maybe try to not draw so much

    // BDF fonts only have lower ascii i think
    if(unicode_letter>127) {
        return false;
    }

    uint32_t font_no = *((uint32_t*)(font->user_data));
    u8g2_font_t ufont;
    ufont.font = NULL; 
    ufont.font_decode.fg_color = 1; 
    ufont.font_decode.is_transparent = 1; 
    ufont.font_decode.dir = 0;
    u8g2_SetFont(&ufont, tulip_fonts[font_no]);
    for(uint16_t i=0;i<(MAX_FONT_H*MAX_FONT_W);i++) { databuf[i] = 0; }
    int16_t adv = u8g2_DrawGlyph_target(&ufont, unicode_letter, databuf);
    u8g2_font_decode_t font_decode = u8g2_GetGlyphInfo(&ufont, unicode_letter);
    dsc_out->adv_w = adv;        /*Horizontal space required by the glyph in [px]*/

    // ufont height/width are swapped
    dsc_out->box_w = ufont.font_info.max_char_width;      /*Width of the bitmap in [px]*/
    dsc_out->box_h = ufont.font_info.max_char_height;       /*Width of the bitmap in [px]*/

    dsc_out->ofs_x = 0;                            /*X offset of the bitmap in [pf]*/
    dsc_out->ofs_y = 0;
    dsc_out->format= LV_FONT_GLYPH_FORMAT_A1;
    dsc_out->gid.index = unicode_letter; 
    return true;                /*true: glyph found; false: glyph was not found*/
}

const void * my_get_glyph_bitmap_cb(lv_font_glyph_dsc_t * g_dsc, lv_draw_buf_t * draw_buf)
{
    memcpy(draw_buf->data, databuf, g_dsc->box_w*g_dsc->box_h);
    return draw_buf;
}

void get_lvgl_font_from_tulip(uint32_t font_no, lv_font_t * outfont) {
    u8g2_font_t ufont;
    ufont.font = NULL; 
    ufont.font_decode.fg_color = 1; 
    ufont.font_decode.is_transparent = 1; 
    ufont.font_decode.dir = 0;
    u8g2_SetFont(&ufont, tulip_fonts[font_no]);
    
    outfont->get_glyph_dsc = my_get_glyph_dsc_cb;        /*Set a callback to get info about glyphs*/
    outfont->get_glyph_bitmap = my_get_glyph_bitmap_cb;  /*Set a callback to get bitmap of a glyph*/
    outfont->line_height = ufont.font_info.max_char_width;                       /*The real line height where any text fits*/
    outfont->base_line = 0;//abs(ufont.font_info.y_offset); // base_line;                      /*Base line measured from the top of line_height*/
    //outfont->fallback = &lv_font_montserrat_12;
    void *ptr = malloc(sizeof(uint32_t));
    *((uint32_t*)ptr) = font_no;
    outfont->user_data = ptr;
}

