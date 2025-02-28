#include "polyfills.h"

// time
// malloc/calloc
// free(?)
// thread / xtaskCreate 

#ifdef AMY_IS_EXTERNAL
void * malloc_caps(uint32_t size, uint32_t flags) {
#ifdef ESP_PLATFORM
    //fprintf(stderr, "allocing size %ld flags %ld\n", size, flags);
    return heap_caps_malloc(size, flags);
#else
    // ignore flags
    return malloc(size);
#endif
}
#endif


#ifndef ESP_PLATFORM

void display_brightness(uint8_t amount) {
}

float compute_cpu_usage(uint8_t debug) {
    return 0;
}
#endif



void display_start() {
#ifdef ESP_PLATFORM
    esp32s3_display_start();
#endif
}


void display_stop() {
#ifdef ESP_PLATFORM
    esp32s3_display_stop();
#endif
}


uint8_t rand_uint8() {
#ifdef ESP_PLATFORM
    return rand() % 255; // i thought esp_random() would be faster but it's not 
#else
    return rand() % 255;
#endif
}

void delay_ms(uint32_t ms) {
#ifdef ESP_PLATFORM
    vTaskDelay(ms / portTICK_PERIOD_MS);
#else
    #ifndef __EMSCRIPTEN__
    usleep(ms * 1000);
    #else
    // nothing
    #endif
#endif
}


void take_semaphore() {
#ifdef ESP_PLATFORM
    ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(100));
#else
    // nothing? not sure yet
#endif
}


int64_t get_time_us() {
    return mp_hal_ticks_us();
}

extern int32_t get_sysclock();
int64_t get_time_ms() {
    return mp_hal_ticks_ms(); // a large number
}
int32_t get_ticks_ms() {
//#ifndef __EMSCRIPTEN__
//    return amy_sysclock(); // based on audio driver
//#else
    return mp_hal_ticks_ms(); 
//#endif
}

void *calloc_caps(uint32_t align, uint32_t count, uint32_t size, uint32_t flags) {
#ifdef ESP_PLATFORM
    //if(flags & MALLOC_CAP_SPIRAM) fprintf(stderr, "spiram callocing count %ld size %ld flags %ld\n", count, size, flags);
    return heap_caps_aligned_calloc(align, count, size, flags); 
#else
    return (void*)malloc(size*count);
#endif
}

void *realloc_caps(void* ptr, uint32_t size, uint32_t caps) {
#ifdef ESP_PLATFORM
  //fprintf(stderr, "re-allocing size %ld flags %ld\n", size, caps);
  return heap_caps_realloc(ptr, size, caps);
#else
  return (void*)realloc(ptr, size);
#endif
}

void free_caps(void *ptr) {
#ifdef ESP_PLATFORM
    heap_caps_free(ptr);
#else
    free(ptr);
#endif
}
