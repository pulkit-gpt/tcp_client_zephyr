#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>

/* Data returned to main */
typedef struct {
    uint32_t red;
    uint32_t ir;
    float hr;
    float spo2;
    float pi;
    bool valid;
} max30102_data_t;

#define MAX_LINES 150
#define LINE_LEN  96

static char max_data[MAX_LINES][LINE_LEN];

int max30102_main(char out[MAX_LINES][LINE_LEN]);

#endif /* MAX30102_H */
