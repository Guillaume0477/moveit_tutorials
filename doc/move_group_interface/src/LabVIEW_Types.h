
#pragma once

#include <memory>

/* Call Library source file */

#include <extcode.h>
/* lv_prolog.h and lv_epilog.h set up the correct alignment for LabVIEW data. */
#include "lv_prolog.h"
#include "lv_epilog.h"

//==============================================================================
 //   LabVIEW types
 //==============================================================================

 // Array1D of float LabVIEW
typedef struct {
    uint32_t dimSize;
    float elt[1];
} arr1Df;
typedef arr1Df** arr1Df_LV;

// Array2D of double LabVIEW
typedef struct {
    uint32_t dimSize[2];
    double elt[1];
} arr2Dd;
typedef arr2Dd** arr2Dd_LV;

// Array2D of uint32_t LabVIEW
typedef struct {
    uint32_t dimSize[2];
    uint32_t elt[1];
} arr2DU32;
typedef arr2DU32** arr2DU32_LV;

// Array2D of uint8_t LabVIEW
typedef struct {
    uint32_t dimSize[2];
    uint8_t elt[1];
} arr2DU8;
typedef arr2DU8** arr2DU8_LV;

// Array1D of string LabVIEW
typedef struct {
    uint32_t dimSize;
    LStrHandle str[1];
} arr1Dstr;
typedef arr1Dstr** arr1Dstr_LV;

// Dimensions xyz
typedef struct{
    float width;//x
    float length;//y
    float height;//z
} Dims;

typedef struct {
    float thickness_top;
    float thickness_bot;
} Thickness;

// Position xyz
typedef struct {
    float Tx;
    float Ty;
    float Tz;
} Position;

// Orientation xyz
typedef struct {
    float Rx;
    float Ry;
    float Rz;
} Orientation;

// Quaternion
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quat;