/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color);

void ProcessFrame(uint8 *pInputImg)
{
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[GRAYSCALE], siz);

	memset(data.u8TempImage[THRESHOLD], 0, siz);
}




