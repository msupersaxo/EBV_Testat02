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

const int MaxCounter = 50;


void ProcessFrame(uint8 *pInputImg)
{
	int c, r, i0;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;/* we work on half of the camera image width */
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);
	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)
		{
			if(data.ipc.state.nStepCounter == 1) {//initialize counters
				for(i0 = 0; i0 < 3; i0++) {
					 data.u8TempImage[INDEX0+i0][r+c] = 0;
				}
			}
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[GRAYSCALE][r+c];
			/* implement Sobel filter (shift by 128 to 'center' grey values */
			short dx =     -(short) *(p-nc-1) + (short) *(p-nc+1)
			            -2* (short) *(p-1) + 2* (short) *(p+1)
			               -(short) *(p+nc-1) + (short) *(p+nc+1);
			/* apply min()/max() to avoid wrap around of values below 0 and above 255 */

			/* implement Sobel filter (shift by 128 to 'center' grey values */
			short dy =    -(short) *(p-nc-1) -2* (short) *(p-nc) - (short) *(p-nc+1)
			               +(short) *(p+nc-1) +2* (short) *(p+nc) + (short) *(p+nc+1);
			/* apply min()/max() to avoid wrap around of values below 0 and above 255 */
			short absdx = abs(dx);
			short absdy = abs(dy);

			unsigned char Edge = (absdx+absdy) < data.ipc.state.nThreshold ? 0 : 255;

			int Index = 0;
			if(Edge) {  //edge is true
				if(absdx > absdy) {
					Index = 1;
				} else {
					Index = 2;
				}
			} //else nothing to do (Index already = 0)

			//increment counters
			int MaxV = data.u8TempImage[INDEX0][r+c];
			int MaxI = 0;
			for(i0 = 0; i0 < 3; i0++) {
				if(Index == i0) {//increment corresponding counter
					data.u8TempImage[INDEX0+i0][r+c] = MIN(MaxCounter, data.u8TempImage[INDEX0+i0][r+c]+1);
				} else {//decrement corresp. counter
					data.u8TempImage[INDEX0+i0][r+c] = MAX(0, data.u8TempImage[INDEX0+i0][r+c]-1);
				}

				if(data.u8TempImage[INDEX0+i0][r+c] > MaxV) {
					MaxV = data.u8TempImage[INDEX0+i0][r+c];
					MaxI = i0;
				}
			}
			//condition for foreground
			data.u8TempImage[THRESHOLD][r+c] = MaxV;
			data.u8TempImage[BACKGROUND][r+c] = (MaxV > MaxCounter/2) && (Index != MaxI) ? 255 : 0;
		}
	}




}




