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
const int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT / 2;
const int siz = (OSC_CAM_MAX_IMAGE_WIDTH / 2) * (OSC_CAM_MAX_IMAGE_HEIGHT / 2);

OSC_ERR OscInitChgDetection();
OSC_ERR OscUpdateChgDetection(int InputIndex, int OutputIndex);
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn,
		struct OSC_VIS_REGIONS *regions, uint8 Color, int MinSize);

void ProcessFrame(uint8 *pInputImg) {
	struct OSC_VIS_REGIONS regions;
	struct OSC_PICTURE mypic1,mypic2,mypic3;
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		OscInitChgDetection();
	}
	//do change detection update
	OscUpdateChgDetection(GRAYSCALE, THRESHOLD);
	// do opening
	OscErosion(THRESHOLD,BACKGROUND);
	OscDilation(BACKGROUND,THRESHOLD);
	//do closing
	OscDilation(THRESHOLD,BACKGROUND);
	OscErosion(BACKGROUND,THRESHOLD);
	//Wrapping
	mypic1.data=data.u8TempImage[THRESHOLD];
	mypic1.height=nr;
	mypic1.width=nc;
	mypic1.type=OSC_PICTURE_GREYSCALE;
	//Wrapping
	mypic3.data=data.u8TempImage[GRAYSCALE];
	mypic3.height=nr;
	mypic3.width=nc;
	mypic3.type=OSC_PICTURE_GREYSCALE;
	//Wrapping
	mypic2.data=data.u8TempImage[BACKGROUND];
	mypic2.height=nr;
	mypic2.width=nc;
	mypic2.type=OSC_PICTURE_GREYSCALE;
	//Do Region Labeling
	OscVisGrey2BW(&mypic1,&mypic2,100,0);
	OscVisLabelBinary(&mypic2, &regions);
	OscVisGetRegionProperties(&regions);
	//Draw Bounding Box
	OscVisDrawBoundingBoxBW(&mypic2, &regions, 255, 600);
	OscVisDrawBoundingBoxBW(&mypic3, &regions, 255, 600);

}

/* initialization of change detection buffers required in first step of processing: */
/* three buffers starting from INDEX0 are set to zero */
OSC_ERR OscInitChgDetection() {
	for (int i0 = 0; i0 < 3; i0++) {
		memset(data.u8TempImage[INDEX0 + i0], 0,
				sizeof(data.u8TempImage[INDEX0 + i0]));
	}
	return SUCCESS;
}


/* do update of change detection counters: */
/* using a gradient filter applied to image information from buffer InputIndex the indices in the buffers INDEX0+0,1,2 are updated:  */
/* INDEX0+0: pixel is no edge i.e. gradient N1-norm is lower than threshold */
/* INDEX0+1,2: pixel is edge i.e. gradient N1-norm is above threshold; INDEX+1 has higher x-derivative */
/* binary output is written to buffer corresponding to OutputIndex */
OSC_ERR OscUpdateChgDetection(int InputIndex, int OutputIndex) {
int c, r, i0;
for (r = nc; r < siz - nc; r += nc)/* we skip the first and last line */
{
	for (c = 1; c < nc - 1; c++) {
		/* do pointer arithmetics with respect to center pixel location */
		unsigned char* p = &data.u8TempImage[InputIndex][r + c];
		/* implement Sobel filter (shift by 128 to 'center' grey values */
		short dx = -(short) *(p - nc - 1) + (short) *(p - nc + 1)
				- 2 * (short) *(p - 1) + 2 * (short) *(p + 1)
				- (short) *(p + nc - 1) + (short) *(p + nc + 1);
		/* apply min()/max() to avoid wrap around of values below 0 and above 255 */

		/* implement Sobel filter (shift by 128 to 'center' grey values */
		short dy = -(short) *(p - nc - 1) - 2 * (short) *(p - nc)
				- (short) *(p - nc + 1) + (short) *(p + nc - 1)
				+ 2 * (short) *(p + nc) + (short) *(p + nc + 1);
		/* apply min()/max() to avoid wrap around of values below 0 and above 255 */
		short absdx = abs(dx);
		short absdy = abs(dy);

		unsigned char Edge =
				(absdx + absdy) < data.ipc.state.nThreshold ? 0 : 255;

		int Index = 0;
		if (Edge) {  //edge is true
			if (absdx > absdy) {
				Index = 1;
			} else {
				Index = 2;
			}
		} //else nothing to do (Index already = 0)

		//increment counters
		int MaxV = data.u8TempImage[INDEX0][r + c];
		int MaxI = 0;
		for (i0 = 0; i0 < 3; i0++) {
			if (Index == i0) { //increment corresponding counter
				data.u8TempImage[INDEX0 + i0][r + c] = MIN(MaxCounter,
						data.u8TempImage[INDEX0 + i0][r + c] + 1);
			} else { //decrement corresp. counter
				data.u8TempImage[INDEX0 + i0][r + c] = MAX(0,
						data.u8TempImage[INDEX0 + i0][r + c] - 1);
			}

			if (data.u8TempImage[INDEX0 + i0][r + c] > MaxV) {
				MaxV = data.u8TempImage[INDEX0 + i0][r + c];
				MaxI = i0;
			}
		}
		//condition for foreground
		data.u8TempImage[OutputIndex][r + c] =
				(MaxV > MaxCounter / 2) && (Index != MaxI) ? 255 : 0;
	}
}
return SUCCESS;
}

/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
/* the minimum size is the number of pixel*/
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn,
	struct OSC_VIS_REGIONS *regions, uint8 Color, int MinSize) {
uint16 i, o;
uint8 *pImg = (uint8*) picIn->data;
const uint16 width = picIn->width;
for (o = 0; o < regions->noOfObjects; o++) //loop over regions
		{
	//OscLog(INFO, "object area is %d\n", regions->objects[o].area);
	if (regions->objects[o].area > MinSize) {
		/* Draw the horizontal lines. */
		for (i = regions->objects[o].bboxLeft;
				i < regions->objects[o].bboxRight; i += 1) {
			pImg[width * regions->objects[o].bboxTop + i] = Color;
			pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		}

		/* Draw the vertical lines. */
		for (i = regions->objects[o].bboxTop;
				i < regions->objects[o].bboxBottom - 1; i += 1) {
			pImg[width * i + regions->objects[o].bboxLeft] = Color;
			pImg[width * i + regions->objects[o].bboxRight] = Color;
		}
	}
}
return SUCCESS;
}

OSC_ERR OscErosion(int InputIndex, int OutputIndex)
{
	int c, r;
	for (r = nc; r < siz - nc; r += nc)/* we skip the first and last line */
	{
		for (c = 1; c < nc - 1; c++) {
			unsigned char* p = &data.u8TempImage[InputIndex][r + c];
			data.u8TempImage[OutputIndex][r + c] = *(p - nc - 1) & *(p - nc)
											 & *(p - nc + 1) & *(p - 1) & *p & *(p + 1)
											 & *(p + nc - 1) & *(p + nc) & *(p + nc + 1);
		}
	}
	return SUCCESS;

}

OSC_ERR OscDilation(int InputIndex, int OutputIndex)
{
	int c, r;
	for (r = nc; r < siz - nc; r += nc)/* we skip the first and last line */
	{
		for (c = 1; c < nc - 1; c++) {
			unsigned char* p = &data.u8TempImage[InputIndex][r + c];
			data.u8TempImage[OutputIndex][r + c] = *(p - nc - 1) | *(p - nc)
											 | *(p - nc + 1) | *(p - 1) | *p & *(p + 1)
											 | *(p + nc - 1) | *(p + nc) | *(p + nc + 1);
		}
	}
	return SUCCESS;

}


