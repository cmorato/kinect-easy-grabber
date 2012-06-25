#ifndef __PPMPLOADER_H__
#define __PPMLOADER_H__

typedef unsigned char uchar;

void LoadPPMFile(uchar **dst, int *width, int *height, const char *name);
void SavePPMFile(uchar *dst, int width, int height, const char *file);

#endif //__PPMLOADER_H__