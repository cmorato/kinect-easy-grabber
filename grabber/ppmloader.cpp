#include <stdio.h>
#include <stdlib.h>
#include "ppmloader.h"


void LoadPPMFile(uchar **dst, int *width, int *height, const char *name){
	char line[256], *ptr;
	FILE* fid;

	// Open file
	fid = fopen(name, "rb");
	if (!fid) {
		printf("***PPM load error: file access denied***\n");
		exit(-1);
	}

	// Read PPM header P6
	char p6[10];
	fgets(p6, 10, fid);
	if ( (p6[0]!='P') || (p6[1]!='6') ) {  
		printf("Worng image type\n");
		exit(-2);
	}

	// Parse comments
	fgets(line, 256, fid);
	while(line[0]=='#')
		fgets(line, 256, fid);

	// Read dimensions
	*width = strtol(line, &ptr, 10);
	*height = strtol(ptr, &ptr, 10);

	if(ptr == NULL || *ptr == '\n'){
		fgets(line, 256, fid);
		ptr = line;
	}

	int levels = strtol(ptr, &ptr, 10);

	if(levels != 255){
		printf("Wrong number of levels\n");
		exit(-3);
	}

	int size = *width * *height * 3;

	unsigned char* data = new unsigned char[ size ];
	
	fread(data, 1, size, fid);
	fclose(fid);

	*dst = new uchar[*width * *height * 3];
	int numel = *width * *height;
	uchar* dest = *dst;

	for(int c=0; c < 3; c++)
		for(int i=0; i<numel; i++)
			*dest++ = data[3*i + c];

	delete [] data;
}


void SavePPMFile(uchar *src, int width, int height, const char *file)
{
	FILE* fid;
	int size = width * height * 3;

	unsigned char* data = new unsigned char[size];

	int numel = width * height;
	for(int c=0; c < 3; c++)
		for(int i=0; i<numel; i++)
			data[3*i + c] = *src++;

	fid = fopen(file,"wb");

	if (!fid) {
		printf("Can't open file: %s\n",file);
		exit(-1);
	}

	// Write PPM header
	fprintf(fid, "P6\n%i %i\n%i\n", width, height, 255);

	fwrite(data,1,size,fid);

	fclose(fid);

	delete [] data;
}
