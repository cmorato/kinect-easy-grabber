/*
 * depth_image.cpp
 *
 *  Created on: 25/06/2012
 *      Author: pachi
 */
#include <stdio.h>
#include <stdlib.h>
#include "raw_image.h"
#include <stdexcept>
#include <string>

RawImage::RawImage ()
{
  data_ = NULL;
  width_ = height_ = channels_, depth_ = 0;
}

RawImage::RawImage (const char* filename)
{
  loadNetpbmImage(filename);
}

RawImage::~RawImage ()
{
  if(data_)
    delete [] data_;
  data_ = NULL;
}

void
RawImage::loadNetpbmImage (const char *name)
{
  char line[256], *ptr;
  FILE* fid;

  // Open file
  fid = fopen(name, "rb");
  if (!fid) {
    throw std::runtime_error("[DepthImage::loadPPMFile] Load error: file access denied\n");
  }

  // Read header: P5-8bit, P5-16bit or P6
  char imtype[10];
  fgets(imtype, 10, fid);
  if ( (imtype[0]=='P') && (imtype[1]=='5') )
  {
    channels_ = 1;
  }else if ( (imtype[0]=='P') && (imtype[1]=='6') )
  {
    channels_ = 3;
  }
  else
    throw std::runtime_error("Wrong image type\n");

  // Parse comments
  fgets(line, 256, fid);
  while(line[0]=='#'){
    comments_ += line;
    fgets(line, 256, fid);
  }

  // Read dimensions
  width_ = strtol(line, &ptr, 10);
  height_ = strtol(ptr, &ptr, 10);

  if(ptr == NULL || *ptr == '\n'){
    fgets(line, 256, fid);
    ptr = line;
  }

  int levels = strtol(ptr, &ptr, 10);

  if(levels <= 255) depth_ = 1;
  else depth_ = 2;

  int size = width_ * height_ * channels_ * depth_;

  data_ = new uchar[ size ];

  fread(data_, 1, size, fid);
  fclose(fid);

  /*
  *dst = new uchar[*width * *height * 3];
  int numel = *width * *height;
  uchar* dest = *dst;

  // split rgb components
  for(int c=0; c < 3; c++)
    for(int i=0; i<numel; i++)
      *dest++ = data_[3*i + c];

  delete [] data_;
  data_ = dest;
  */
}


void
RawImage::saveNetpbmImage (const char *filename)
{
  FILE* fid = NULL;
  int size = width_ * height_ * channels_ * depth_;
  /*
  uchar* odata = new uchar[size];
  // merge rgb channels
  int numel = width * height;
  for(int c=0; c < 3; c++)
    for(int i=0; i<numel; i++)
      odata[3*i + c] = *src++;
  */

  fid = fopen(filename,"wb");

  if (!fid)
    throw std::runtime_error("Can't open file\n");

  // Write header
  //TODO: write comments too
  if (channels_ == 1) //PGM P5 header
  {
    if (depth_ == 1)
      fprintf(fid, "P5\n%i %i\n%i\n", width_, height_, 255);
    else if (depth_ == 2)
      fprintf(fid, "P5\n%i %i\n%i\n", width_, height_, 65535);
    else
      throw std::runtime_error("Invalid depth.\n");
  }
  else if (channels_ == 3) //PPM P6 header
  {
    if (depth_ == 1)
      fprintf(fid, "P6\n%i %i\n%i\n", width_, height_, 255);
    else
      throw std::runtime_error("Invalid depth.\n");
  }

  fwrite(data_,1,size,fid);
  fclose(fid);
}
