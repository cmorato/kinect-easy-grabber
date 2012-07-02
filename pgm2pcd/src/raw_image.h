/*
 * depth_image.h
 *
 *  Created on: 25/06/2012
 *      Author: pachi
 */

#ifndef RAW_IMAGE_H_
#define RAW_IMAGE_H_

#include <stdexcept>

typedef unsigned char uchar;
typedef unsigned short ushort;

class RawImage
{
  public:
    RawImage ();
    RawImage (const char*);
    virtual ~RawImage ();

    inline int getChannels () const
    {
      return channels_;
    }

    inline uchar* getData () const
    {
      if (!data_) throw std::runtime_error("No data allocated.");
      return data_;
    }

    inline ushort* get16bitData () const
	{
	  return (ushort*)(data_);
	}

    inline int getDepth () const
    {
      return depth_;
    }

    inline int getHeight () const
    {
      return height_;
    }

    inline int getWidth () const
    {
      return width_;
    }

    inline void loadImage (const char *filename)
    {
      loadNetpbmImage (filename);
    }

  protected:
    void
    loadNetpbmImage(const char *filename);
    void
    saveNetpbmImage(const char *filename);

  private:
    int width_;
    int height_;
    int channels_;
    int depth_;
    uchar* data_;
};

#endif /* RAW_IMAGE_H_ */
