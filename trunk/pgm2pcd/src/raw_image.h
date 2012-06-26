/*
 * depth_image.h
 *
 *  Created on: 25/06/2012
 *      Author: pachi
 */

#ifndef RAW_IMAGE_H_
#define RAW_IMAGE_H_

typedef unsigned char uchar;

class RawImage
{
  public:
    RawImage ();
    RawImage (const char*);
    virtual ~RawImage ();

    int getChannels () const
    {
      return channels_;
    }

    uchar* getData () const
    {
      return data_;
    }

    int getDepth () const
    {
      return depth_;
    }

    int getHeight () const
    {
      return height_;
    }

    int getWidth () const
    {
      return width_;
    }

    void loadImage (const char *filename)
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
