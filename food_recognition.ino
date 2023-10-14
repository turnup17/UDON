/*
 *  Spresense_gnss_simple.ino - Simplified gnss example application
 *  Copyright 2019-2021 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Camera.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DNNRT.h>
#include "Adafruit_ILI9341.h"

#include <SDHCI.h>
SDClass theSD;

/* LCD Settings */
#define TFT_RST 8
#define TFT_DC  9
#define TFT_CS  10

//(56,56), (32,32), (32,32)
#define DNN_IMG_W 56 //56, 32, 32
#define DNN_IMG_H 56 //56, 32, 32
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 48 //48, 96, 32
#define CAM_CLIP_Y 8 //8, 56, 56
#define CAM_CLIP_W 224 //224, 128, 256
#define CAM_CLIP_H 224 //224, 128, 128

#define LINE_THICKNESS 5

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

uint8_t buf[DNN_IMG_W*DNN_IMG_H];

DNNRT dnnrt;
DNNVariable input(DNN_IMG_W*DNN_IMG_H*3); //RGBだから3倍する
  
//static uint8_t const label[6] = {'carrot', 'onion', 'green_pepper', 'eggplant', 'potato', 'pork'};
static const char* label[6] = {"carrot", "onion", "green_pepper", "eggplant", "potato", "pork"};

void putStringOnLcd(String str, int color) {
  int len = str.length();
  tft.fillRect(0,224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - len/2*12;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

void drawBox(uint16_t* imgBuf) {
  /* Draw target line */
  for (int x = CAM_CLIP_X; x < CAM_CLIP_X+CAM_CLIP_W; ++x) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*(CAM_CLIP_Y+n) + x)              = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*(CAM_CLIP_Y+CAM_CLIP_H-1-n) + x) = ILI9341_RED;
    }
  }
  for (int y = CAM_CLIP_Y; y < CAM_CLIP_Y+CAM_CLIP_H; ++y) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W*y + CAM_CLIP_X+n)                = ILI9341_RED;
      *(imgBuf + CAM_IMG_W*y + CAM_CLIP_X + CAM_CLIP_W-1-n) = ILI9341_RED;
    }
  }  
}

void CamCB(CamImage img) {

  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  CamImage small; //縮小した画像を格納する
  CamErr err = img.clipAndResizeImageByHW(small
                     , CAM_CLIP_X, CAM_CLIP_Y
                     , CAM_CLIP_X + CAM_CLIP_W -1
                     , CAM_CLIP_Y + CAM_CLIP_H -1
                     , DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()){
    putStringOnLcd("Clip and Reize Error:" + String(err), ILI9341_RED);
    return;
  }

  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* tmp = (uint16_t*)small.getImgBuff();

  float* dnnbuf = input.data();

  for (int n = 0; n < DNN_IMG_W * DNN_IMG_H; ++n) {
    dnnbuf[n] = (float)((tmp[n] & 0xF800) >> 11) / 31.0;
    dnnbuf[n + DNN_IMG_W * DNN_IMG_H] = (float)((tmp[n] & 0x07E0) >> 5) / 63.0;
    dnnbuf[n + 2 * DNN_IMG_W * DNN_IMG_H] = (float)(tmp[n] & 0x001F) / 31.0;
  }
  
  String gStrResult = "?";
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  
  if (index < 6) {
    gStrResult = String(label[index]) + String(":") + String(output[index]);
  } else {
    gStrResult = String("?:") + String(output[index]);
  }
  Serial.println(gStrResult);

  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff(); 

  drawBox(imgBuf); 
  tft.drawRGBBitmap(0, 0, (uint16_t *)img.getImgBuff(), 320, 224);
  putStringOnLcd(gStrResult, ILI9341_YELLOW);
}


void setup() {   
  Serial.begin(115200);
 
  tft.begin();
  tft.setRotation(3);

  while (!theSD.begin()) { putStringOnLcd("Insert SD card", ILI9341_RED); }
  
  File nnbfile = theSD.open("6_56_30kb_470.nnb");
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    putStringOnLcd("dnnrt.begin failed" + String(ret), ILI9341_RED);
    return;
  }

  theCamera.begin();
  theCamera.startStreaming(true, CamCB);
}

void loop() { }