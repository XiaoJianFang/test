
/* in case someone actually tries to compile this */

/* example.c - an example of using libpng
 * Last changed in libpng 1.6.15 [November 20, 2014]
 * Maintained 1998-2014 Glenn Randers-Pehrson
 * Maintained 1996, 1997 Andreas Dilger)
 * Written 1995, 1996 Guy Eric Schalnat, Group 42, Inc.)
 * To the extent possible under law, the authors have waived
 * all copyright and related or neighboring rights to this file.
 * This work is published from: United States.
 */

/* This is an example of how to use libpng to read and write PNG files.
 * The file libpng-manual.txt is much more verbose then this.  If you have not
 * read it, do so first.  This was designed to be a starting point of an
 * implementation.  This is not officially part of libpng, is hereby placed
 * in the public domain, and therefore does not require a copyright notice.
 *
 * This file does not currently compile, because it is missing certain
 * parts, like allocating memory to hold an image.  You will have to
 * supply these parts to get it to compile.  For an example of a minimal
 * working PNG reader/writer, see pngtest.c, included in this distribution;
 * see also the programs in the contrib directory.
 */

/* The simple, but restricted, approach to reading a PNG file or data stream
 * just requires two function calls, as in the following complete program.
 * Writing a file just needs one function call, so long as the data has an
 * appropriate layout.
 *
 * The following code reads PNG image data from a file and writes it, in a
 * potentially new format, to a new file.  While this code will compile there is
 * minimal (insufficient) error checking; for a more realistic version look at
 * contrib/examples/pngtopng.c
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../pnglib/png.h"
#include "../zlib-1.2.8/zlib.h"
#define PNG_BYTES_TO_CHECK 4

//#define _CRT_SECURE_NO_WARNINGS

int loadpngimage( char *filepath)
{
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr;
	png_bytep* row_pointers;
	char buf[PNG_BYTES_TO_CHECK];
	int w, h, x, y, temp, color_type;

	fp = fopen(filepath, "rb");
	if (fp == NULL) 
	{
		return -1 /* 返回值 */;
	}

	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
	info_ptr = png_create_info_struct(png_ptr);

	setjmp(png_jmpbuf(png_ptr));
	/* 读取PNG_BYTES_TO_CHECK个字节的数据 */
	temp = fread(buf, 1, PNG_BYTES_TO_CHECK, fp);
	/* 若读到的数据并没有PNG_BYTES_TO_CHECK个字节 */
	if (temp < PNG_BYTES_TO_CHECK) 
	{
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
		return  -1/* 返回值 */;
	}
	/* 检测数据是否为PNG的签名 */
	temp = png_sig_cmp((png_bytep)buf, (png_size_t)0, PNG_BYTES_TO_CHECK);
	/* 如果不是PNG的签名，则说明该文件不是PNG文件 */
	if (temp != 0)
	{
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
		return -1/* 返回值 */;
	}

	/* 复位文件指针 */
	rewind(fp);
	/* 开始读文件 */
	png_init_io(png_ptr, fp);
	/* 读取PNG图片信息 */
	png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_EXPAND, 0);
	/* 获取图像的色彩类型 */
	color_type = png_get_color_type(png_ptr, info_ptr);
	/* 获取图像的宽高 */
	w = png_get_image_width(png_ptr, info_ptr);
	h = png_get_image_height(png_ptr, info_ptr);
	/* 获取图像的所有行像素数据，row_pointers里边就是rgba数据 */
	row_pointers = png_get_rows(png_ptr, info_ptr);
	/* 根据不同的色彩类型进行相应处理 */
	switch (color_type) {
	case PNG_COLOR_TYPE_RGB_ALPHA:
		//for (y = 0; y < h; ++y) 
		//{
		//	for (x = 0; x < w * 4;) 
		//	{
		//		/* 以下是RGBA数据，需要自己补充代码，保存RGBA数据 */
		//		/* 目标内存 */ 
		//		= row_pointers[y][x++]; // red  
		//		/* 目标内存 */ 
		//		= row_pointers[y][x++]; // green  
		//		/* 目标内存 */ 
		//		= row_pointers[y][x++]; // blue  
		//		/* 目标内存 */ 
		//		= row_pointers[y][x++]; // alpha  
		//	}
		//}
		break;

	case PNG_COLOR_TYPE_RGB:
		//for (y = 0; y < h; ++y) 
		//{
		//	for (x = 0; x < w * 3;) 
		//	{
		//		/* 目标内存 */
		//		= row_pointers[y][x++]; // red  
		//		/* 目标内存 */
		//		= row_pointers[y][x++]; // green  
		//		/* 目标内存 */ 
		//		= row_pointers[y][x++]; // blue  
		//	}
		//}
		break;
		/* 其它色彩类型的图像就不读了 */
	default:
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, 0);
		return  -1/* 返回值 */;
	}
	png_destroy_read_struct(&png_ptr, &info_ptr, 0);
	return 0;
}



