/**************************************************************************
 *
 * FILE NAME:			bitmapDb.h
 * FILE DESCRIPTION:	Database of all bitmaps in the system
 *
 * FILE CREATION DATE:	24-07-2009
 *
 *==========================================================================
 *
 * Modification history:
 * --------------------
 * 01a,24jul09 erd written
 *
 ***************************************************************************/

#ifndef __BITMAP_DB_H_
#define __BITMAP_DB_H_

#include <stdint.h>

// ==========================================================================
// structure definition
// ==========================================================================

// This structure describes a single character's display information
typedef struct
{
	uint8_t widthBits;					// width, in bits (or pixels), of the character
	uint16_t offset;					// offset of the character's bitmap, in bytes, into the the FONT_INFO's data array
} FONT_CHAR_INFO;

typedef struct
{
    uint8_t 		    startChar;		// the first character in the font (e.g. in charInfo and data)
	uint8_t 		    endChar;		// the last character in the font
	const FONT_CHAR_INFO *charInfo;
} FONT_CHAR_INFO_LOOKUP;

// Describes a single font
typedef struct
{
	uint8_t 		    heightChar;	// height, in pages (8 pixels), of the font's characters
	uint8_t 		    startChar;		// the first character in the font (e.g. in charInfo and data)
	uint8_t 		    endChar;		// the last character in the font
	uint8_t 		    spacePixels;	// number of pixels that a space character takes up
	const FONT_CHAR_INFO_LOOKUP *BlockLookup;
	const FONT_CHAR_INFO *charInfo;		// pointer to array of char information
	const uint8_t *data;			// pointer to generated array of character visual representation

} FONT_INFO;

extern const FONT_INFO arial_8ptFontInfo;
extern const FONT_INFO arial_11ptFontInfo;
extern const FONT_INFO arial_16ptFontInfo;
extern const FONT_INFO impact_28ptFontInfo;
extern const FONT_INFO squaredeal_28ptFontInfo;
extern const FONT_INFO squaredeal_24ptFontInfo;
extern const FONT_INFO steelfishRg_28ptFontInfo;

#endif /* __BITMAP_DB_H_ */


