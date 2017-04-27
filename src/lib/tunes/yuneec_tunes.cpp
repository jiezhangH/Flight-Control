#include "tunes.h"

// initialise default tunes
const char *Tunes::_default_tunes[] = {
	"", // empty to align with the index
	"MNT255L32O2<CDEFGAB>CDEFGAB>CDE", // startup tune
	"MBT200a8a8a8PaaaP", // ERROR tone
	"MFT200e8a8a", // Notify Positive tone
	"MFT200e8e", // Notify Neutral tone
	"MFT200e8c8e8c8e8c8", // Notify Negative tone
	"MNT75L1O2G", //arming warning
	"MBNT100a8", //battery warning slow
	"MBNT255a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8a8", //battery warning fast
	"MFT255L4AAAL1F#", //gps warning slow
	"MFT255L4<<<BAP", // arming failure tune
	"MFT255L16agagagag", // parachute release
	"MFT255L8ddd#d#eeff", // ekf warning
	"MFT255L4gf#fed#d", // baro warning
	"MFT100a8", // single beep
	"MFT100L4>G#6A#6B#4", // home set tune
	"MFT255L32O2EDC<BAGFEDC<BAGFEDC>", // shutdown tune
};

// set default_tunes array size
const unsigned int Tunes::_default_tunes_size =  sizeof(_default_tunes) / sizeof(_default_tunes[0]);
