/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "version.h"

#include "build_git_version.h" //generated from build_git_version.h.in

#include <string.h>

#if !defined(CONFIG_CDCACM_PRODUCTID)
# define CONFIG_CDCACM_PRODUCTID 0
#endif

#if defined(__PX4_LINUX)
#include <sys/utsname.h>
#endif

// dev >= 0
// alpha >= 64
// beta >= 128
// release candidate >= 192
// release == 255
enum FIRMWARE_TYPE {
	FIRMWARE_TYPE_DEV = 0,
	FIRMWARE_TYPE_ALPHA = 64,
	FIRMWARE_TYPE_BETA = 128,
	FIRMWARE_TYPE_RC = 192,
	FIRMWARE_TYPE_RELEASE = 255
};

/**
 * Convert a version tag string to a number
 * @param tag version tag in one of the following forms:
 *            - dev: v1.4.0rc3-7-g7e282f57
 *            - rc: v1.4.0rc4
 *            - release: v1.4.0
 *            - linux: 7.9.3
 * @return version in the form 0xAABBCCTT (AA: Major, BB: Minor, CC: Patch, TT Type @see FIRMWARE_TYPE)
 */
static uint32_t version_tag_to_number(const char *tag)
{
	uint32_t version_number = 0;

	// extract version numbers
	int16_t buffer = -1;
	size_t buffer_counter = 0;
	char version[3] = {0, 0, 0};
	int firmware_type = FIRMWARE_TYPE_RELEASE;

	for (size_t i = 0; i < strlen(tag); i++) {
		if (tag[i] == 'r' && i < strlen(tag) - 1 && tag[i + 1] == 'c') {
			firmware_type = FIRMWARE_TYPE_RC;

		} else if (tag[i] == 'p') {
			firmware_type = FIRMWARE_TYPE_ALPHA;

		} else if (tag[i] == 't') {
			firmware_type = FIRMWARE_TYPE_BETA;

		} else if (tag[i] == 'v' && i > 0) {
			firmware_type = FIRMWARE_TYPE_DEV;
		}

		if (buffer_counter > 2) {
			continue;
		}

		if (tag[i] >= '0' && tag[i] <= '9') {
			buffer = (buffer == -1) ? 0 : buffer;
			buffer = buffer * 10 + (tag[i] - '0');

		} else {
			if (buffer >= 0) {
				version[buffer_counter] = buffer;

				buffer_counter++;
			}

			buffer = -1;
		}
	}

	if (buffer >= 0) {
		version[buffer_counter] = buffer;
		buffer_counter++;
	}

	version_number = (version[0] << 8 * 3) |
			 (version[1] << 8 * 2) |
			 (version[2] << 8 * 1) | firmware_type;

	return version_number;
}

uint32_t px4_firmware_version(void)
{
	return version_tag_to_number(PX4_GIT_TAG_STR);
}

/**
 * Convert a version tag string to a vendor version number
 * @param tag version tag in one of the following forms:
 *            - vendor: v1.4.0-0.2.0
 *			  - vendor-dev: v1.4.0-7.2.3-7-g8fdf1db
 *            - dev: v1.4.0rc3-7-g7e282f57
 *            - rc: v1.4.0rc4
 *            - release: v1.4.0
 *            - linux: 7.9.3
 * @return version in the form 0xAABBCC (AA: Major, BB: Minor, CC: Patch)
 */
static uint32_t version_tag_to_vendor_version_number(const char *tag)
{
	uint32_t version_number = 0;

	// extract version numbers
	int16_t buffer = -1;
	size_t buffer_counter = 0;
	char version[6] = {0, 0, 0, 0, 0, 0};

	for (size_t i = 0; i < strlen(tag); i++) {
		if (buffer_counter > 5) {
			continue;
		}

		if (tag[i] >= '0' && tag[i] <= '9') {
			buffer = (buffer == -1) ? 0 : buffer;
			buffer = buffer * 10 + (tag[i] - '0');

		} else {
			if (buffer >= 0) {
				version[buffer_counter] = buffer;
				buffer_counter++;
			}

			buffer = -1;
		}
	}

	if (buffer >= 0) {
		version[buffer_counter] = buffer;
		buffer_counter++;
	}

	version_number = (version[3] << 8 * 2) |
			 (version[4] << 8 * 1) |
			 (version[5] << 8 * 0);

	return version_number;
}

uint32_t px4_firmware_vendor_version(void)
{
	return version_tag_to_vendor_version_number(PX4_GIT_TAG_STR);
}

uint32_t px4_board_version(void)
{
#if defined(__PX4_NUTTX)
	return CONFIG_CDCACM_PRODUCTID;
#else
	return 1;
#endif
}

uint32_t px4_os_version(void)
{
#if defined(__PX4_DARWIN)
	return 0; //TODO: implement version for Darwin
#elif defined(__PX4_LINUX)
	struct utsname name;

	if (uname(&name) == 0) {
		char *c = name.release;

		// cut the part after the first '-'
		while (*c && *c != '-') {
			++c;
		}

		*c = 0;
		return version_tag_to_number(name.release);

	} else {
		return 0;
	}

#elif defined(__PX4_QURT)
	return 0; //TODO: implement version for QuRT
#elif defined(__PX4_NUTTX)
	return version_tag_to_number("v7.18.0"); //TODO: get correct version
#else
# error "px4_os_version not implemented for current OS"
#endif
}

const char *px4_os_version_string(void)
{
#if defined(__PX4_NUTTX)
	return NULL; //TODO: get NuttX git tag as string
#else
	return NULL;
#endif
}

const char *px4_os_name(void)
{
#if defined(__PX4_DARWIN)
	return "Darwin";
#elif defined(__PX4_LINUX)
	return "Linux";
#elif defined(__PX4_QURT)
	return "QuRT";
#elif defined(__PX4_NUTTX)
	return "NuttX";
#else
# error "px4_os_name not implemented for current OS"
#endif
}

const char *px4_toolchain_name(void)
{
#if defined(__clang__)
	return "Clang/LLVM";
#elif defined(__ICC) || defined(__INTEL_COMPILER)
	return "Intel ICC";
#elif defined(__GNUC__) || defined(__GNUG__)
	return "GNU GCC";
#elif defined(_MSC_VER)
	return "MS Visual Studio";
#else
	return "Unknown";
#endif
}

const char *px4_toolchain_version(void)
{
#ifdef __VERSION__
	return __VERSION__;
#else
	return "";
#endif
}

const char *px4_firmware_version_string(void)
{
	return PX4_GIT_VERSION_STR;
}

uint64_t px4_firmware_version_binary(void)
{
	return PX4_GIT_VERSION_BINARY;
}

uint64_t px4_os_version_binary(void)
{
	//TODO: get NuttX version via git tag
	return 0;
}

