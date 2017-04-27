#!/usr/bin/env python

from __future__ import print_function

"""Script to generate a .yuneec file from a .px4 file.

This script uses AES-128 CBC to encrypt the image including the
length and CRC32 sum. The private key needs to be supplied to
the script.

This script requires the px_uploader.py script to access the .px4
firmware file.

This script is to be kept Python 2 and 3 compatible.

Author: Julian Oes <julian@yuneecresearch.com>
"""

from Crypto.Cipher import AES
import binascii
import argparse
import px_uploader
import json
import base64
import zlib
import struct
import os


def pad(blob, chunk_size):
    """Adding padding to blob up to chunk size."""
    num_bytes = len(blob)
    padding = chunk_size - (num_bytes % chunk_size)
    return blob + padding*b"\x00"


def encrypt(cleartext, iv, key):
    """Encrypt with 128bit AES CBC."""
    cleartext = pad(cleartext, 16)
    mode = AES.MODE_CBC
    encryptor = AES.new(key, mode, IV=iv)
    return encryptor.encrypt(cleartext)


def decrypt(ciphertext, iv, key):
    """Decrypt with 128bit AES CBC."""
    mode = AES.MODE_CBC
    decryptor = AES.new(key, mode, IV=iv)
    return decryptor.decrypt(ciphertext)


def print_image(image):
    """Show contents of image.

    This can be used to manually compare what has been written
    to the flash using e.g. `gdb: x 0x0800c000`.
    """
    base = 0x0800c000
    for i in range(0, len(image), 4):
        print("0x%02x: 0x%02x%02x%02x%02x" %
              (base+i, image[i+3], image[i+2], image[i+1], image[i]))


def main():
    """Generate the .yuneec file from .px4 file."""
    # Parse commandline
    parser = argparse.ArgumentParser(
        description="Generate .yuneec file using .px4 JSON.")
    parser.add_argument("px4", action="store",
                        help="the existing .px4 file ")
    parser.add_argument("yuneec", action="store",
                        help="the .yuneec file to create")
    parser.add_argument("key", action="store",
                        help="the 16 byte AES key as 32 hex chars")
    args = parser.parse_args()

    # Check arguments
    if not os.path.isfile(args.px4):
        parser.error("File {} not found.".format(args.px4))
    if len(args.key) != 32:
        parser.error("Key needs to be 32 hex chars")

    # The key is passed as a hex string.
    key = binascii.unhexlify(args.key)

    # Load the .px4 JSON file.
    fw = px_uploader.firmware(args.px4)

    # The initialisation vector is generated randomly every time.
    iv = os.urandom(16)

    # The CRC is only over the actual image and not over the whole
    # flash area.
    len_image = len(fw.image)
    crc32_image = fw.crc(len_image)

    # Header is 4x 32bit, the last two words are reserved.
    #
    # The CRC sum and the length of the sent image needs to be encrypted
    # as well and sent along in a 16 byte header.
    #
    # The header is:
    # uint32_t: image length in bytes
    # uint32_t: crc32 over image length
    # uint32_t: reserved 1
    # uint32_t: reserved 2
    header = struct.pack('IIII', len_image, crc32_image, 0, 0)

    # Add header to image and encrypt both.
    encrypted = encrypt(bytes(header + fw.image), iv, key)

    # print_image(fw.image)

    # Replace image with encrypted one
    fw.desc['image_encrypted'] = \
        base64.b64encode(zlib.compress(encrypted, 9)).decode("utf-8")
    fw.desc['image'] = ""
    fw.desc['image_encrypted_iv'] = base64.b64encode(iv).decode("utf-8")

    # print("len image: {}, crc32: {}".format(len_image, crc32_image))
    # print("len encrypted: {}".format(len(encrypted)))
    # print("len base64 encrypted: {}".format(len(fw.desc['image_encrypted'])))

    with open(args.yuneec, "w") as f:
        json.dump(fw.desc, f, indent=4)


if __name__ == '__main__':
    main()
