/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __PacketHighResolutionMap_hpp__
#define __PacketHighResolutionMap_hpp__

#include <lcm/lcm_coretypes.h>



class PacketHighResolutionMap
{
    public:
        float      gpsx[250];

        float      gpsy[250];

        float      curve[250];

        uint8_t    MapFlag[250];

        float      lane1x[40];

        float      lane1y[40];

        float      lane2x[40];

        float      lane2y[40];

        float      lane3x[40];

        float      lane3y[40];

        float      lane4x[40];

        float      lane4y[40];

        float      ibcontourx[3000];

        float      ibcontoury[3000];

        float      pointx[50];

        float      pointy[50];

        int16_t    pointxstyle[50];

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "PacketHighResolutionMap"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int PacketHighResolutionMap::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int PacketHighResolutionMap::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int PacketHighResolutionMap::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t PacketHighResolutionMap::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* PacketHighResolutionMap::getTypeName()
{
    return "PacketHighResolutionMap";
}

int PacketHighResolutionMap::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gpsx[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gpsy[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->curve[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->MapFlag[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane1x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane1y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane2x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane2y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane3x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane3y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane4x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->lane4y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibcontourx[0], 3000);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibcontoury[0], 3000);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->pointx[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->pointy[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->pointxstyle[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int PacketHighResolutionMap::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gpsx[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gpsy[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->curve[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->MapFlag[0], 250);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane1x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane1y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane2x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane2y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane3x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane3y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane4x[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->lane4y[0], 40);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibcontourx[0], 3000);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibcontoury[0], 3000);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->pointx[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->pointy[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->pointxstyle[0], 50);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int PacketHighResolutionMap::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __float_encoded_array_size(NULL, 250);
    enc_size += __float_encoded_array_size(NULL, 250);
    enc_size += __float_encoded_array_size(NULL, 250);
    enc_size += __byte_encoded_array_size(NULL, 250);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 40);
    enc_size += __float_encoded_array_size(NULL, 3000);
    enc_size += __float_encoded_array_size(NULL, 3000);
    enc_size += __float_encoded_array_size(NULL, 50);
    enc_size += __float_encoded_array_size(NULL, 50);
    enc_size += __int16_t_encoded_array_size(NULL, 50);
    return enc_size;
}

uint64_t PacketHighResolutionMap::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xdaeabfb4050ec7d9LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
