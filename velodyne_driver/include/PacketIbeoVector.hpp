/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __PacketIbeoVector_hpp__
#define __PacketIbeoVector_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>


class PacketIbeoVector
{
    public:
        int8_t     index;

        int16_t    lengthOfObject;

        int16_t    lengthOfContour;

        std::vector< float > ibx;

        std::vector< float > iby;

        std::vector< float > ibw;

        std::vector< float > ibl;

        std::vector< float > ibvx;

        std::vector< float > ibvy;

        std::vector< float > ibvabx;

        std::vector< float > ibvaby;

        std::vector< int16_t > ibcontournum;

        std::vector< float > ibcontourx;

        std::vector< float > ibcontoury;

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
         * Returns "PacketIbeoVector"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int PacketIbeoVector::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int PacketIbeoVector::decode(const void *buf, int offset, int maxlen)
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

int PacketIbeoVector::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t PacketIbeoVector::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* PacketIbeoVector::getTypeName()
{
    return "PacketIbeoVector";
}

int PacketIbeoVector::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->index, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->lengthOfObject, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->lengthOfContour, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->iby[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibw[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibl[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibvx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibvy[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibvabx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibvaby[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject > 0) {
        tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->ibcontournum[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfContour > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibcontourx[0], this->lengthOfContour);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfContour > 0) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->ibcontoury[0], this->lengthOfContour);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int PacketIbeoVector::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->index, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->lengthOfObject, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->lengthOfContour, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->lengthOfObject) {
        this->ibx.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->iby.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->iby[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibw.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibw[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibl.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibl[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibvx.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibvx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibvy.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibvy[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibvabx.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibvabx[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibvaby.resize(this->lengthOfObject);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibvaby[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfObject) {
        this->ibcontournum.resize(this->lengthOfObject);
        tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->ibcontournum[0], this->lengthOfObject);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfContour) {
        this->ibcontourx.resize(this->lengthOfContour);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibcontourx[0], this->lengthOfContour);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->lengthOfContour) {
        this->ibcontoury.resize(this->lengthOfContour);
        tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->ibcontoury[0], this->lengthOfContour);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int PacketIbeoVector::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __int16_t_encoded_array_size(NULL, this->lengthOfObject);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfContour);
    enc_size += __float_encoded_array_size(NULL, this->lengthOfContour);
    return enc_size;
}

uint64_t PacketIbeoVector::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xc1eb96397c43be35LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
