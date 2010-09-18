/*! ====================================================================================================================
 * \file
    TComDynamicArray.h
 *  \brief
    Copyright information.
 *  \par Copyright statements
    HEVC (JCTVC cfp)

    This software, including its corresponding source code, object code and executable, may only be used for
    (1) research purposes or (2) for evaluation for standardisation purposes within the joint collaborative team on
    video coding for HEVC , provided that this copyright notice and this corresponding notice appear in all copies,
    and that the name of Research in Motion Limited not be used in advertising or publicity without specific, written
    prior permission.  This software, as defined above, is provided as a proof-of-concept and for demonstration
    purposes only; there is no representation about the suitability of this software, as defined above, for any purpose.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
    USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    TRADEMARKS, Product and company names mentioned herein may be the trademarks of their respective owners.
    Any rights not expressly granted herein are reserved.

    Copyright (C) 2010 by Research in Motion Limited, Canada
    All rights reserved.

 *  \par Full Contact Information
    Standards & Licensing Department      (standards-ipr@rim.com)
    Research in Motion
    122 West John Carpenter Parkway
    Irving, TX 75039, USA

 * ====================================================================================================================
 */


#ifndef DYNAMIC_ARRAY
#define DYNAMIC_ARRAY

#include "TypeDef.h"

// Templates the C way

#define DYNAMICARRAY(type)              \
class DynamicExtent ## type {           \
    friend class DynamicArray ## type;  \
                                        \
    UInt size;                          \
    type *data;                         \
    DynamicExtent ## type *next;        \
                                        \
    DynamicExtent ## type (UInt size) { \
        this->size = size;              \
        data = new type[size];          \
        next = 0;                       \
    }                                   \
                                        \
    ~DynamicExtent ## type () {         \
        delete next;                    \
        delete [] data;                 \
    }                                   \
};                                      \
                                        \
class DynamicArray ## type {                   \
public:                                        \
    type & operator[] (UInt index) {           \
        if (index >= totalCount) {             \
        }                                      \
        UInt size = 0;                         \
        DynamicExtent ## type *ext = headExt;  \
        while (size + ext->size <= index) {    \
            size += ext->size;                 \
            ext = ext->next;                   \
        }                                      \
        return ext->data[index - size];        \
    }                                          \
                                               \
    DynamicArray ## type & operator<< (type value) {                   \
        if (writePos == writeExt->size) {                              \
            writeExt->next = new DynamicExtent##type(totalSize / 2);   \
            totalSize += totalSize / 2;        \
            writeExt = writeExt->next;         \
            writePos = 0;                      \
        }                                      \
        writeExt->data[writePos++] = value;    \
        ++totalCount;                          \
        return *this;                          \
    }                                          \
                                               \
    DynamicArray ## type & operator>> (type &value) {                  \
        if (readExt == writeExt && readPos == writePos) {              \
        }                                      \
        if (readPos == readExt->size) {        \
            readExt = readExt->next;           \
            readPos = 0;                       \
        }                                      \
        value = readExt->data[readPos++];      \
        return *this;                          \
    }                                          \
                                               \
    DynamicArray ## type () {                  \
        readExt = writeExt =                   \
        headExt = new DynamicExtent ## type(totalSize = 1000);         \
        readPos = writePos = totalCount = 0;   \
    }                                          \
    ~DynamicArray ## type () {                 \
        delete headExt;                        \
    }                                          \
                                               \
    void clear () {                            \
        delete headExt;                        \
        readExt = writeExt =                   \
        headExt = new DynamicExtent ## type(totalSize = 1000);         \
        readPos = writePos = totalCount = 0;   \
    }                                          \
                                               \
private:                                       \
    DynamicExtent ## type *headExt;            \
    DynamicExtent ## type *readExt, *writeExt; \
    UInt readPos, writePos, totalSize, totalCount;                     \
}

DYNAMICARRAY(UInt);
DYNAMICARRAY(UChar);


#endif

