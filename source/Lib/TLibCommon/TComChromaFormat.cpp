/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2013, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "TComChromaFormat.h"
#include "TComPic.h"
#include "TComDataCU.h"
#include "TComTrQuant.h"


//----------------------------------------------------------------------------------------------------------------------

Void setQPforQuant(       QpParam      &result,
                    const Int           qpy,
                    const ChannelType   chType,
                    const Int           qpBdOffset,
                    const Int           chromaQPOffset,
                    const ChromaFormat  chFmt,
                    const Bool          useTransformSkip )
{
  Int baseQp     = MAX_INT;
  Int adjustedQp = MAX_INT;

  //------------------------------------------------

  if(isLuma(chType))
  {
    baseQp     = qpy + qpBdOffset;
    adjustedQp = baseQp;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }

    //adjustment for chroma 4:2:2
    adjustedQp = ((chFmt == CHROMA_422) && !useTransformSkip) ? (baseQp + 3) : baseQp;
  }

  //------------------------------------------------

  result.setBaseQp    (QpParam::QpData(baseQp,     (baseQp     / 6), (baseQp     % 6)));
  result.setAdjustedQp(QpParam::QpData(adjustedQp, (adjustedQp / 6), (adjustedQp % 6)));
}


//----------------------------------------------------------------------------------------------------------------------

Void getTUEntropyCodingParameters(      TUEntropyCodingParameters &result,
                                  const UInt                       uiScanIdx,
                                  const UInt                       width,
                                  const UInt                       height,
                                  const ComponentID                component,
                                  const ChromaFormat               format)
{
  //------------------------------------------------

  //set the local parameters

  const UInt        log2BlockWidth  = g_aucConvertToBit[width]  + 2;
  const UInt        log2BlockHeight = g_aucConvertToBit[height] + 2;
  const ChannelType channelType     = toChannelType(component);

  result.scanType = COEFF_SCAN_TYPE(uiScanIdx);
  
  //------------------------------------------------

  //set the group layout

  result.widthInGroups  = width  >> MLS_CG_LOG2_WIDTH;
  result.heightInGroups = height >> MLS_CG_LOG2_HEIGHT;

  //------------------------------------------------

  //set the scan orders

  const UInt log2WidthInGroups  = g_aucConvertToBit[result.widthInGroups  * 4];
  const UInt log2HeightInGroups = g_aucConvertToBit[result.heightInGroups * 4];

  result.scan   = g_scanOrder[ SCAN_GROUPED_4x4 ][ result.scanType ][ log2BlockWidth    ][ log2BlockHeight    ];
  result.scanCG = g_scanOrder[ SCAN_UNGROUPED   ][ result.scanType ][ log2WidthInGroups ][ log2HeightInGroups ];

  //------------------------------------------------

  //set the significance map context selection parameters

  if ((width == 4) && (height == 4))
  {
    result.firstSignificanceMapContext = significanceMapContextSetStart[channelType][CONTEXT_TYPE_4x4];
  }
  else if ((width == 8) && (height == 8))
  {
    result.firstSignificanceMapContext = significanceMapContextSetStart[channelType][CONTEXT_TYPE_8x8];
    if (result.scanType != SCAN_DIAG) result.firstSignificanceMapContext += nonDiagonalScan8x8ContextOffset[channelType];
  }
  else
  {
    result.firstSignificanceMapContext = significanceMapContextSetStart[channelType][CONTEXT_TYPE_NxN];
  }

  //------------------------------------------------
}


//----------------------------------------------------------------------------------------------------------------------