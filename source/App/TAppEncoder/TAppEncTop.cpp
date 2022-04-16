/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>

#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

using namespace std;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
}

TAppEncTop::~TAppEncTop()
{
}

Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;

  vps.setMaxTLayers                                               ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                                                ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                                         ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering                                     ( m_maxDecPicBuffering[i], i );
  }


  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile                                           ( m_profile);
  m_cTEncTop.setLevel                                             ( m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag                           ( m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cTEncTop.setBitDepthConstraintValue                           ( m_bitDepthConstraint );
  m_cTEncTop.setChromaFormatConstraintValue                       ( m_chromaFormatConstraint );
  m_cTEncTop.setIntraConstraintFlag                               ( m_intraConstraintFlag );
  m_cTEncTop.setOnePictureOnlyConstraintFlag                      ( m_onePictureOnlyConstraintFlag );
  m_cTEncTop.setLowerBitRateConstraintFlag                        ( m_lowerBitRateConstraintFlag );

  m_cTEncTop.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cTEncTop.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cTEncTop.setPrintSequenceMSE                                  ( m_printSequenceMSE);
  m_cTEncTop.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_cTEncTop.setFrameRate                                         ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                                         ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                                       ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                                      ( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow                                 ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTop.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                                       ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                                           ( m_iGOPSize );
  m_cTEncTop.setGopList                                           ( m_GOPList );
  m_cTEncTop.setExtraRPSs                                         ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_cTEncTop.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_cTEncTop.setQP                                                ( m_iQP );

  m_cTEncTop.setPad                                               ( m_aiPad );

  m_cTEncTop.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );

  m_cTEncTop.setMaxTempLayer                                      ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_cTEncTop.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_cTEncTop.setSearchRange                                       ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_cTEncTop.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_cTEncTop.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_cTEncTop.setMinSearchWindow                                   ( m_minSearchWindow );
  m_cTEncTop.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                                     ( m_iMaxCuDQPDepth  );
  m_cTEncTop.setDiffCuChromaQpOffsetDepth                         ( m_diffCuChromaQpOffsetDepth );
  m_cTEncTop.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset                                  ( m_crQpOffset  );

  m_cTEncTop.setChromaFormatIdc                                   ( m_chromaFormatIDC  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect                                  ( m_bUseAdaptQpSelect   );
#endif

  m_cTEncTop.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_cTEncTop.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
  m_cTEncTop.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_cTEncTop.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_cTEncTop.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
  m_cTEncTop.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
  m_cTEncTop.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_cTEncTop.setUseASR                                            ( m_bUseASR      );
  m_cTEncTop.setUseHADME                                          ( m_bUseHADME    );
  m_cTEncTop.setdQPs                                              ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                                           ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                                         ( m_useRDOQTS   );
#if T0196_SELECTIVE_RDOQ
  m_cTEncTop.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
#endif
  m_cTEncTop.setRDpenalty                                         ( m_rdPenalty );
  m_cTEncTop.setMaxCUWidth                                        ( m_uiMaxCUWidth );
  m_cTEncTop.setMaxCUHeight                                       ( m_uiMaxCUHeight );
  m_cTEncTop.setMaxTotalCUDepth                                   ( m_uiMaxTotalCUDepth );
  m_cTEncTop.setLog2DiffMaxMinCodingBlockSize                     ( m_uiLog2DiffMaxMinCodingBlockSize );
  m_cTEncTop.setQuadtreeTULog2MaxSize                             ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize                             ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter                           ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra                           ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_cTEncTop.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cTEncTop.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cTEncTop.setCrossComponentPredictionEnabledFlag               ( m_crossComponentPredictionEnabledFlag );
  m_cTEncTop.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cTEncTop.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_LUMA  , m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA]   );
  m_cTEncTop.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  m_cTEncTop.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_cTEncTop.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_cTEncTop.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_cTEncTop.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_cTEncTop.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_cTEncTop.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_cTEncTop.setRdpcmEnabledFlag                                ( RDPCMSignallingMode(signallingModeIndex), m_rdpcmEnabledFlag[signallingModeIndex]);
  }
  m_cTEncTop.setUseConstrainedIntraPred                           ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_cTEncTop.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_cTEncTop.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_cTEncTop.setPCMLog2MinSize                                    ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                                            ( m_usePCM );

  // set internal bit-depth and constants
  for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_cTEncTop.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_cTEncTop.setPCMBitDepth((ChannelType)channelType, m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[channelType] : m_internalBitDepth[channelType]);
  }

  m_cTEncTop.setPCMLog2MaxSize                                    ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand                                   ( m_maxNumMergeCand );


  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                                             ( m_useWeightedPred     );
  m_cTEncTop.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2                      ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode                                         ( m_sliceMode );
  m_cTEncTop.setSliceArgument                                     ( m_sliceArgument );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode                                  ( m_sliceSegmentMode );
  m_cTEncTop.setSliceSegmentArgument                              ( m_sliceSegmentArgument );

  if(m_sliceMode == NO_SLICES )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag                          ( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO                                            ( m_bUseSAO );
  m_cTEncTop.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_cTEncTop.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_cTEncTop.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_cTEncTop.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoCtuBoundary                                    ( m_saoCtuBoundary);
  m_cTEncTop.setPCMInputBitDepthFlag                              ( m_bPCMInputBitDepthFlag);
  m_cTEncTop.setPCMFilterDisableFlag                              ( m_bPCMFilterDisableFlag);

  m_cTEncTop.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );
  m_cTEncTop.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
  m_cTEncTop.setRecoveryPointSEIEnabled                           ( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled                         ( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId                                   ( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag                           ( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag                      ( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth                           ( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth                              ( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID                                     ( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue                                    ( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue                                    ( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint                             ( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth                                ( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva                         ( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots                                   ( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue                             ( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue                            ( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc                           ( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue                         ( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureIndexIdc                            ( m_exposureIndexIdc );
  m_cTEncTop.setTMISEIExposureIndexValue                          ( m_exposureIndexValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag           ( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator          ( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc           ( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite                     ( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel                     ( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue              ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue              ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue             ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setChromaResamplingFilterHintEnabled                 ( m_chromaResamplingFilterSEIenabled );
  m_cTEncTop.setChromaResamplingHorFilterIdc                      ( m_chromaResamplingHorFilterIdc );
  m_cTEncTop.setChromaResamplingVerFilterIdc                      ( m_chromaResamplingVerFilterIdc );
  m_cTEncTop.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIEnabled    ( m_segmentedRectFramePackingSEIEnabled );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEICancel     ( m_segmentedRectFramePackingSEICancel );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIType       ( m_segmentedRectFramePackingSEIType );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIPersistence( m_segmentedRectFramePackingSEIPersistence );
  m_cTEncTop.setDisplayOrientationSEIAngle                        ( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled                     ( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled                 ( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setNoDisplaySEITLayer                                ( m_noDisplaySEITLayer );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_cTEncTop.setSOPDescriptionSEIEnabled                          ( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_cTEncTop.setTMCTSSEIEnabled                                   ( m_tmctsSEIEnabled );
  m_cTEncTop.setTimeCodeSEIEnabled                                ( m_timeCodeSEIEnabled );
  m_cTEncTop.setNumberOfTimeSets                                  ( m_timeCodeSEINumTs );
  for(Int i = 0; i < m_timeCodeSEINumTs; i++)
  {
    m_cTEncTop.setTimeSet(m_timeSetArray[i], i);
  }
  m_cTEncTop.setKneeSEIEnabled                                    ( m_kneeSEIEnabled );
  m_cTEncTop.setKneeSEIId                                         ( m_kneeSEIId );
  m_cTEncTop.setKneeSEICancelFlag                                 ( m_kneeSEICancelFlag );
  m_cTEncTop.setKneeSEIPersistenceFlag                            ( m_kneeSEIPersistenceFlag );
  m_cTEncTop.setKneeSEIInputDrange                                ( m_kneeSEIInputDrange );
  m_cTEncTop.setKneeSEIInputDispLuminance                         ( m_kneeSEIInputDispLuminance );
  m_cTEncTop.setKneeSEIOutputDrange                               ( m_kneeSEIOutputDrange );
  m_cTEncTop.setKneeSEIOutputDispLuminance                        ( m_kneeSEIOutputDispLuminance );
  m_cTEncTop.setKneeSEINumKneePointsMinus1                        ( m_kneeSEINumKneePointsMinus1 );
  m_cTEncTop.setKneeSEIInputKneePoint                             ( m_kneeSEIInputKneePoint );
  m_cTEncTop.setKneeSEIOutputKneePoint                            ( m_kneeSEIOutputKneePoint );
  m_cTEncTop.setColourRemapInfoSEIFileRoot                        ( m_colourRemapSEIFileRoot );
  m_cTEncTop.setMasteringDisplaySEI                               ( m_masteringDisplay );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  m_cTEncTop.setSEIAlternativeTransferCharacteristicsSEIEnable    ( m_preferredTransferCharacteristics>=0     );
  m_cTEncTop.setSEIPreferredTransferCharacteristics               ( UChar(m_preferredTransferCharacteristics) );
#endif

  m_cTEncTop.setTileUniformSpacingFlag                            ( m_tileUniformSpacingFlag );
  m_cTEncTop.setNumColumnsMinus1                                  ( m_numTileColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1                                     ( m_numTileRowsMinus1 );
  if(!m_tileUniformSpacingFlag)
  {
    m_cTEncTop.setColumnWidth                                     ( m_tileColumnWidth );
    m_cTEncTop.setRowHeight                                       ( m_tileRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount = (m_numTileRowsMinus1+1) * (m_numTileColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossTileBoundaryFlag                           ( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setEntropyCodingSyncEnabledFlag                      ( m_entropyCodingSyncEnabledFlag );
  m_cTEncTop.setTMVPModeId                                        ( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId                                  ( m_useScalingListId  );
  m_cTEncTop.setScalingListFileName                               ( m_scalingListFileName );
  m_cTEncTop.setSignHideFlag                                      ( m_signHideFlag);
  m_cTEncTop.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP                                         ( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP                                      ( m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
  m_cTEncTop.setCpbSaturationEnabled                              ( m_RCCpbSaturationEnabled );
  m_cTEncTop.setCpbSize                                           ( m_RCCpbSize );
  m_cTEncTop.setInitialCpbFullness                                ( m_RCInitialCpbFullness );
#endif
  m_cTEncTop.setTransquantBypassEnableFlag                        ( m_TransquantBypassEnableFlag );
  m_cTEncTop.setCUTransquantBypassFlagForceValue                  ( m_CUTransquantBypassFlagForce );
  m_cTEncTop.setCostMode                                          ( m_costMode );
  m_cTEncTop.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing                           ( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled                     ( m_activeParameterSetsSEIEnabled );
  m_cTEncTop.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_cTEncTop.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth                                          ( m_sarWidth );
  m_cTEncTop.setSarHeight                                         ( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag                        ( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat                                       ( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries                                   ( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag                       ( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow                              ( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag                         ( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag                       ( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1                          ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTop.setBitstreamRestrictionFlag                          ( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag                           ( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag                ( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc                         ( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom                               ( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom                              ( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal                         ( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical                           ( m_log2MaxMvLengthVertical );
  m_cTEncTop.setEfficientFieldIRAPEnabled                         ( m_bEfficientFieldIRAPEnabled );
  m_cTEncTop.setHarmonizeGopFirstFieldCoupleEnabled               ( m_bHarmonizeGopFirstFieldCoupleEnabled );

  m_cTEncTop.setSummaryOutFilename                                ( m_summaryOutFilename );
  m_cTEncTop.setSummaryPicFilenameBase                            ( m_summaryPicFilenameBase );
  m_cTEncTop.setSummaryVerboseness                                ( m_summaryVerboseness );

  //GENIUS

  m_cTEncTopSad.setVPS(&vps);

  m_cTEncTopSad.setProfile                                           ( m_profile);
  m_cTEncTopSad.setLevel                                             ( m_levelTier, m_level);
  m_cTEncTopSad.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cTEncTopSad.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cTEncTopSad.setNonPackedConstraintFlag                           ( m_nonPackedConstraintFlag);
  m_cTEncTopSad.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cTEncTopSad.setBitDepthConstraintValue                           ( m_bitDepthConstraint );
  m_cTEncTopSad.setChromaFormatConstraintValue                       ( m_chromaFormatConstraint );
  m_cTEncTopSad.setIntraConstraintFlag                               ( m_intraConstraintFlag );
  m_cTEncTopSad.setOnePictureOnlyConstraintFlag                      ( m_onePictureOnlyConstraintFlag );
  m_cTEncTopSad.setLowerBitRateConstraintFlag                        ( m_lowerBitRateConstraintFlag );

  m_cTEncTopSad.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cTEncTopSad.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cTEncTopSad.setPrintSequenceMSE                                  ( m_printSequenceMSE);
  m_cTEncTopSad.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_cTEncTopSad.setFrameRate                                         ( m_iFrameRate );
  m_cTEncTopSad.setFrameSkip                                         ( m_FrameSkip );
  m_cTEncTopSad.setSourceWidth                                       ( m_iSourceWidth );
  m_cTEncTopSad.setSourceHeight                                      ( m_iSourceHeight );
  m_cTEncTopSad.setConformanceWindow                                 ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTopSad.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  //====== Coding Structure ========
  m_cTEncTopSad.setIntraPeriod                                       ( m_iIntraPeriod );
  m_cTEncTopSad.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cTEncTopSad.setGOPSize                                           ( m_iGOPSize );
  m_cTEncTopSad.setGopList                                           ( m_GOPList );
  m_cTEncTopSad.setExtraRPSs                                         ( m_extraRPSs );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTopSad.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cTEncTopSad.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTopSad.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTopSad.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_cTEncTopSad.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_cTEncTopSad.setQP                                                ( m_iQP );

  m_cTEncTopSad.setPad                                               ( m_aiPad );

  m_cTEncTopSad.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );

  m_cTEncTopSad.setMaxTempLayer                                      ( m_maxTempLayer );
  m_cTEncTopSad.setUseAMP( m_enableAMP );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cTEncTopSad.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cTEncTopSad.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cTEncTopSad.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTopSad.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTopSad.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTopSad.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_cTEncTopSad.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_cTEncTopSad.setSearchRange                                       ( m_iSearchRange );
  m_cTEncTopSad.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_cTEncTopSad.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_cTEncTopSad.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_cTEncTopSad.setMinSearchWindow                                   ( m_minSearchWindow );
  m_cTEncTopSad.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_cTEncTopSad.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cTEncTopSad.setMaxCuDQPDepth                                     ( m_iMaxCuDQPDepth  );
  m_cTEncTopSad.setDiffCuChromaQpOffsetDepth                         ( m_diffCuChromaQpOffsetDepth );
  m_cTEncTopSad.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cTEncTopSad.setChromaCrQpOffset                                  ( m_crQpOffset  );

  m_cTEncTopSad.setChromaFormatIdc                                   ( m_chromaFormatIDC  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTopSad.setUseAdaptQpSelect                                  ( m_bUseAdaptQpSelect   );
#endif

  m_cTEncTopSad.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_cTEncTopSad.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
  m_cTEncTopSad.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_cTEncTopSad.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_cTEncTopSad.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
  m_cTEncTopSad.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
  m_cTEncTopSad.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_cTEncTopSad.setUseASR                                            ( m_bUseASR      );
  m_cTEncTopSad.setUseHADME                                          ( m_bUseHADME    );
  m_cTEncTopSad.setdQPs                                              ( m_aidQP        );
  m_cTEncTopSad.setUseRDOQ                                           ( m_useRDOQ     );
  m_cTEncTopSad.setUseRDOQTS                                         ( m_useRDOQTS   );
#if T0196_SELECTIVE_RDOQ
  m_cTEncTopSad.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
#endif
  m_cTEncTopSad.setRDpenalty                                         ( m_rdPenalty );
  m_cTEncTopSad.setMaxCUWidth                                        ( m_uiMaxCUWidth );
  m_cTEncTopSad.setMaxCUHeight                                       ( m_uiMaxCUHeight );
  m_cTEncTopSad.setMaxTotalCUDepth                                   ( m_uiMaxTotalCUDepth );
  m_cTEncTopSad.setLog2DiffMaxMinCodingBlockSize                     ( m_uiLog2DiffMaxMinCodingBlockSize );
  m_cTEncTopSad.setQuadtreeTULog2MaxSize                             ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTopSad.setQuadtreeTULog2MinSize                             ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTopSad.setQuadtreeTUMaxDepthInter                           ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTopSad.setQuadtreeTUMaxDepthIntra                           ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTopSad.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_cTEncTopSad.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cTEncTopSad.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cTEncTopSad.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cTEncTopSad.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cTEncTopSad.setCrossComponentPredictionEnabledFlag               ( m_crossComponentPredictionEnabledFlag );
  m_cTEncTopSad.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cTEncTopSad.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_LUMA  , m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA]   );
  m_cTEncTopSad.setLog2SaoOffsetScale                                ( CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  m_cTEncTopSad.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cTEncTopSad.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_cTEncTopSad.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_cTEncTopSad.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_cTEncTopSad.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_cTEncTopSad.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_cTEncTopSad.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_cTEncTopSad.setRdpcmEnabledFlag                                ( RDPCMSignallingMode(signallingModeIndex), m_rdpcmEnabledFlag[signallingModeIndex]);
  }
  m_cTEncTopSad.setUseConstrainedIntraPred                           ( m_bUseConstrainedIntraPred );
  m_cTEncTopSad.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_cTEncTopSad.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_cTEncTopSad.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_cTEncTopSad.setPCMLog2MinSize                                    ( m_uiPCMLog2MinSize);
  m_cTEncTopSad.setUsePCM                                            ( m_usePCM );

  // set internal bit-depth and constants
  for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_cTEncTopSad.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_cTEncTopSad.setPCMBitDepth((ChannelType)channelType, m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[channelType] : m_internalBitDepth[channelType]);
  }

  m_cTEncTopSad.setPCMLog2MaxSize                                    ( m_pcmLog2MaxSize);
  m_cTEncTopSad.setMaxNumMergeCand                                   ( m_maxNumMergeCand );


  //====== Weighted Prediction ========
  m_cTEncTopSad.setUseWP                                             ( m_useWeightedPred     );
  m_cTEncTopSad.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_cTEncTopSad.setLog2ParallelMergeLevelMinus2                      ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTopSad.setSliceMode                                         ( m_sliceMode );
  m_cTEncTopSad.setSliceArgument                                     ( m_sliceArgument );

  //====== Dependent Slice ========
  m_cTEncTopSad.setSliceSegmentMode                                  ( m_sliceSegmentMode );
  m_cTEncTopSad.setSliceSegmentArgument                              ( m_sliceSegmentArgument );

  if(m_sliceMode == NO_SLICES )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTopSad.setLFCrossSliceBoundaryFlag                          ( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTopSad.setUseSAO                                            ( m_bUseSAO );
  m_cTEncTopSad.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_cTEncTopSad.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_cTEncTopSad.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_cTEncTopSad.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cTEncTopSad.setSaoCtuBoundary                                    ( m_saoCtuBoundary);
  m_cTEncTopSad.setPCMInputBitDepthFlag                              ( m_bPCMInputBitDepthFlag);
  m_cTEncTopSad.setPCMFilterDisableFlag                              ( m_bPCMFilterDisableFlag);

  m_cTEncTopSad.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );
  m_cTEncTopSad.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
  m_cTEncTopSad.setRecoveryPointSEIEnabled                           ( m_recoveryPointSEIEnabled );
  m_cTEncTopSad.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_cTEncTopSad.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_cTEncTopSad.setToneMappingInfoSEIEnabled                         ( m_toneMappingInfoSEIEnabled );
  m_cTEncTopSad.setTMISEIToneMapId                                   ( m_toneMapId );
  m_cTEncTopSad.setTMISEIToneMapCancelFlag                           ( m_toneMapCancelFlag );
  m_cTEncTopSad.setTMISEIToneMapPersistenceFlag                      ( m_toneMapPersistenceFlag );
  m_cTEncTopSad.setTMISEICodedDataBitDepth                           ( m_toneMapCodedDataBitDepth );
  m_cTEncTopSad.setTMISEITargetBitDepth                              ( m_toneMapTargetBitDepth );
  m_cTEncTopSad.setTMISEIModelID                                     ( m_toneMapModelId );
  m_cTEncTopSad.setTMISEIMinValue                                    ( m_toneMapMinValue );
  m_cTEncTopSad.setTMISEIMaxValue                                    ( m_toneMapMaxValue );
  m_cTEncTopSad.setTMISEISigmoidMidpoint                             ( m_sigmoidMidpoint );
  m_cTEncTopSad.setTMISEISigmoidWidth                                ( m_sigmoidWidth );
  m_cTEncTopSad.setTMISEIStartOfCodedInterva                         ( m_startOfCodedInterval );
  m_cTEncTopSad.setTMISEINumPivots                                   ( m_numPivots );
  m_cTEncTopSad.setTMISEICodedPivotValue                             ( m_codedPivotValue );
  m_cTEncTopSad.setTMISEITargetPivotValue                            ( m_targetPivotValue );
  m_cTEncTopSad.setTMISEICameraIsoSpeedIdc                           ( m_cameraIsoSpeedIdc );
  m_cTEncTopSad.setTMISEICameraIsoSpeedValue                         ( m_cameraIsoSpeedValue );
  m_cTEncTopSad.setTMISEIExposureIndexIdc                            ( m_exposureIndexIdc );
  m_cTEncTopSad.setTMISEIExposureIndexValue                          ( m_exposureIndexValue );
  m_cTEncTopSad.setTMISEIExposureCompensationValueSignFlag           ( m_exposureCompensationValueSignFlag );
  m_cTEncTopSad.setTMISEIExposureCompensationValueNumerator          ( m_exposureCompensationValueNumerator );
  m_cTEncTopSad.setTMISEIExposureCompensationValueDenomIdc           ( m_exposureCompensationValueDenomIdc );
  m_cTEncTopSad.setTMISEIRefScreenLuminanceWhite                     ( m_refScreenLuminanceWhite );
  m_cTEncTopSad.setTMISEIExtendedRangeWhiteLevel                     ( m_extendedRangeWhiteLevel );
  m_cTEncTopSad.setTMISEINominalBlackLevelLumaCodeValue              ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTopSad.setTMISEINominalWhiteLevelLumaCodeValue              ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTopSad.setTMISEIExtendedWhiteLevelLumaCodeValue             ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTopSad.setChromaResamplingFilterHintEnabled                 ( m_chromaResamplingFilterSEIenabled );
  m_cTEncTopSad.setChromaResamplingHorFilterIdc                      ( m_chromaResamplingHorFilterIdc );
  m_cTEncTopSad.setChromaResamplingVerFilterIdc                      ( m_chromaResamplingVerFilterIdc );
  m_cTEncTopSad.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_cTEncTopSad.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_cTEncTopSad.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_cTEncTopSad.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_cTEncTopSad.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_cTEncTopSad.setSegmentedRectFramePackingArrangementSEIEnabled    ( m_segmentedRectFramePackingSEIEnabled );
  m_cTEncTopSad.setSegmentedRectFramePackingArrangementSEICancel     ( m_segmentedRectFramePackingSEICancel );
  m_cTEncTopSad.setSegmentedRectFramePackingArrangementSEIType       ( m_segmentedRectFramePackingSEIType );
  m_cTEncTopSad.setSegmentedRectFramePackingArrangementSEIPersistence( m_segmentedRectFramePackingSEIPersistence );
  m_cTEncTopSad.setDisplayOrientationSEIAngle                        ( m_displayOrientationSEIAngle );
  m_cTEncTopSad.setTemporalLevel0IndexSEIEnabled                     ( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTopSad.setGradualDecodingRefreshInfoEnabled                 ( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTopSad.setNoDisplaySEITLayer                                ( m_noDisplaySEITLayer );
  m_cTEncTopSad.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_cTEncTopSad.setSOPDescriptionSEIEnabled                          ( m_SOPDescriptionSEIEnabled );
  m_cTEncTopSad.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_cTEncTopSad.setTMCTSSEIEnabled                                   ( m_tmctsSEIEnabled );
  m_cTEncTopSad.setTimeCodeSEIEnabled                                ( m_timeCodeSEIEnabled );
  m_cTEncTopSad.setNumberOfTimeSets                                  ( m_timeCodeSEINumTs );
  for(Int i = 0; i < m_timeCodeSEINumTs; i++)
  {
    m_cTEncTopSad.setTimeSet(m_timeSetArray[i], i);
  }
  m_cTEncTopSad.setKneeSEIEnabled                                    ( m_kneeSEIEnabled );
  m_cTEncTopSad.setKneeSEIId                                         ( m_kneeSEIId );
  m_cTEncTopSad.setKneeSEICancelFlag                                 ( m_kneeSEICancelFlag );
  m_cTEncTopSad.setKneeSEIPersistenceFlag                            ( m_kneeSEIPersistenceFlag );
  m_cTEncTopSad.setKneeSEIInputDrange                                ( m_kneeSEIInputDrange );
  m_cTEncTopSad.setKneeSEIInputDispLuminance                         ( m_kneeSEIInputDispLuminance );
  m_cTEncTopSad.setKneeSEIOutputDrange                               ( m_kneeSEIOutputDrange );
  m_cTEncTopSad.setKneeSEIOutputDispLuminance                        ( m_kneeSEIOutputDispLuminance );
  m_cTEncTopSad.setKneeSEINumKneePointsMinus1                        ( m_kneeSEINumKneePointsMinus1 );
  m_cTEncTopSad.setKneeSEIInputKneePoint                             ( m_kneeSEIInputKneePoint );
  m_cTEncTopSad.setKneeSEIOutputKneePoint                            ( m_kneeSEIOutputKneePoint );
  m_cTEncTopSad.setColourRemapInfoSEIFileRoot                        ( m_colourRemapSEIFileRoot );
  m_cTEncTopSad.setMasteringDisplaySEI                               ( m_masteringDisplay );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  m_cTEncTopSad.setSEIAlternativeTransferCharacteristicsSEIEnable    ( m_preferredTransferCharacteristics>=0     );
  m_cTEncTopSad.setSEIPreferredTransferCharacteristics               ( UChar(m_preferredTransferCharacteristics) );
#endif

  m_cTEncTopSad.setTileUniformSpacingFlag                            ( m_tileUniformSpacingFlag );
  m_cTEncTopSad.setNumColumnsMinus1                                  ( m_numTileColumnsMinus1 );
  m_cTEncTopSad.setNumRowsMinus1                                     ( m_numTileRowsMinus1 );
  if(!m_tileUniformSpacingFlag)
  {
    m_cTEncTopSad.setColumnWidth                                     ( m_tileColumnWidth );
    m_cTEncTopSad.setRowHeight                                       ( m_tileRowHeight );
  }
  m_cTEncTopSad.xCheckGSParameters();
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true;
  }
  m_cTEncTopSad.setLFCrossTileBoundaryFlag                           ( m_bLFCrossTileBoundaryFlag );
  m_cTEncTopSad.setEntropyCodingSyncEnabledFlag                      ( m_entropyCodingSyncEnabledFlag );
  m_cTEncTopSad.setTMVPModeId                                        ( m_TMVPModeId );
  m_cTEncTopSad.setUseScalingListId                                  ( m_useScalingListId  );
  m_cTEncTopSad.setScalingListFileName                               ( m_scalingListFileName );
  m_cTEncTopSad.setSignHideFlag                                      ( m_signHideFlag);
  m_cTEncTopSad.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_cTEncTopSad.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_cTEncTopSad.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_cTEncTopSad.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_cTEncTopSad.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_cTEncTopSad.setInitialQP                                         ( m_RCInitialQP );
  m_cTEncTopSad.setForceIntraQP                                      ( m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
  m_cTEncTopSad.setCpbSaturationEnabled                              ( m_RCCpbSaturationEnabled );
  m_cTEncTopSad.setCpbSize                                           ( m_RCCpbSize );
  m_cTEncTopSad.setInitialCpbFullness                                ( m_RCInitialCpbFullness );
#endif
  m_cTEncTopSad.setTransquantBypassEnableFlag                        ( m_TransquantBypassEnableFlag );
  m_cTEncTopSad.setCUTransquantBypassFlagForceValue                  ( m_CUTransquantBypassFlagForce );
  m_cTEncTopSad.setCostMode                                          ( m_costMode );
  m_cTEncTopSad.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_cTEncTopSad.setUseStrongIntraSmoothing                           ( m_useStrongIntraSmoothing );
  m_cTEncTopSad.setActiveParameterSetsSEIEnabled                     ( m_activeParameterSetsSEIEnabled );
  m_cTEncTopSad.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_cTEncTopSad.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_cTEncTopSad.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_cTEncTopSad.setSarWidth                                          ( m_sarWidth );
  m_cTEncTopSad.setSarHeight                                         ( m_sarHeight );
  m_cTEncTopSad.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_cTEncTopSad.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_cTEncTopSad.setVideoSignalTypePresentFlag                        ( m_videoSignalTypePresentFlag );
  m_cTEncTopSad.setVideoFormat                                       ( m_videoFormat );
  m_cTEncTopSad.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_cTEncTopSad.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_cTEncTopSad.setColourPrimaries                                   ( m_colourPrimaries );
  m_cTEncTopSad.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_cTEncTopSad.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_cTEncTopSad.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_cTEncTopSad.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_cTEncTopSad.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_cTEncTopSad.setNeutralChromaIndicationFlag                       ( m_neutralChromaIndicationFlag );
  m_cTEncTopSad.setDefaultDisplayWindow                              ( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTopSad.setFrameFieldInfoPresentFlag                         ( m_frameFieldInfoPresentFlag );
  m_cTEncTopSad.setPocProportionalToTimingFlag                       ( m_pocProportionalToTimingFlag );
  m_cTEncTopSad.setNumTicksPocDiffOneMinus1                          ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTopSad.setBitstreamRestrictionFlag                          ( m_bitstreamRestrictionFlag );
  m_cTEncTopSad.setTilesFixedStructureFlag                           ( m_tilesFixedStructureFlag );
  m_cTEncTopSad.setMotionVectorsOverPicBoundariesFlag                ( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTopSad.setMinSpatialSegmentationIdc                         ( m_minSpatialSegmentationIdc );
  m_cTEncTopSad.setMaxBytesPerPicDenom                               ( m_maxBytesPerPicDenom );
  m_cTEncTopSad.setMaxBitsPerMinCuDenom                              ( m_maxBitsPerMinCuDenom );
  m_cTEncTopSad.setLog2MaxMvLengthHorizontal                         ( m_log2MaxMvLengthHorizontal );
  m_cTEncTopSad.setLog2MaxMvLengthVertical                           ( m_log2MaxMvLengthVertical );
  m_cTEncTopSad.setEfficientFieldIRAPEnabled                         ( m_bEfficientFieldIRAPEnabled );
  m_cTEncTopSad.setHarmonizeGopFirstFieldCoupleEnabled               ( m_bHarmonizeGopFirstFieldCoupleEnabled );

  m_cTEncTopSad.setSummaryOutFilename                                ( m_summaryOutFilename );
  m_cTEncTopSad.setSummaryPicFilenameBase                            ( m_summaryPicFilenameBase );
  m_cTEncTopSad.setSummaryVerboseness                                ( m_summaryVerboseness );
}

Void TAppEncTop::xCreateLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.open( m_inputFileName,     false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1], m_InputChromaFormatIDC);

  if (!m_reconFileName.empty())
  {
    m_cTVideoIOYuvReconFile.open(m_reconFileName, true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth);  // write mode
    m_cTVideoIOYuvReconFileSad.open(std::string(m_reconFileName + "Sad"), true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth);  // write mode
  }

  // Neo Decoder
  m_cTEncTop.create();
  m_cTEncTopSad.create();
}

Void TAppEncTop::xDestroyLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.close();
  m_cTVideoIOYuvReconFile.close();
  m_cTVideoIOYuvReconFileSad.close();

  // Neo Decoder
  m_cTEncTop.destroy();
  m_cTEncTopSad.destroy();
}

Void TAppEncTop::xInitLib(Bool isFieldCoding)
{
  m_cTEncTop.init(isFieldCoding);
  m_cTEncTopSad.init(isFieldCoding);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
Void TAppEncTop::encode()
{
  fstream bitstreamFile(m_bitstreamFileName.c_str(), fstream::binary | fstream::out);
  fstream bitstreamFileSad(std::string(m_bitstreamFileName + "Sad").c_str(), fstream::binary | fstream::out);

  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_bitstreamFileName.c_str());
    exit(EXIT_FAILURE);
  }
  
  if (!bitstreamFileSad)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", std::string(m_bitstreamFileName + "Sad").c_str());
    exit(EXIT_FAILURE);
  }

  TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
  TComPicYuv*       pcPicYuvRec = NULL;

  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib();
  xInitLib(m_isField);

  printChromaFormat();

  // main encoder loop
  Int   iNumEncoded = 0;
  Bool  bEos = false;

  const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
  list<AccessUnit> outputAccessUnitsSad; ///< sad list of access units to write out.  is populated by the encoding process

  TComPicYuv cPicYuvTrueOrg;

  // allocate original YUV buffer
  if( m_isField )
  {
    pcPicYuvOrg->create  ( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true );
    cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true);
  }
  else
  {
    pcPicYuvOrg->create  ( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true );
    cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true );
  }

  while ( !bEos )
  {
    // get buffers
    xGetBuffer(pcPicYuvRec);

    // read input YUV file
    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, &cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );

    // increase number of received frames
    m_iFrameRcvd++;

    bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );

    Bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_cTVideoIOYuvInputFile.isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
      m_cTEncTopSad.setFramesToBeEncoded(m_iFrameRcvd);
    }

    // call encoding function for one frame
    if ( m_isField )
    {
      m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
      m_cTEncTopSad.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnitsSad, iNumEncoded, m_isTopFieldFirst );
    }
    else
    {
      m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
      m_cTEncTopSad.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnitsSad, iNumEncoded );
    }

    // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits, false);
      xWriteOutput(bitstreamFileSad, iNumEncoded, outputAccessUnitsSad, true);
      outputAccessUnits.clear();
      outputAccessUnitsSad.clear();
    }
  }

  m_cTEncTop.printSummary(m_isField);
  m_cTEncTopSad.printSummary(m_isField);

  // delete original YUV buffer
  pcPicYuvOrg->destroy();
  delete pcPicYuvOrg;
  pcPicYuvOrg = NULL;

  // delete used buffers in encoder class
  m_cTEncTop.deletePicBuffer();
  m_cTEncTopSad.deletePicBuffer();
  cPicYuvTrueOrg.destroy();

  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();

  printRateSummary();

  return;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec)
{
  assert( m_iGOPSize > 0 );

  // org. buffer
  if ( m_cListPicYuvRec.size() >= (UInt)m_iGOPSize ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
  {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();

  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;

    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true );

  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}

Void TAppEncTop::xDeleteBuffer( )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvRec.begin();

  Int iSize = Int( m_cListPicYuvRec.size() );

  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }

}

/** 
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param iNumEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */
Void TAppEncTop::xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits, bool isSad)
{
  const InputColourSpaceConversion ipCSC = (!m_outputInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (!m_reconFileName.empty())
      {
        if (!isSad) {
          m_cTVideoIOYuvReconFile.write( pcPicYuvRecTop, pcPicYuvRecBottom, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
        }
        else
        {
          m_cTVideoIOYuvReconFileSad.write( pcPicYuvRecTop, pcPicYuvRecBottom, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
        }
      }

      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop, isSad);

      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom, isSad);
    }
  }
  else
  {
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      if (!m_reconFileName.empty())
      {
        if (!isSad) {
          m_cTVideoIOYuvReconFile.write( pcPicYuvRec, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom,
              NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range  );
        }
        else
        {
          m_cTVideoIOYuvReconFileSad.write( pcPicYuvRec, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom,
              NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range  );
        }
      }

      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats, isSad);
      
    }
  }
}

/**
 *
 */
Void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& annexBsizes, bool isSad)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TSA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
      if (!isSad) {
        m_essentialBytes += *it_stats;
      }
      else
      {
        m_essentialBytesSad += *it_stats;
      }
      break;
    default:
      break;
    }

    if (!isSad) {
      m_totalBytes += *it_stats;
    }
    else
    {
      m_totalBytesSad += *it_stats;
    }
  }
}

Void TAppEncTop::printRateSummary()
{
  Double time = (Double) m_iFrameRcvd / m_iFrameRate;
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
  if (m_summaryVerboseness > 0)
  {
    printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
  }
}

Void TAppEncTop::printChromaFormat()
{
  std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
  switch (m_InputChromaFormatIDC)
  {
  case CHROMA_400:  std::cout << "  4:0:0"; break;
  case CHROMA_420:  std::cout << "  4:2:0"; break;
  case CHROMA_422:  std::cout << "  4:2:2"; break;
  case CHROMA_444:  std::cout << "  4:4:4"; break;
  default:
    std::cerr << "Invalid";
    exit(1);
  }
  std::cout << std::endl;

  std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
  switch (m_cTEncTop.getChromaFormatIdc())
  {
  case CHROMA_400:  std::cout << "  4:0:0"; break;
  case CHROMA_420:  std::cout << "  4:2:0"; break;
  case CHROMA_422:  std::cout << "  4:2:2"; break;
  case CHROMA_444:  std::cout << "  4:4:4"; break;
  default:
    std::cerr << "Invalid";
    exit(1);
  }
  std::cout << "\n" << std::endl;

  std::cout << std::setw(43) << "Sad Output (internal) ChromaFormatIDC = ";
  switch (m_cTEncTopSad.getChromaFormatIdc())
  {
  case CHROMA_400:  std::cout << "  4:0:0"; break;
  case CHROMA_420:  std::cout << "  4:2:0"; break;
  case CHROMA_422:  std::cout << "  4:2:2"; break;
  case CHROMA_444:  std::cout << "  4:4:4"; break;
  default:
    std::cerr << "Invalid";
    exit(1);
  }
  std::cout << "\n" << std::endl;
}

//! \}
