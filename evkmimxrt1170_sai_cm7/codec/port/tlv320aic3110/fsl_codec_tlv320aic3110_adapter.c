/*
 * Copyright  2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_codec_tlv320aic3110_adapter.h"
#include "fsl_codec_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief module capability definition */
#define HAL_TLV320AIC3110_MODULE_CAPABILITY                                                                                   \
    (kCODEC_SupportModuleADC | kCODEC_SupportModuleDAC | kCODEC_SupportModuleHeadphone | kCODEC_SupportModuleLineout | \
     kCODEC_SupportModuleSpeaker | kCODEC_SupportModuleMic | kCODEC_SupportModuleMixer | kCODEC_SupportModuleMicbias | \
     kCODEC_SupportModuleVref | kCODEC_SupportModuleLinein)

#define HAL_TLV320AIC3110_PLAY_CAPABILITY                                                                       \
    (kCODEC_SupportPlayChannelLeft0 | kCODEC_SupportPlayChannelRight0 | kCODEC_SupportPlayChannelLeft1 | \
     kCODEC_SupportPlayChannelRight1 | kCODEC_SupportPlaySourcePGA | kCODEC_SupportPlaySourceDAC |       \
     kCODEC_SupportPlaySourceInput)

#define HAL_TLV320AIC3110_VOLUME_CAPABILITY                                                                     \
    (kCODEC_SupportPlayChannelLeft0 | kCODEC_SupportPlayChannelRight0 | kCODEC_SupportPlayChannelLeft1 | \
     kCODEC_SupportPlayChannelRight1 | kCODEC_VolumeDAC)

#define HAL_TLV320AIC3110_RECORD_CAPABILITY                                                                    \
    (kCODEC_SupportPlayChannelLeft0 | kCODEC_SupportPlayChannelLeft1 | kCODEC_SupportPlayChannelLeft2 | \
     kCODEC_SupportPlayChannelRight0 | kCODEC_SupportPlayChannelRight1 | kCODEC_SupportPlayChannelRight2)

/*! @brief tlv320aic3110 map protocol */
#define HAL_TLV320AIC3110_MAP_PROTOCOL(protocol)                 \
    ((protocol) == kCODEC_BusI2S ?                        \
         kTLV320AIC3110_BusI2S :                                 \
         (protocol) == kCODEC_BusLeftJustified ?          \
         kTLV320AIC3110_BusLeftJustified :                       \
         (protocol) == kCODEC_BusRightJustified ?         \
         kTLV320AIC3110_BusRightJustified :                      \
         (protocol) == kCODEC_BusPCMA ? kTLV320AIC3110_BusPCMA : \
                                        (protocol) == kCODEC_BusPCMB ? kTLV320AIC3110_BusPCMB : kTLV320AIC3110_BusI2S)

/*! @brief tlv320aic3110 map module */
#define HAL_TLV320AIC3110_MAP_MODULE(module)                   \
    ((module) == (uint32_t)kCODEC_ModuleADC ?           \
         kTLV320AIC3110_ModuleADC :                            \
         (module) == (uint32_t)kCODEC_ModuleDAC ?       \
         kTLV320AIC3110_ModuleDAC :                            \
         (module) == (uint32_t)kCODEC_ModuleVref ?      \
         kTLV320AIC3110_ModuleVREF :                           \
         (module) == (uint32_t)kCODEC_ModuleHeadphone ? \
         kTLV320AIC3110_ModuleHP :                             \
         (module) == (uint32_t)kCODEC_ModuleMicbias ?   \
         kTLV320AIC3110_ModuleMICB :                           \
         (module) == (uint32_t)kCODEC_ModuleMic ?       \
         kTLV320AIC3110_ModuleMIC :                            \
         (module) == (uint32_t)kCODEC_ModuleLinein ?    \
         kTLV320AIC3110_ModuleLineIn :                         \
         (module) == (uint32_t)kCODEC_ModuleSpeaker ?   \
         kTLV320AIC3110_ModuleSpeaker :                        \
         (module) == (uint32_t)kCODEC_ModuleMixer ?     \
         kTLV320AIC3110_ModuleOMIX :                           \
         (module) == (uint32_t)kCODEC_ModuleLineout ? kTLV320AIC3110_ModuleLineOut : kTLV320AIC3110_ModuleADC)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const codec_capability_t s_tlv320aic3110_capability = {
    .codecPlayCapability   = HAL_TLV320AIC3110_PLAY_CAPABILITY,
    .codecVolumeCapability = HAL_TLV320AIC3110_VOLUME_CAPABILITY,
    .codecModuleCapability = HAL_TLV320AIC3110_MODULE_CAPABILITY,
    .codecRecordCapability = HAL_TLV320AIC3110_RECORD_CAPABILITY,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * brief Codec initilization.
 *
 * param handle codec handle.
 * param config codec configuration.
 * return kStatus_Success is success, else initial failed.
 */
status_t HAL_CODEC_TLV320AIC3110_Init(void *handle, void *config)
{
    assert((config != NULL) && (handle != NULL));

    codec_config_t *codecConfig = (codec_config_t *)config;

    tlv320aic3110_config_t *devConfig = (tlv320aic3110_config_t *)(codecConfig->codecDevConfig);
    tlv320aic3110_handle_t *devHandle = (tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle));

    ((codec_handle_t *)handle)->codecCapability = &s_tlv320aic3110_capability;

    /* codec device initialization */
    return TLV320AIC3110_Init(devHandle, devConfig);
}

/*!
 * brief Codec de-initilization.
 *
 * param handle codec handle.
 * return kStatus_Success is success, else de-initial failed.
 */
status_t HAL_CODEC_TLV320AIC3110_Deinit(void *handle)
{
    assert(handle != NULL);

    return TLV320AIC3110_Deinit((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)));
}

/*!
 * brief set audio data format.
 *
 * param handle codec handle.
 * param mclk master clock frequency in HZ.
 * param sampleRate sample rate in HZ.
 * param bitWidth bit width.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetFormat(void *handle, uint32_t mclk, uint32_t sampleRate, uint32_t bitWidth)
{
    assert(handle != NULL);

    return TLV320AIC3110_ConfigDataFormat((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)), mclk,
                                   sampleRate, bitWidth);
}

/*!
 * brief set audio codec module volume.
 *
 * param handle codec handle.
 * param channel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * param volume volume value, support 0 ~ 100, 0 is mute, 100 is the maximum volume value.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetVolume(void *handle, uint32_t playChannel, uint32_t volume)
{
    assert(handle != NULL);

    status_t retVal       = kStatus_Success;
    uint32_t mappedVolume = 0U;

    if (((playChannel & (uint32_t)kTLV320AIC3110_HeadphoneLeft) != 0U) ||
        ((playChannel & (uint32_t)kTLV320AIC3110_HeadphoneRight) != 0U))
    {
        /*
         * 0 is mute
         * 1 - 100 is mapped to 0x30 - 0x7F
         */
        mappedVolume = (volume * (TLV320AIC3110_HEADPHONE_MAX_VOLUME_vALUE - TLV320AIC3110_HEADPHONE_MIN_VOLUME_vALUE)) / 100U +
                       TLV320AIC3110_HEADPHONE_MIN_VOLUME_vALUE;
        retVal = TLV320AIC3110_SetVolume((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                                  kTLV320AIC3110_ModuleHP, mappedVolume);
    }

    if (((playChannel & (uint32_t)kTLV320AIC3110_SpeakerLeft) != 0U) || ((playChannel & (uint32_t)kTLV320AIC3110_SpeakerRight) != 0U))
    {
        /*
         * 0 is mute
         * 1 - 100 is mapped to 0x30 - 0x7F
         */
        mappedVolume = (volume * (TLV320AIC3110_HEADPHONE_MAX_VOLUME_vALUE - TLV320AIC3110_HEADPHONE_MIN_VOLUME_vALUE)) / 100U +
                       TLV320AIC3110_HEADPHONE_MIN_VOLUME_vALUE;
        retVal = TLV320AIC3110_SetVolume((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                                  kTLV320AIC3110_ModuleSpeaker, mappedVolume);
    }

    if ((playChannel & (uint32_t)kCODEC_VolumeDAC) != 0U)
    {
        /*
         * 0 is mute
         * 0 - 100 is mapped to 0x00 - 0xFF
         */
        mappedVolume = (volume * (TLV320AIC3110_DAC_MAX_VOLUME_vALUE - 0U)) / 100U;

        retVal = TLV320AIC3110_SetVolume((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                                  kTLV320AIC3110_ModuleDAC, mappedVolume);
    }

    return retVal;
}

/*!
 * brief set audio codec module mute.
 *
 * param handle codec handle.
 * param channel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * param isMute true is mute, false is unmute.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetMute(void *handle, uint32_t playChannel, bool isMute)
{
    assert(handle != NULL);

    status_t retVal = kStatus_Success;

    if (((playChannel & (uint32_t)kTLV320AIC3110_HeadphoneLeft) != 0U) ||
        ((playChannel & (uint32_t)kTLV320AIC3110_HeadphoneRight) != 0U))
    {
        retVal = TLV320AIC3110_SetMute((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                                kTLV320AIC3110_ModuleHP, isMute);
    }

    if (((playChannel & (uint32_t)kTLV320AIC3110_SpeakerLeft) != 0U) || ((playChannel & (uint32_t)kTLV320AIC3110_SpeakerRight) != 0U))
    {
        retVal = TLV320AIC3110_SetMute((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                                kTLV320AIC3110_ModuleSpeaker, isMute);
    }

    return retVal;
}

/*!
 * brief set audio codec module power.
 *
 * param handle codec handle.
 * param module audio codec module.
 * param powerOn true is power on, false is power down.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetPower(void *handle, uint32_t module, bool powerOn)
{
    assert(handle != NULL);

    return TLV320AIC3110_SetModule((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)),
                            HAL_TLV320AIC3110_MAP_MODULE(module), powerOn);
}

/*!
 * brief codec set record channel.
 *
 * param handle codec handle.
 * param leftRecordChannel audio codec record channel, reference _codec_record_channel, can be a value or combine value
 of member in _codec_record_channel.
 * param rightRecordChannel audio codec record channel, reference _codec_record_channel, can be a value combine of
 member in _codec_record_channel.

 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetRecordChannel(void *handle, uint32_t leftRecordChannel, uint32_t rightRecordChannel)
{
    return kStatus_CODEC_NotSupport;
}

/*!
 * brief codec set record source.
 *
 * param handle codec handle.
 * param source audio codec record source, can be a value or combine value of _codec_record_source.
 *
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetRecord(void *handle, uint32_t recordSource)
{
    return kStatus_CODEC_NotSupport;
}

/*!
 * brief codec set play source.
 *
 * param handle codec handle.
 * param playSource audio codec play source, can be a value or combine value of _codec_play_source.
 *
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetPlay(void *handle, uint32_t playSource)
{
    assert(handle != NULL);

    return TLV320AIC3110_SetPlay((tlv320aic3110_handle_t *)((uintptr_t)(((codec_handle_t *)handle)->codecDevHandle)), playSource);
}

/*!
 * brief codec module control.
 *
 * param handle codec handle.
 * param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_ModuleControl(void *handle, uint32_t cmd, uint32_t data)
{
    return kStatus_CODEC_NotSupport;
}
