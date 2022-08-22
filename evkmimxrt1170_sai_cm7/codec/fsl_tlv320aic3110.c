/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_tlv320aic3110.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/
#define TLV320AIC3110_CHECK_RET(x, status)  \
    (status) = (x);                  \
    if ((status) != kStatus_Success) \
    {                                \
        return (status);             \
    }

/*! @brief TLV320AIC3110 f2 better performance range */
#define TLV320AIC3110_PLL_F2_MIN_FREQ 90000000U
#define TLV320AIC3110_PLL_F2_MAX_FREQ 100000000U
/*! @brief TLV320AIC3110 PLLN range */
#define TLV320AIC3110_PLL_N_MIN_VALUE 6U
#define TLV320AIC3110_PLL_N_MAX_VALUE 12U
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
 * tlv320aic3110 register cache
 * We can't read the TLV320AIC3110 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const uint16_t tlv320aic3110_reg[TLV320AIC3110_CACHEREGNUM] = {
    0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000a, 0x01c0, 0x0000, 0x00ff, 0x00ff, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x007b, 0x0100, 0x0032, 0x0000, 0x00c3, 0x00c3, 0x01c0, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000, 0x0002, 0x0037, 0x004d, 0x0080, 0x0008, 0x0031, 0x0026, 0x00e9,
};

static uint16_t reg_cache[TLV320AIC3110_CACHEREGNUM];
/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t TLV320AIC3110_SetInternalPllConfig(
    tlv320aic3110_handle_t *handle, uint32_t inputMclk, uint32_t outputClk, uint32_t sampleRate, uint32_t bitWidth)
{
    status_t ret   = kStatus_Success;
    uint32_t pllF2 = outputClk * 4U, pllPrescale = 0U, sysclkDiv = 1U, pllR = 0, pllN = 0, pllK = 0U, fracMode = 0U;

    /* disable PLL power */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, 1U, 0U), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_CLOCK1, 7U, 0U), ret);

    pllN = pllF2 / inputMclk;
    if (pllN < TLV320AIC3110_PLL_N_MIN_VALUE)
    {
        inputMclk >>= 1U;
        pllPrescale = 1;
        pllN        = pllF2 / inputMclk;
        if (pllN < TLV320AIC3110_PLL_N_MIN_VALUE)
        {
            sysclkDiv = 2U;
            pllN      = (pllF2 * sysclkDiv) / inputMclk;
        }
    }

    if ((pllN < TLV320AIC3110_PLL_N_MIN_VALUE) || (pllN > TLV320AIC3110_PLL_N_MAX_VALUE))
    {
        return kStatus_InvalidArgument;
    }

    pllR = (uint32_t)(((uint64_t)pllF2 * sysclkDiv * 1000U) / (inputMclk / 1000U));
    pllK = (uint32_t)(((1UL << 24U) * ((uint64_t)pllR - (uint64_t)pllN * 1000U * 1000U)) / 1000U / 1000U);
    if (pllK != 0U)
    {
        fracMode = 1U;
    }
    TLV320AIC3110_CHECK_RET(
        TLV320AIC3110_WriteReg(handle, TLV320AIC3110_PLL1,
                        ((uint16_t)fracMode << 5U) | ((uint16_t)pllPrescale << 4U) | ((uint16_t)pllN & 0xFU)),
        ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_PLL2, (uint16_t)(pllK >> 16U) & 0xFFU), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_PLL3, (uint16_t)(pllK >> 8U) & 0xFFU), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_PLL4, (uint16_t)pllK & 0xFFU), ret);
    /* enable PLL power */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, 1U, 1U), ret);

    TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_CLOCK1, 7U, ((sysclkDiv == 1U ? 0U : sysclkDiv) << 1U) | 1U), ret);

    return ret;
}

static status_t TLV320AIC3110_SetMasterClock(tlv320aic3110_handle_t *handle, uint32_t sysclk, uint32_t sampleRate, uint32_t bitWidth)
{
    uint32_t bitClockDivider = 0U, regDivider = 0U;
    status_t ret = kStatus_Success;

    bitClockDivider = (sysclk * 2U) / (sampleRate * bitWidth * 2U);

    switch (bitClockDivider)
    {
        case 2:
            regDivider = 0U;
            break;
        case 3:
            regDivider = 1U;
            break;
        case 4:
            regDivider = 2U;
            break;
        case 6:
            regDivider = 3U;
            break;
        case 8:
            regDivider = 4U;
            break;
        case 11:
            regDivider = 5U;
            break;
        case 12:
            regDivider = 6U;
            break;
        case 16:
            regDivider = 7U;
            break;
        case 22:
            regDivider = 8U;
            break;
        case 24:
            regDivider = 9U;
            break;
        case 32:
            regDivider = 10U;
            break;
        case 44:
            regDivider = 11U;
            break;
        case 48:
            regDivider = 12U;
            break;

        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    if (ret == kStatus_Success)
    {
        /* configure the master bit clock divider will be better */
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_CLOCK2, TLV320AIC3110_CLOCK2_BCLK_DIV_MASK, (uint16_t)regDivider),
                         ret);
    }

    return ret;
}

status_t TLV320AIC3110_Init(tlv320aic3110_handle_t *handle, const tlv320aic3110_config_t *config)
{
    status_t ret = kStatus_Success;

    handle->config  = config;
    uint32_t sysclk = config->format.mclk_HZ;

    /* i2c bus initialization */
    if (CODEC_I2C_Init(handle->i2cHandle, config->i2cConfig.codecI2CInstance, TLV320AIC3110_I2C_BAUDRATE,
                       config->i2cConfig.codecI2CSourceClock) != (status_t)kStatus_HAL_I2cSuccess)
    {
        return kStatus_Fail;
    }
    /* load tlv320aic3110 register map */
    (void)memcpy(reg_cache, tlv320aic3110_reg, sizeof(tlv320aic3110_reg));

    /* Reset the codec */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RESET, 0x00), ret);
    /*
     * VMID=50K, Enable VREF, AINL, AINR, ADCL and ADCR
     * I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5), ADC (bit 6) are powered on
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, 0xFE), ret);
    /*
     * Enable DACL, DACR, LOUT1, ROUT1, PLL down, SPKL, SPKR
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER2, 0x1F8), ret);
    /*
     * Enable left and right channel input PGA, left and right output mixer
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER3, 0x3C), ret);
    /* ADC and DAC uses same clock */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_IFACE2, 0x40), ret);
    /* set data route */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetDataRoute(handle, config->route), ret);
    /* set data protocol */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetProtocol(handle, config->bus), ret);

    if ((config->masterClock.sysclkSource == kTLV320AIC3110_SysClkSourceInternalPLL))
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetInternalPllConfig(handle, sysclk, config->masterClock.sysclkFreq,
                                                     config->format.sampleRate, config->format.bitWidth),
                         ret);
        sysclk = config->masterClock.sysclkFreq;
    }
    /* set master or slave */
    if (config->master_slave)
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetMasterClock(handle, sysclk, config->format.sampleRate, config->format.bitWidth),
                         ret);
    }
    TLV320AIC3110_SetMasterSlave(handle, config->master_slave);
    /* select left input */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetLeftInput(handle, config->leftInputSource), ret);
    /* select right input */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetRightInput(handle, config->rightInputSource), ret);
    /* speaker power */
    if (config->enableSpeaker)
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleSpeaker, true), ret);
    }

    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ADDCTL1, 0x0C0), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ADDCTL4, 0x40), ret);

    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_BYPASS1, 0x0), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_BYPASS2, 0x0), ret);
    /*
     * ADC volume, 0dB
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LADC, 0x1C3), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RADC, 0x1C3), ret);

    /*
     * Digital DAC volume, -15.5dB
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LDAC, 0x1E0), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RDAC, 0x1E0), ret);

    /*
     * Headphone volume, LOUT1 and ROUT1, -10dB
     */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT1, 0x16F), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT1, 0x16F), ret);

    /* speaker volume 6dB */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT2, 0x1ff), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT2, 0x1ff), ret);
    /* enable class D output */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_CLASSD1, 0xf7), ret);

    /* Unmute DAC. */
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_DACCTL1, 0x0000), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, 0x117), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, 0x117), ret);

    TLV320AIC3110_CHECK_RET(TLV320AIC3110_ConfigDataFormat(handle, sysclk, config->format.sampleRate, config->format.bitWidth), ret);

    return ret;
}

status_t TLV320AIC3110_Deinit(tlv320aic3110_handle_t *handle)
{
    status_t ret = kStatus_Success;

    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleADC, false), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleDAC, false), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleVREF, false), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineIn, false), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineOut, false), ret);
    TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleSpeaker, false), ret);
    TLV320AIC3110_CHECK_RET(CODEC_I2C_Deinit(handle->i2cHandle), ret);

    return ret;
}

void TLV320AIC3110_SetMasterSlave(tlv320aic3110_handle_t *handle, bool master)
{
    if (master)
    {
        (void)TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_MS_MASK, TLV320AIC3110_IFACE1_MS(TLV320AIC3110_IFACE1_MASTER));
    }
    else
    {
        (void)TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_MS_MASK, TLV320AIC3110_IFACE1_MS(TLV320AIC3110_IFACE1_SLAVE));
    }
}

status_t TLV320AIC3110_SetModule(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_ADCL_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_ADCL_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_ADCR_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_ADCR_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleDAC:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_DACL_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_DACL_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_DACR_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_DACR_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleVREF:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_VREF_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_VREF_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleLineIn:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_AINL_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_AINL_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_AINR_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_AINR_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER3, TLV320AIC3110_POWER3_LMIC_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER3_LMIC_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER3, TLV320AIC3110_POWER3_RMIC_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER3_RMIC_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleLineOut:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_LOUT1_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_LOUT1_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_ROUT1_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_ROUT1_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleMICB:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER1, TLV320AIC3110_POWER1_MICB_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER1_MICB_SHIFT)),
                             ret);
            break;
        case kTLV320AIC3110_ModuleSpeaker:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_SPKL_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_SPKL_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER2, TLV320AIC3110_POWER2_SPKR_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER2_SPKR_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_CLASSD1, 0xF7), ret);
            break;
        case kTLV320AIC3110_ModuleOMIX:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER3, TLV320AIC3110_POWER3_LOMIX_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER3_LOMIX_SHIFT)),
                             ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_POWER3, TLV320AIC3110_POWER3_ROMIX_MASK,
                                              ((uint16_t)isEnabled << TLV320AIC3110_POWER3_ROMIX_SHIFT)),
                             ret);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_SetDataRoute(tlv320aic3110_handle_t *handle, tlv320aic3110_route_t route)
{
    status_t ret = kStatus_Success;
    switch (route)
    {
        case kTLV320AIC3110_RouteBypass:
            /* Bypass means from line-in to HP*/
            /*
             * Left LINPUT3 to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUTMIX, 0x80), ret);

            /*
             * Right RINPUT3 to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUTMIX, 0x80), ret);
            break;
        case kTLV320AIC3110_RoutePlayback:
            /* Data route I2S_IN-> DAC-> HP */
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUTMIX, 0x100), ret);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUTMIX, 0x100), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER3, 0x0C), ret);
            /* Set power for DAC */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleDAC, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleOMIX, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineOut, true), ret);
            break;
        case kTLV320AIC3110_RoutePlaybackandRecord:
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUTMIX, 0x100), ret);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUTMIX, 0x100), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER3, 0x3C), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleDAC, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleADC, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineIn, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleOMIX, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineOut, true), ret);
            break;
        case kTLV320AIC3110_RouteRecord:
            /* LINE_IN->ADC->I2S_OUT */
            /*
             * Left and right input boost, LIN3BOOST and RIN3BOOST = 0dB
             */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER3, 0x30), ret);
            /* Power up ADC and AIN */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleLineIn, true), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_SetModule(handle, kTLV320AIC3110_ModuleADC, true), ret);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_SetLeftInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kTLV320AIC3110_InputClosed:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val &= (uint16_t) ~(TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            break;
        case kTLV320AIC3110_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINPATH, 0x138), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputDifferentialMicInput2:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINPATH, 0x178), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputDifferentialMicInput3:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINPATH, 0x1B8), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputLineINPUT2:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_INBMIX1, &val), ret);
            val |= 0xEU;
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_INBMIX1, val), ret);
            break;
        case kTLV320AIC3110_InputLineINPUT3:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINL_MASK | TLV320AIC3110_POWER1_ADCL_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_INBMIX1, &val), ret);
            val |= 0x70U;
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_INBMIX1, val), ret);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }

    return ret;
}

status_t TLV320AIC3110_SetRightInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kTLV320AIC3110_InputClosed:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val &= (uint16_t) ~(TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            break;
        case kTLV320AIC3110_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINPATH, 0x138), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputDifferentialMicInput2:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINPATH, 0x178), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputDifferentialMicInput3:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK | TLV320AIC3110_POWER1_MICB_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINPATH, 0x1B8), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, 0x117), ret);
            break;
        case kTLV320AIC3110_InputLineINPUT2:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_INBMIX2, &val), ret);
            val |= 0xEU;
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_INBMIX2, val), ret);
            break;
        case kTLV320AIC3110_InputLineINPUT3:
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_POWER1, &val), ret);
            val |= (TLV320AIC3110_POWER1_AINR_MASK | TLV320AIC3110_POWER1_ADCR_MASK);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_POWER1, val), ret);
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_ReadReg(TLV320AIC3110_INBMIX2, &val), ret);
            val |= 0x70U;
            TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_INBMIX2, val), ret);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }

    return ret;
}

status_t TLV320AIC3110_SetProtocol(tlv320aic3110_handle_t *handle, tlv320aic3110_protocol_t protocol)
{
    return TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_FORMAT_MASK | TLV320AIC3110_IFACE1_LRP_MASK,
                            (uint16_t)protocol);
}

status_t TLV320AIC3110_SetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, uint32_t volume)
{
    uint16_t vol = 0;
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            if (volume > 255U)
            {
                ret = kStatus_InvalidArgument;
            }
            else
            {
                vol = (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LADC, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RADC, vol), ret);
                /* Update volume */
                vol = (uint16_t)(0x100U | volume);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LADC, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RADC, vol), ret);
            }
            break;
        case kTLV320AIC3110_ModuleDAC:
            if (volume > 255U)
            {
                ret = kStatus_InvalidArgument;
            }
            else
            {
                vol = (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LDAC, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RDAC, vol), ret);
                vol = 0x100U | (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LDAC, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RDAC, vol), ret);
            }
            break;
        case kTLV320AIC3110_ModuleHP:
            if (volume > 0x7FU)
            {
                ret = kStatus_InvalidArgument;
            }
            else
            {
                vol = (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT1, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT1, vol), ret);
                vol = 0x100U | (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT1, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT1, vol), ret);
            }
            break;
        case kTLV320AIC3110_ModuleLineIn:
            if (volume > 0x3FU)
            {
                ret = kStatus_InvalidArgument;
            }
            else
            {
                vol = (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, vol), ret);
                vol = 0x100U | (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LINVOL, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RINVOL, vol), ret);
            }
            break;
        case kTLV320AIC3110_ModuleSpeaker:
            if (volume > 0x7FU)
            {
                ret = kStatus_InvalidArgument;
            }
            else
            {
                vol = (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT2, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT2, vol), ret);
                vol = 0x100U | (uint16_t)volume;
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT2, vol), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT2, vol), ret);
            }
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

uint32_t TLV320AIC3110_GetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module)
{
    uint16_t vol = 0;

    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            (void)TLV320AIC3110_ReadReg(TLV320AIC3110_LADC, &vol);
            vol &= 0xFFU;
            break;
        case kTLV320AIC3110_ModuleDAC:
            (void)TLV320AIC3110_ReadReg(TLV320AIC3110_LDAC, &vol);
            vol &= 0xFFU;
            break;
        case kTLV320AIC3110_ModuleHP:
            (void)TLV320AIC3110_ReadReg(TLV320AIC3110_LOUT1, &vol);
            vol &= 0x7FU;
            break;
        case kTLV320AIC3110_ModuleLineOut:
            (void)TLV320AIC3110_ReadReg(TLV320AIC3110_LINVOL, &vol);
            vol &= 0x3FU;
            break;
        default:
            vol = 0;
            break;
    }
    return vol;
}

status_t TLV320AIC3110_SetMute(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            /*
             * Digital Mute
             */
            if (isEnabled)
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LADC, 0x100), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RADC, 0x100), ret);
            }
            else
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LADC, 0x1C3), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RADC, 0x1C3), ret);
            }
            break;
        case kTLV320AIC3110_ModuleDAC:
            /*
             * Digital mute
             */
            if (isEnabled)
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LDAC, 0x100), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RDAC, 0x100), ret);
            }
            else
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LDAC, 0x1FF), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_RDAC, 0x1FF), ret);
            }
            break;
        case kTLV320AIC3110_ModuleHP:
            /*
             * Analog mute
             */
            if (isEnabled)
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT1, 0x100), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT1, 0x100), ret);
            }
            else
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT1, 0x16F), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT1, 0x16F), ret);
            }
            break;

        case kTLV320AIC3110_ModuleSpeaker:
            if (isEnabled)
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT2, 0x100), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT2, 0x100), ret);
            }
            else
            {
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_LOUT2, 0x16F), ret);
                TLV320AIC3110_CHECK_RET(TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ROUT2, 0x16f), ret);
            }
            break;

        case kTLV320AIC3110_ModuleLineOut:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_ConfigDataFormat(tlv320aic3110_handle_t *handle, uint32_t sysclk, uint32_t sample_rate, uint32_t bits)
{
    status_t retval  = kStatus_Success;
    uint32_t divider = 0;
    uint16_t val     = 0;

    /* Compute sample rate divider, dac and adc are the same sample rate */
    divider = sysclk / sample_rate;
    if (divider == 256U)
    {
        val = 0;
    }
    else if (divider > 256U)
    {
        val = (uint16_t)(((divider / 256U) << 6U) | ((divider / 256U) << 3U));
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    retval = TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_CLOCK1, 0x1F8U, val);
    if (retval != kStatus_Success)
    {
        return retval;
    }

    /*
     * Slave mode (MS = 0), LRP = 0, 32bit WL, left justified (FORMAT[1:0]=0b01)
     */
    switch (bits)
    {
        case 16:
            retval = TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_WL_MASK,
                                      TLV320AIC3110_IFACE1_WL(TLV320AIC3110_IFACE1_WL_16BITS));
            break;
        case 20:
            retval = TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_WL_MASK,
                                      TLV320AIC3110_IFACE1_WL(TLV320AIC3110_IFACE1_WL_20BITS));
            break;
        case 24:
            retval = TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_WL_MASK,
                                      TLV320AIC3110_IFACE1_WL(TLV320AIC3110_IFACE1_WL_24BITS));
            break;
        case 32:
            retval = TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_IFACE1, TLV320AIC3110_IFACE1_WL_MASK,
                                      TLV320AIC3110_IFACE1_WL(TLV320AIC3110_IFACE1_WL_32BITS));
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }

    return retval;
}

status_t TLV320AIC3110_SetJackDetect(tlv320aic3110_handle_t *handle, bool isEnabled)
{
    status_t retval = 0;
    uint16_t val    = 0;

    if (TLV320AIC3110_ReadReg(TLV320AIC3110_ADDCTL2, &val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    if (isEnabled)
    {
        val |= 0x40U;
    }
    else
    {
        val &= 0xCFU;
    }

    retval = TLV320AIC3110_WriteReg(handle, TLV320AIC3110_ADDCTL2, val);

    return retval;
}

status_t TLV320AIC3110_WriteReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t val)
{
    uint8_t cmd;
    uint8_t buff = (uint8_t)val & 0xFFU;

    /* The register address */
    cmd = (reg << 1U) | (uint8_t)((val >> 8U) & 0x0001U);

    reg_cache[reg] = val;

    return CODEC_I2C_Send(handle->i2cHandle, handle->config->slaveAddress, cmd, 1U, &buff, 1U);
}

status_t TLV320AIC3110_ReadReg(uint8_t reg, uint16_t *val)
{
    if (reg >= TLV320AIC3110_CACHEREGNUM)
    {
        return kStatus_InvalidArgument;
    }

    *val = reg_cache[reg];

    return kStatus_Success;
}

status_t TLV320AIC3110_ModifyReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t mask, uint16_t val)
{
    status_t retval  = 0;
    uint16_t reg_val = 0;
    retval           = TLV320AIC3110_ReadReg(reg, &reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    reg_val &= (uint16_t)~mask;
    reg_val |= val;
    retval = TLV320AIC3110_WriteReg(handle, reg, reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t TLV320AIC3110_SetPlay(tlv320aic3110_handle_t *handle, uint32_t playSource)
{
    status_t ret = kStatus_Success;

    if (((uint32_t)kTLV320AIC3110_PlaySourcePGA & playSource) != 0U)
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS1, 0x80U, 0x80U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS2, 0x80U, 0x80U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_LOUTMIX, 0x180U, 0U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_ROUTMIX, 0x180U, 0U), ret);
    }

    if ((playSource & (uint32_t)kTLV320AIC3110_PlaySourceDAC) != 0U)
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS1, 0x80U, 0x00U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS2, 0x80U, 0x00U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_LOUTMIX, 0x180U, 0x100U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_ROUTMIX, 0x180U, 0x100U), ret);
    }

    if ((playSource & (uint32_t)kTLV320AIC3110_PlaySourceInput) != 0U)
    {
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS1, 0x80U, 0x0U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_BYPASS2, 0x80U, 0x0U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_LOUTMIX, 0x180U, 0x80U), ret);
        TLV320AIC3110_CHECK_RET(TLV320AIC3110_ModifyReg(handle, TLV320AIC3110_ROUTMIX, 0x180U, 0x80U), ret);
    }

    return ret;
}
