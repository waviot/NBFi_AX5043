
#include "ax5043.h"
#include "easyax5043.h"
#include <libmftypes.h>
#include <libmfcrc.h>
#include <nbfi.h>
#include <rf.h>

#define NAIVE_MEMCPY(x,y,n) for(int i=0;i<(n);i++){x[i]=y[i];}


extern nbfi_phy_channel_t nbfi_phy_channel;

extern NBFi_ax5043_pins_s nbfi_ax5043_pins; 

void ax5043_set_registers(void)
{
    ax5043_spi_write(AX5043_PINFUNCSYSCLK, nbfi_ax5043_pins.sysclk);
    ax5043_spi_write(AX5043_PINFUNCDCLK, nbfi_ax5043_pins.dclk);
    ax5043_spi_write(AX5043_PINFUNCDATA, nbfi_ax5043_pins.data);
    ax5043_spi_write(AX5043_PINFUNCANTSEL, nbfi_ax5043_pins.antsel);
    ax5043_spi_write(AX5043_TXPWRCOEFFB1, (nbfi_ax5043_pins.txpwr>>8));
    ax5043_spi_write(AX5043_TXPWRCOEFFB0, (nbfi_ax5043_pins.txpwr)&0xFF);
    ax5043_spi_write(AX5043_MODCFGA, nbfi_ax5043_pins.cfga);
    
    switch(nbfi_phy_channel)
    {
    case UL_DBPSK_50_PROT_D:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x03);
        ax5043_spi_write(AX5043_FRAMING                , 0x06);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x01);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xFC);
        ax5043_spi_write(AX5043_DECIMATION             , 0x7F);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x7F);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0xF3);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xE9);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xF9);
        ax5043_spi_write(AX5043_DRGAIN0                , 0x84);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xE9);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xF7);
        ax5043_spi_write(AX5043_DRGAIN1                , 0x83);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xF6);
        ax5043_spi_write(AX5043_DRGAIN3                , 0x82);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x00);
        ax5043_spi_write(AX5043_TXRATE0                , 0x81);
        ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x80);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x00);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x88);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xC8);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xFF);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0x00);
        ax5043_spi_write(AX5043_MATCH0LEN              , 0x07);
        ax5043_spi_write(AX5043_MATCH0MAX              , 0x07);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x0A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x12);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xCB);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF34                  , 0x08);
        ax5043_spi_write(AX5043_0xF35                  , 0x12);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        break;
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
 //   case UL_DBPSK_400_PROT_E:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x03);
        ax5043_spi_write(AX5043_FRAMING                , 0x06);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x00);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xFC);
        ax5043_spi_write(AX5043_DECIMATION             , 0x66);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x27);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0xD4);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xD7);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0x98);
        ax5043_spi_write(AX5043_DRGAIN0                , 0xA2);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xD7);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0x96);
        ax5043_spi_write(AX5043_DRGAIN1                , 0xA1);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0x95);
        ax5043_spi_write(AX5043_DRGAIN3                , 0xA0);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x04);
        ax5043_spi_write(AX5043_TXRATE0                , 0x08);
        ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x80);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x00);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x88);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xFE);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xFF);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0x00);
        ax5043_spi_write(AX5043_MATCH0LEN              , 0x07);
        ax5043_spi_write(AX5043_MATCH0MAX              , 0x07);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x0A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x12);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xDD);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF34                  , 0x08);
        ax5043_spi_write(AX5043_0xF35                  , 0x11);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        break;
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x03);
        ax5043_spi_write(AX5043_FRAMING                , 0x06);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x03);
        ax5043_spi_write(AX5043_IFFREQ0                , 0x5C);
        ax5043_spi_write(AX5043_DECIMATION             , 0x0C);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x2A);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0x51);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xA4);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xA8);
        ax5043_spi_write(AX5043_DRGAIN0                , 0xA2);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xA4);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xA6);
        ax5043_spi_write(AX5043_DRGAIN1                , 0xA1);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xA5);
        ax5043_spi_write(AX5043_DRGAIN3                , 0xA0);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x20);
        ax5043_spi_write(AX5043_TXRATE0                , 0x44);
        ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x80);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x00);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x88);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xC8);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xFF);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0x00);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0x00);
        ax5043_spi_write(AX5043_MATCH0LEN              , 0x07);
        ax5043_spi_write(AX5043_MATCH0MAX              , 0x07);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0xFF);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x0A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x12);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xE3);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF34                  , 0x08);
        ax5043_spi_write(AX5043_0xF35                  , 0x11);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);

        break;
    case DL_PSK_200:
    case UL_PSK_200:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x00);
        ax5043_spi_write(AX5043_FRAMING                , 0x64);
        ax5043_spi_write(AX5043_FEC                    , 0x33);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);  
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x01);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xF8);
        ax5043_spi_write(AX5043_DECIMATION             , 0x7F);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x7F);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0xF3);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xE9);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xF9);
        ax5043_spi_write(AX5043_DRGAIN0                , 0x84);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xE9);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xF7);
        ax5043_spi_write(AX5043_DRGAIN1                , 0x83);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xF6);
        ax5043_spi_write(AX5043_DRGAIN3                , 0x82);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0x03);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x00);
        ax5043_spi_write(AX5043_TXRATE0                , 0x81);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x81);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x80);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x00);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xF0);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0xCC);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0xCC);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x8A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x17);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xDD);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF35                  , 0x12);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        if(PSK_BAND)
        {
            #ifdef GLONASS
            ax5043_spi_write(AX5043_PLLVCOI                , 0x98);
            #else
            ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
            #endif // GLONASS
            ax5043_spi_write(AX5043_0xF34                  , 0x08);

        }
        else
        {
            ax5043_spi_write(AX5043_PLLVCOI                , 0x97);
            ax5043_spi_write(AX5043_0xF34                  , 0x28);
        }
        break;
    case DL_PSK_500:
    case UL_PSK_500:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x00);
        ax5043_spi_write(AX5043_FRAMING                , 0x64);
        ax5043_spi_write(AX5043_FEC                    , 0x33);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x01);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xF8);
        ax5043_spi_write(AX5043_DECIMATION             , 0x7F);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x33);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0x2E);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xE8);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xC8);
        ax5043_spi_write(AX5043_DRGAIN0                , 0xC2);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xE8);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xC6);
        ax5043_spi_write(AX5043_DRGAIN1                , 0xC1);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xC5);
        ax5043_spi_write(AX5043_DRGAIN3                , 0xC0);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x01);
        ax5043_spi_write(AX5043_TXRATE0                , 0x43);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x81);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x80);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x00);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xF0);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0xCC);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0xCC);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x8A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x17);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xDD);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF35                  , 0x12);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        if(PSK_BAND)
        {
            #ifdef GLONASS
            ax5043_spi_write(AX5043_PLLVCOI                , 0x98);
            #else
            ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
            #endif // GLONASS
            ax5043_spi_write(AX5043_0xF34                  , 0x08);

        }
        else
        {
            ax5043_spi_write(AX5043_PLLVCOI                , 0x97);
            ax5043_spi_write(AX5043_0xF34                  , 0x28);
        }
        break;
    case DL_PSK_5000:
    case UL_PSK_5000:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x00);
        ax5043_spi_write(AX5043_FRAMING                , 0x64);
        ax5043_spi_write(AX5043_FEC                    , 0x33);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x00);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xFC);
        ax5043_spi_write(AX5043_DECIMATION             , 0x20);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x28);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0xA0);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0xC5);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xA8);
        ax5043_spi_write(AX5043_DRGAIN0                , 0xA2);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0xC5);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xA6);
        ax5043_spi_write(AX5043_DRGAIN1                , 0xA1);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xA5);
        ax5043_spi_write(AX5043_DRGAIN3                , 0xA0);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0xC3);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x0C);
        ax5043_spi_write(AX5043_TXRATE0                , 0x9A);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x81);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x80);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x00);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xF0);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0xCC);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0xCC);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x8A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x17);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xDD);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x20);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF35                  , 0x11);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        if(PSK_BAND)
        {
            #ifdef GLONASS
            ax5043_spi_write(AX5043_PLLVCOI                , 0x98);
            #else
            ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
            #endif // GLONASS
            ax5043_spi_write(AX5043_0xF34                  , 0x08);

        }
        else
        {
            ax5043_spi_write(AX5043_PLLVCOI                , 0x97);
            ax5043_spi_write(AX5043_0xF34                  , 0x28);
        }
        break;
    case DL_PSK_FASTDL:
    case UL_PSK_FASTDL:
        ax5043_spi_write(AX5043_MODULATION             , 0x04);
        ax5043_spi_write(AX5043_ENCODING               , 0x00);
        ax5043_spi_write(AX5043_FRAMING                , 0x64);
        ax5043_spi_write(AX5043_FEC                    , 0x33);
        ax5043_spi_write(AX5043_PINFUNCPWRAMP          , 1);
        ax5043_spi_write(AX5043_WAKEUPXOEARLY          , 0x01);
        ax5043_spi_write(AX5043_IFFREQ1                , 0x0B);
        ax5043_spi_write(AX5043_IFFREQ0                , 0xD1);
        ax5043_spi_write(AX5043_DECIMATION             , 0x02);
        ax5043_spi_write(AX5043_RXDATARATE2            , 0x00);
        ax5043_spi_write(AX5043_RXDATARATE1            , 0x38);
        ax5043_spi_write(AX5043_RXDATARATE0            , 0x6C);
        ax5043_spi_write(AX5043_MAXDROFFSET2           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXDROFFSET0           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET2           , 0x80);
        ax5043_spi_write(AX5043_MAXRFOFFSET1           , 0x00);
        ax5043_spi_write(AX5043_MAXRFOFFSET0           , 0x00);
        ax5043_spi_write(AX5043_AMPLFILTER             , 0x00);
        ax5043_spi_write(AX5043_RXPARAMSETS            , 0xF4);
        ax5043_spi_write(AX5043_AGCGAIN0               , 0x83);
        ax5043_spi_write(AX5043_AGCTARGET0             , 0x84);
        ax5043_spi_write(AX5043_TIMEGAIN0              , 0xE8);
        ax5043_spi_write(AX5043_DRGAIN0                , 0xE2);
        ax5043_spi_write(AX5043_PHASEGAIN0             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA0        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB0        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC0        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND0        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN0         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV10              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV00              , 0x00);
        ax5043_spi_write(AX5043_BBOFFSRES0             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN1               , 0x83);
        ax5043_spi_write(AX5043_AGCTARGET1             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST1              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX1             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN1              , 0xE6);
        ax5043_spi_write(AX5043_DRGAIN1                , 0xE1);
        ax5043_spi_write(AX5043_PHASEGAIN1             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA1        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB1        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC1        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND1        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN1         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV11              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV01              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK1               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES1             , 0x00);
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xFF);
        ax5043_spi_write(AX5043_AGCTARGET3             , 0x84);
        ax5043_spi_write(AX5043_AGCAHYST3              , 0x00);
        ax5043_spi_write(AX5043_AGCMINMAX3             , 0x00);
        ax5043_spi_write(AX5043_TIMEGAIN3              , 0xE5);
        ax5043_spi_write(AX5043_DRGAIN3                , 0xE0);
        ax5043_spi_write(AX5043_PHASEGAIN3             , 0x83);
        ax5043_spi_write(AX5043_FREQUENCYGAINA3        , 0x46);
        ax5043_spi_write(AX5043_FREQUENCYGAINB3        , 0x0A);
        ax5043_spi_write(AX5043_FREQUENCYGAINC3        , 0x1F);
        ax5043_spi_write(AX5043_FREQUENCYGAIND3        , 0x1F);
        ax5043_spi_write(AX5043_AMPLITUDEGAIN3         , 0x06);
        ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
        ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
        ax5043_spi_write(AX5043_FOURFSK3               , 0x16);
        ax5043_spi_write(AX5043_BBOFFSRES3             , 0x00);
        ax5043_spi_write(AX5043_FSKDEV2                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV1                , 0x00);
        ax5043_spi_write(AX5043_FSKDEV0                , 0x00);
        ax5043_spi_write(AX5043_TXRATE2                , 0x00);
        ax5043_spi_write(AX5043_TXRATE1                , 0x91);
        ax5043_spi_write(AX5043_TXRATE0                , 0x30);
        ax5043_spi_write(AX5043_PLLRNGCLK              , 0x04);
        ax5043_spi_write(AX5043_BBTUNE                 , 0x0F);
        ax5043_spi_write(AX5043_BBOFFSCAP              , 0x77);
        ax5043_spi_write(AX5043_PKTADDRCFG             , 0x81);
        ax5043_spi_write(AX5043_PKTLENCFG              , 0x80);
        ax5043_spi_write(AX5043_PKTLENOFFSET           , 0x00);
        ax5043_spi_write(AX5043_PKTMAXLEN              , 0xF0);
        ax5043_spi_write(AX5043_MATCH0PAT3             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT2             , 0xCC);
        ax5043_spi_write(AX5043_MATCH0PAT1             , 0xAA);
        ax5043_spi_write(AX5043_MATCH0PAT0             , 0xCC);
        ax5043_spi_write(AX5043_MATCH1PAT1             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1PAT0             , 0x7E);
        ax5043_spi_write(AX5043_MATCH1LEN              , 0x8A);
        ax5043_spi_write(AX5043_MATCH1MAX              , 0x0A);
        ax5043_spi_write(AX5043_TMGTXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGTXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXBOOST             , 0x3E);
        ax5043_spi_write(AX5043_TMGRXSETTLE            , 0x31);
        ax5043_spi_write(AX5043_TMGRXOFFSACQ           , 0x00);
        ax5043_spi_write(AX5043_TMGRXCOARSEAGC         , 0x7F);
        ax5043_spi_write(AX5043_TMGRXRSSI              , 0x03);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE2         , 0x17);
        ax5043_spi_write(AX5043_RSSIABSTHR             , 0xE8);
        ax5043_spi_write(AX5043_BGNDRSSITHR            , 0x00);
        ax5043_spi_write(AX5043_PKTCHUNKSIZE           , 0x0D);
        ax5043_spi_write(AX5043_PKTACCEPTFLAGS         , 0x00);
        ax5043_spi_write(AX5043_DACVALUE1              , 0x00);
        ax5043_spi_write(AX5043_DACVALUE0              , 0x00);
        ax5043_spi_write(AX5043_DACCONFIG              , 0x00);
        ax5043_spi_write(AX5043_REF                    , 0x03);
        ax5043_spi_write(AX5043_XTALOSC                , 0x04);
        ax5043_spi_write(AX5043_XTALAMPL               , 0x00);
        ax5043_spi_write(AX5043_0xF1C                  , 0x07);
        ax5043_spi_write(AX5043_0xF21                  , 0x68);
        ax5043_spi_write(AX5043_0xF22                  , 0xFF);
        ax5043_spi_write(AX5043_0xF23                  , 0x84);
        ax5043_spi_write(AX5043_0xF26                  , 0x98);
        ax5043_spi_write(AX5043_0xF35                  , 0x11);
        ax5043_spi_write(AX5043_0xF44                  , 0x25);
        ax5043_spi_write(AX5043_MODCFGP                , 0xE1);
        if(PSK_BAND)
        {
            #ifdef GLONASS
            ax5043_spi_write(AX5043_PLLVCOI                , 0x98);
            #else
            ax5043_spi_write(AX5043_PLLVCOI                , 0x99);
            #endif // GLONASS
            ax5043_spi_write(AX5043_0xF34                  , 0x08);

        }
        else
        {
            ax5043_spi_write(AX5043_PLLVCOI                , 0x97);
            ax5043_spi_write(AX5043_0xF34                  , 0x28);
        }
        break;
    default:
        break;
    }
}


void ax5043_set_registers_tx(void)
{
    ax5043_spi_write(AX5043_XTALCAP                , 0x00);
    ax5043_spi_write(AX5043_0xF00                  , 0x0F);
    ax5043_spi_write(AX5043_0xF18                  , 0x06);

    if(PSK_BAND)
    {
        switch(nbfi_phy_channel)
        {
 //       case UL_DBPSK_50_PROT_C:
        case UL_DBPSK_50_PROT_D:
 //       case UL_DBPSK_50_PROT_E:
 //       case UL_DBPSK_400_PROT_C:
        case UL_DBPSK_400_PROT_D:
  //      case UL_DBPSK_400_PROT_E:
        case UL_PSK_200:
        case DL_PSK_200:
        case DL_PSK_500:
        case UL_PSK_500:
        case DL_PSK_5000:
        case UL_PSK_5000:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x07);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x12);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
            break;
        case DL_PSK_FASTDL:
        case UL_PSK_FASTDL:
        case UL_DBPSK_3200_PROT_D:
   //     case UL_DBPSK_3200_PROT_E:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x09);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x02);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
        default:
            break;
        }

    }
    else
    {
        switch(nbfi_phy_channel)
        {
   //     case UL_DBPSK_50_PROT_C:
        case UL_DBPSK_50_PROT_D:
   //     case UL_DBPSK_50_PROT_E:
   //     case UL_DBPSK_400_PROT_C:
        case UL_DBPSK_400_PROT_D:
   //     case UL_DBPSK_400_PROT_E:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x07);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x12);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
            break;
        case UL_DBPSK_3200_PROT_D:
   //     case UL_DBPSK_3200_PROT_E:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x09);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x02);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
            break;
        case UL_PSK_200:
        case DL_PSK_200:
        case DL_PSK_500:
        case UL_PSK_500:
        case DL_PSK_5000:
        case UL_PSK_5000:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x0B);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x10);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x24);
            break;
        case DL_PSK_FASTDL:
        case UL_PSK_FASTDL:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x09);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x02);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x24);
        default:
            break;
        }
    }



}


void ax5043_set_registers_rx(void)
{
    if(PSK_BAND)
    {
        ax5043_spi_write(AX5043_PLLLOOP                , 0x07);
        ax5043_spi_write(AX5043_PLLCPI                 , 0x08);
        ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
        ax5043_spi_write(AX5043_0xF18                  , 0x06);
    }
    else
    {
        switch(nbfi_phy_channel)
        {
   //     case UL_DBPSK_50_PROT_C:
        case UL_DBPSK_50_PROT_D:
   //     case UL_DBPSK_50_PROT_E:
   //     case UL_DBPSK_400_PROT_C:
        case UL_DBPSK_400_PROT_D:
   //     case UL_DBPSK_400_PROT_E:
        case UL_DBPSK_3200_PROT_D:
   //     case UL_DBPSK_3200_PROT_E:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x07);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x08);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x20);
            ax5043_spi_write(AX5043_0xF18                  , 0x06);
            break;
        case UL_PSK_200:
        case DL_PSK_200:
        case DL_PSK_500:
        case UL_PSK_500:
        case DL_PSK_5000:
        case UL_PSK_5000:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x0B);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x10);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x24);
            ax5043_spi_write(AX5043_0xF18                  , 0x02);
            break;
        case DL_PSK_FASTDL:
        case UL_PSK_FASTDL:
            ax5043_spi_write(AX5043_PLLLOOP                , 0x09);
            ax5043_spi_write(AX5043_PLLCPI                 , 0x01);
            ax5043_spi_write(AX5043_PLLVCODIV              , 0x24);
            ax5043_spi_write(AX5043_0xF18                  , 0x02);

        default:
            break;
        }
    }
    ax5043_spi_write(AX5043_XTALCAP                , 0x00);
    ax5043_spi_write(AX5043_0xF00                  , 0x0F);
}

void ax5043_set_registers_rxcont(void)
{

        ax5043_spi_write(AX5043_TMGRXAGC               , 0x00);
        ax5043_spi_write(AX5043_TMGRXPREAMBLE1         , 0x00);
        ax5043_spi_write(AX5043_PKTMISCFLAGS           , 0x00);

}


void ax5043_set_registers_rxcont_singleparamset(void)
{
    ax5043_spi_write(AX5043_RXPARAMSETS            , 0xFF);
    ax5043_spi_write(AX5043_FREQDEV13              , 0x00);
    ax5043_spi_write(AX5043_FREQDEV03              , 0x00);
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
    case UL_PSK_200:
    case DL_PSK_200:
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xE9);
        break;
    case DL_PSK_FASTDL:
	case UL_PSK_FASTDL:
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xA5);
        break;
    case DL_PSK_5000:
    case UL_PSK_5000:
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xE7);
        break;
    case DL_PSK_500:
    case UL_PSK_500:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xE8);
        break;
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        ax5043_spi_write(AX5043_AGCGAIN3               , 0xC6);
        break;
    }
}

/*void axradio_setup_pincfg1(void)
{
	PALTRADIO = 0x00; //pass through
}

void axradio_setup_pincfg2(void)
{
	PORTR = (PORTR & 0x3F) | 0x00; //AX8052F143 --> no pull-ups on PR6, PR7
}
*/
#define MUL8_16(x,y) ((uint8_t)((x)&0xff)*(uint16_t)(uint8_t)((y)&0xff))

#define CONSTMULFIX24(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r += MUL8_16((x),f>>16);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += MUL8_16((x)>>8,f>>16);			\
		r += MUL8_16((x),f>>24);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r += ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r -= (uint32_t)MUL8_16((x),f>>16);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>16);		\
		r -= (uint32_t)MUL8_16((x),f>>24);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>16))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>8,f>>24))<<8;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>16))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>16,f>>24))<<16;	\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>24))<<24;	\
		return r;					\
	}

#define CONSTMULFIX16(x)					\
	if (f >= 0) {						\
		uint32_t r = MUL8_16((x)>>16,f);		\
		r += MUL8_16((x)>>8,f>>8);			\
		r >>= 8;					\
		r += MUL8_16((x)>>24,f);			\
		r += MUL8_16((x)>>16,f>>8);			\
		r += ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}							\
	{							\
		int32_t r;					\
		f = -f;						\
		r = -(uint32_t)MUL8_16((x)>>16,f);		\
		r -= (uint32_t)MUL8_16((x)>>8,f>>8);		\
		r >>= 8;					\
		r -= (uint32_t)MUL8_16((x)>>24,f);		\
		r -= (uint32_t)MUL8_16((x)>>16,f>>8);		\
		r -= ((uint32_t)MUL8_16((x)>>24,f>>8))<<8;	\
		return r;					\
	}

int32_t axradio_conv_freq_fromhz(int32_t f)
{
    CONSTMULFIX24(0xa530e8);/* scale by 0.645278 (true 0.645278) */
}

int32_t axradio_conv_freq_tohz(int32_t f)
{
    CONSTMULFIX24(0x18cba80);/* scale by 1.549721 (true 1.549721) */
}

uint8_t axradio_phy_innerfreqloop = 1;

int32_t axradio_conv_freq_fromreg(int32_t f)
{
    if(PSK_BAND)
    {
        switch(nbfi_phy_channel)
        {
            case DL_PSK_FASTDL:
            case UL_PSK_FASTDL:
                CONSTMULFIX16(0x912ffc);
                break;
            case DL_PSK_500:
            case UL_PSK_500:
                CONSTMULFIX16(0x142a3);
                break;
            case DL_PSK_5000:
            case UL_PSK_5000:
                CONSTMULFIX16(0xc9a63);
                break;
            case UL_DBPSK_3200_PROT_D:
//            case UL_DBPSK_3200_PROT_E:
                CONSTMULFIX16(0x20438d);
                break;
            default:
                CONSTMULFIX16(0x810e);/* scale by 0.001969 (true 0.001969) */
            break;
        }

    }
    else
    {
        switch(nbfi_phy_channel)
        {
            case DL_PSK_FASTDL:
            case UL_PSK_FASTDL:
                CONSTMULFIX16(0x912ffc);
                break;
            case DL_PSK_5000:
            case UL_PSK_5000:
                CONSTMULFIX16(0xc9a63);
            break;
            case UL_DBPSK_3200_PROT_D:
 //           case UL_DBPSK_3200_PROT_E:
                CONSTMULFIX16(0x20438d);
            break;
            default:
                CONSTMULFIX16(0x810e);/* scale by 0.001969 (true 0.001969) */
            break;
        }
    }
}

int32_t axradio_conv_timeinterval_totimer0(int32_t dt)
{
	/* scale by 0.020142 (true 0.020165) */
	int32_t r;
	dt >>= 6;
	r = dt;
	dt >>= 2;
	r += dt;
	dt >>= 3;
	r += dt;
	dt >>= 2;
	r += dt;
	return r;
}

uint8_t axradio_byteconv(uint8_t b)
{
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
//   case UL_DBPSK_3200_PROT_E:
        return rev8(b);
    case UL_PSK_200:
    case DL_PSK_200:
    case DL_PSK_FASTDL:
	case UL_PSK_FASTDL:
    case DL_PSK_500:
    case UL_PSK_500:
    case DL_PSK_5000:
    case UL_PSK_5000:
        return b;
    }
    return 0;
}


void axradio_byteconv_buffer(uint8_t *buf, uint16_t buflen)
{
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50:
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        while (buflen)
        {
            *buf = rev8(*buf);
            ++buf;
            --buflen;
        }
    default:
        break;
    }
}

uint16_t axradio_framing_check_crc(uint8_t *pkt, uint16_t cnt)
{
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        return cnt;
    case UL_PSK_200:
    case DL_PSK_200:
    case DL_PSK_FASTDL:
	case UL_PSK_FASTDL:
    case DL_PSK_500:
    case UL_PSK_500:
    case DL_PSK_5000:
    case UL_PSK_5000:
        if (crc_crc32(pkt, cnt, 0xFFFFFFFF) != 0xDEBB20E3)	return 0;
        return cnt;
    }
    return 0;
}

uint16_t axradio_framing_append_crc(uint8_t *pkt, uint16_t cnt)
{
    uint32_t s;
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:
        return cnt;
	case UL_PSK_200:
    case DL_PSK_200:
    case DL_PSK_FASTDL:
	case UL_PSK_FASTDL:
    case DL_PSK_500:
    case UL_PSK_500:
    case DL_PSK_5000:
    case UL_PSK_5000:
        s = 0xFFFFFFFF;
        s = crc_crc32(pkt, cnt, s);
        pkt += cnt;
        *pkt++ = ~(uint8_t)(s);
        *pkt++ = ~(uint8_t)(s >> 8);
        *pkt++ = ~(uint8_t)(s >> 16);
        *pkt++ = ~(uint8_t)(s >> 24);
        return cnt + 4;
    }
    return 0;
}

// physical layer
uint8_t axradio_phy_pn9;
uint8_t axradio_phy_nrchannels;
uint32_t axradio_phy_chanfreq[1];
uint8_t axradio_phy_chanpllrnginit[1];
uint8_t axradio_phy_chanvcoiinit[1];
uint8_t axradio_phy_chanpllrng[1];
uint8_t axradio_phy_chanvcoi[1];
uint8_t axradio_phy_vcocalib;
int32_t axradio_phy_maxfreqoffset;
int8_t axradio_phy_rssioffset;

int8_t axradio_phy_rssireference;
int8_t axradio_phy_channelbusy;
uint16_t axradio_phy_cs_period;
uint8_t axradio_phy_cs_enabled;
uint8_t axradio_phy_lbt_retries;
uint8_t axradio_phy_lbt_forcetx;
uint16_t axradio_phy_preamble_wor_longlen;
uint16_t axradio_phy_preamble_wor_len;
uint16_t axradio_phy_preamble_longlen;
uint16_t axradio_phy_preamble_len;
uint8_t axradio_phy_preamble_byte;
uint8_t axradio_phy_preamble_flags;
uint8_t axradio_phy_preamble_appendbits;
uint8_t axradio_phy_preamble_appendpattern;

//framing
uint8_t axradio_framing_maclen;
uint8_t axradio_framing_addrlen;
uint8_t axradio_framing_destaddrpos;
uint8_t axradio_framing_sourceaddrpos;
uint8_t axradio_framing_lenpos;
uint8_t axradio_framing_lenoffs;
uint8_t axradio_framing_lenmask;
uint8_t axradio_framing_swcrclen;

uint8_t axradio_framing_synclen;
uint8_t axradio_framing_syncword[4];
uint8_t axradio_framing_syncflags;
uint8_t axradio_framing_enable_sfdcallback;

uint32_t axradio_framing_ack_timeout;
uint32_t axradio_framing_ack_delay;
uint8_t axradio_framing_ack_retransmissions;
uint8_t axradio_framing_ack_seqnrpos;

uint8_t axradio_framing_minpayloadlen;
//WOR
uint16_t axradio_wor_period;

// synchronous mode
uint32_t axradio_sync_period;
uint32_t axradio_sync_xoscstartup;
uint32_t axradio_sync_slave_syncwindow;
uint32_t axradio_sync_slave_initialsyncwindow;
uint32_t axradio_sync_slave_syncpause;
int16_t axradio_sync_slave_maxperiod;
uint8_t axradio_sync_slave_resyncloss;

uint8_t axradio_sync_slave_nrrx;
uint32_t axradio_sync_slave_rxadvance[3];
uint32_t axradio_sync_slave_rxwindow[3];
uint32_t axradio_sync_slave_rxtimeout;

/*---------------------------------UNB-------------------------------------*/
// physical layer
const uint8_t unb_axradio_phy_pn9 = 0;
const uint8_t unb_axradio_phy_nrchannels = 1;
const uint32_t unb_axradio_phy_chanfreq[1] = { 0x216a17a1 };
const uint8_t unb_axradio_phy_chanpllrnginit[1] = { 0x0a };
const uint8_t unb_axradio_phy_chanvcoiinit[1] = { 0x99 };
uint8_t unb_axradio_phy_chanpllrng[1];
uint8_t unb_axradio_phy_chanvcoi[1];
const uint8_t unb_axradio_phy_vcocalib = 0;
const int32_t unb_axradio_phy_maxfreqoffset = 65;
const int8_t unb_axradio_phy_rssioffset = 64;
const int8_t unb_axradio_phy_rssireference = -6 + 64;//0xFA + 64;
const int8_t unb_axradio_phy_channelbusy = -117 + 64;
const uint16_t unb_axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t unb_axradio_phy_cs_enabled = 0;
const uint8_t unb_axradio_phy_lbt_retries = 0;
const uint8_t unb_axradio_phy_lbt_forcetx = 0;
const uint16_t unb_axradio_phy_preamble_wor_longlen = 0; // wor_longlen + wor_len totals to 240.0ms plus 0bits
const uint16_t unb_axradio_phy_preamble_wor_len = 96;
const uint16_t unb_axradio_phy_preamble_longlen = 0;
const uint16_t unb_axradio_phy_preamble_len = 0;
const uint8_t unb_axradio_phy_preamble_byte = 0xff;
const uint8_t unb_axradio_phy_preamble_flags = 0x18;
const uint8_t unb_axradio_phy_preamble_appendbits = 0;
const uint8_t unb_axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t unb_axradio_framing_maclen = 0;
const uint8_t unb_axradio_framing_addrlen = 0;
const uint8_t unb_axradio_framing_destaddrpos = 0;
const uint8_t unb_axradio_framing_sourceaddrpos = 0xff;
const uint8_t unb_axradio_framing_lenpos = 0;
const uint8_t unb_axradio_framing_lenoffs = 136;
const uint8_t unb_axradio_framing_lenmask = 0x00;
const uint8_t unb_axradio_framing_swcrclen = 0;

const uint8_t unb_axradio_framing_synclen = 8;
const uint8_t unb_axradio_framing_syncword[] = { 0xff, 0xff, 0x99, 0x20};
const uint8_t unb_axradio_framing_syncflags = 0x18;
const uint8_t unb_axradio_framing_enable_sfdcallback = 0;

const uint32_t unb_axradio_framing_ack_timeout = 85; // 130.0ms in wtimer0 units (640Hz)
const uint32_t unb_axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t unb_axradio_framing_ack_retransmissions = 2;
const uint8_t unb_axradio_framing_ack_seqnrpos = 0xff;

const uint8_t unb_axradio_framing_minpayloadlen = 0; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
//WOR
const uint16_t unb_axradio_wor_period = 128;

// synchronous mode
const uint32_t unb_axradio_sync_period = 327680; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t unb_axradio_sync_xoscstartup = 49;
const uint32_t unb_axradio_sync_slave_syncwindow = 983040; // 30.000s
const uint32_t unb_axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t unb_axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t unb_axradio_sync_slave_maxperiod = 5795; // in (2^SYNC_K1) * wtimer0 units
const uint8_t unb_axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t unb_axradio_sync_slave_nrrx = 3;
const uint32_t unb_axradio_sync_slave_rxadvance[] = { 2685, 2535, 3336 };// 81.931ms, 77.353ms, 101.797ms
const uint32_t unb_axradio_sync_slave_rxwindow[] = { 3053, 2753, 4355 }; // 93.162ms, 84.006ms, 132.894ms
const uint32_t unb_axradio_sync_slave_rxtimeout = 89949; // 2745.0ms, maximum duration of a packet


/*---------------------------------DOWNLINK-------------------------------------*/
const uint8_t dl_axradio_phy_pn9 = 0;
const uint8_t dl_axradio_phy_nrchannels = 1;
const uint32_t dl_axradio_phy_chanfreq[1] = { 0x11276277 };
const uint8_t dl_axradio_phy_chanpllrnginit[1] = { 0x09 };
const uint8_t dl_axradio_phy_chanvcoiinit[1] = { 0x97 };

#ifdef GLONASS
const uint32_t dl_band1_axradio_phy_chanfreq[1] = { 0x21a76277 };
const uint8_t dl_band1_axradio_phy_chanvcoiinit[1] = { 0x98 };
#else
const uint32_t dl_band1_axradio_phy_chanfreq[1] = { 0x213b13b1 };
const uint8_t dl_band1_axradio_phy_chanvcoiinit[1] = { 0x99 };
#endif // GLONASS
const uint8_t dl_band1_axradio_phy_chanpllrnginit[1] = { 0x0a };




uint8_t dl_axradio_phy_chanpllrng[1];
uint8_t dl_axradio_phy_chanvcoi[1];
const uint8_t dl_axradio_phy_vcocalib = 0;
const int32_t dl_axradio_phy_maxfreqoffset = 32;
const int8_t dl_axradio_phy_rssioffset = 64;
const int8_t dl_axradio_phy_rssireference = -6 + 64;//0xFA + 64;
const int8_t dl_axradio_phy_channelbusy = -99 + 64;
const uint16_t dl_axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t dl_axradio_phy_cs_enabled = 0;
const uint8_t dl_axradio_phy_lbt_retries = 0;
const uint8_t dl_axradio_phy_lbt_forcetx = 0;
const uint16_t dl_axradio_phy_preamble_wor_longlen = 1; // wor_longlen + wor_len totals to 240.0ms plus 328bits
const uint16_t dl_axradio_phy_preamble_wor_len = 96;
const uint16_t dl_axradio_phy_preamble_longlen = 1;
const uint16_t dl_axradio_phy_preamble_len = 72;
const uint8_t dl_axradio_phy_preamble_byte = 0x7e;
const uint8_t dl_axradio_phy_preamble_flags = 0x38;
const uint8_t dl_axradio_phy_preamble_appendbits = 0;
const uint8_t dl_axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t dl_axradio_framing_maclen = 2;
const uint8_t dl_axradio_framing_addrlen = 1;
const uint8_t dl_axradio_framing_destaddrpos = 1;
const uint8_t dl_axradio_framing_sourceaddrpos = 0xff;
const uint8_t dl_axradio_framing_lenpos = 0;
const uint8_t dl_axradio_framing_lenoffs = 0;
const uint8_t dl_axradio_framing_lenmask = 0xff;
const uint8_t dl_axradio_framing_swcrclen = 0;

const uint8_t dl_axradio_framing_synclen = 32;
const uint8_t dl_axradio_framing_syncword[] = { 0x33, 0x55, 0x33, 0x55};
const uint8_t dl_axradio_framing_syncflags = 0x18;
const uint8_t dl_axradio_framing_enable_sfdcallback = 0;

const uint32_t dl_axradio_framing_ack_timeout = 3117; // 4868.5ms in wtimer0 units (640Hz)
const uint32_t dl_band1_axradio_framing_ack_timeout = 3230; // 5044.5ms in wtimer0 units (640Hz)


const uint32_t dl_axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t dl_axradio_framing_ack_retransmissions = 0;
const uint8_t dl_axradio_framing_ack_seqnrpos = 0xff;

const uint8_t dl_axradio_framing_minpayloadlen = 0; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
const uint8_t dl_band1_axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured

//WOR
const uint16_t dl_axradio_wor_period = 128;

// synchronous mode
const uint32_t dl_axradio_sync_period = 191103; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t dl_axradio_sync_xoscstartup = 49;
const uint32_t dl_axradio_sync_slave_syncwindow = 573309; // 17.496s
const uint32_t dl_axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t dl_axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t dl_axradio_sync_slave_maxperiod = 4047; // in (2^SYNC_K1) * wtimer0 units
const uint8_t dl_axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t dl_axradio_sync_slave_nrrx = 3;
const uint32_t dl_axradio_sync_slave_rxadvance[] = { 112547, 112464, 113979 };// 3434.650ms, 3432.117ms, 3478.351ms
const uint32_t dl_axradio_sync_slave_rxwindow[] = { 113086, 112920, 115950 }; // 3451.099ms, 3446.033ms, 3538.501ms
const uint32_t dl_axradio_sync_slave_rxtimeout = 148767; // 4540.0ms, maximum duration of a packet


/*----------------------------psk500_axradio-------------------------------------*/

const uint8_t psk500_axradio_phy_pn9 = 0;
const uint8_t psk500_axradio_phy_nrchannels = 1;
const uint32_t psk500_axradio_phy_chanfreq[1] = { 0x11276277 };
const uint8_t psk500_axradio_phy_chanpllrnginit[1] = { 0x09 };
const uint8_t psk500_axradio_phy_chanvcoiinit[1] = { 0x97 };

const uint32_t psk500_band1_axradio_phy_chanfreq[1] = { 0x213b13b1 };
const uint8_t psk500_band1_axradio_phy_chanpllrnginit[1] = { 0x0a };
const uint8_t psk500_band1_axradio_phy_chanvcoiinit[1] = { 0x99 };

uint8_t psk500_axradio_phy_chanpllrng[1];
uint8_t psk500_axradio_phy_chanvcoi[1];
const uint8_t psk500_axradio_phy_vcocalib = 0;
const int32_t psk500_axradio_phy_maxfreqoffset = 81;
const int8_t psk500_axradio_phy_rssioffset = 64;
const int8_t psk500_axradio_phy_rssireference = -6 + 64;//0xFA + 64;
const int8_t psk500_axradio_phy_channelbusy = -99 + 64;
const uint16_t psk500_axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t psk500_axradio_phy_cs_enabled = 0;
const uint8_t psk500_axradio_phy_lbt_retries = 0;
const uint8_t psk500_axradio_phy_lbt_forcetx = 0;
const uint16_t psk500_axradio_phy_preamble_wor_longlen = 1; // wor_longlen + wor_len totals to 256.0ms plus 328bits
const uint16_t psk500_axradio_phy_preamble_wor_len = 136;
const uint16_t psk500_axradio_phy_preamble_longlen = 1;
const uint16_t psk500_axradio_phy_preamble_len = 72;
const uint8_t psk500_axradio_phy_preamble_byte = 0x7e;
const uint8_t psk500_axradio_phy_preamble_flags = 0x38;
const uint8_t psk500_axradio_phy_preamble_appendbits = 0;
const uint8_t psk500_axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t psk500_axradio_framing_maclen = 2;
const uint8_t psk500_axradio_framing_addrlen = 1;
const uint8_t psk500_axradio_framing_destaddrpos = 1;
const uint8_t psk500_axradio_framing_sourceaddrpos = 0xff;
const uint8_t psk500_axradio_framing_lenpos = 0;
const uint8_t psk500_axradio_framing_lenoffs = 0;
const uint8_t psk500_axradio_framing_lenmask = 0xff;
const uint8_t psk500_axradio_framing_swcrclen = 0;

const uint8_t psk500_axradio_framing_synclen = 32;
const uint8_t psk500_axradio_framing_syncword[] = { 0x33, 0x55, 0x33, 0x55};
const uint8_t psk500_axradio_framing_syncflags = 0x18;
const uint8_t psk500_axradio_framing_enable_sfdcallback = 0;

const uint32_t psk500_axradio_framing_ack_timeout = 1249; // 1948.9ms in wtimer0 units (640Hz)
const uint32_t psk500_band1_axradio_framing_ack_timeout = 1294; // 2019.3ms in wtimer0 units (640Hz)

const uint32_t psk500_axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t psk500_axradio_framing_ack_retransmissions = 0;
const uint8_t psk500_axradio_framing_ack_seqnrpos = 0xff;

const uint8_t psk500_axradio_framing_minpayloadlen = 0; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
const uint8_t psk500_band1_axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured

//WOR
const uint16_t psk500_axradio_wor_period = 128;

// synchronous mode
const uint32_t psk500_axradio_sync_period = 191103; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t psk500_axradio_sync_xoscstartup = 49;
const uint32_t psk500_axradio_sync_slave_syncwindow = 573309; // 17.496s
const uint32_t psk500_axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t psk500_axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t psk500_axradio_sync_slave_maxperiod = 4047; // in (2^SYNC_K1) * wtimer0 units
const uint8_t psk500_axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t psk500_axradio_sync_slave_nrrx = 3;
const uint32_t psk500_axradio_sync_slave_rxadvance[] = { 45110, 45027, 45657 };// 1376.647ms, 1374.114ms, 1393.340ms
const uint32_t psk500_axradio_sync_slave_rxwindow[] = { 45354, 45188, 46448 }; // 1384.094ms, 1379.028ms, 1417.480ms
const uint32_t psk500_axradio_sync_slave_rxtimeout = 59507; // 1816.0ms, maximum duration of a packet

/*----------------------------psk5000_axradio-------------------------------------*/
const uint8_t psk5000_axradio_phy_pn9 = 0;
const uint8_t psk5000_axradio_phy_nrchannels = 1;
const uint32_t psk5000_axradio_phy_chanfreq[1] = { 0x11276277 };
const uint8_t psk5000_axradio_phy_chanpllrnginit[1] = { 0x09 };
const uint8_t psk5000_axradio_phy_chanvcoiinit[1] = { 0x97 };
const uint32_t psk5000_band1_axradio_phy_chanfreq[1] = { 0x213b13b1 };
const uint8_t psk5000_band1_axradio_phy_chanpllrnginit[1] = { 0x0a };
const uint8_t psk5000_band1_axradio_phy_chanvcoiinit[1] = { 0x99 };

uint8_t psk5000_axradio_phy_chanpllrng[1];
uint8_t psk5000_axradio_phy_chanvcoi[1];
const uint8_t psk5000_axradio_phy_vcocalib = 0;
const int32_t psk5000_axradio_phy_maxfreqoffset = 807;
const int8_t psk5000_axradio_phy_rssioffset = 64;
const int8_t psk5000_axradio_phy_rssireference = -7 + 64;//0xF9 + 64;
const int8_t psk5000_axradio_phy_channelbusy = -99 + 64;
const uint16_t psk5000_axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t psk5000_axradio_phy_cs_enabled = 0;
const uint8_t psk5000_axradio_phy_lbt_retries = 0;
const uint8_t psk5000_axradio_phy_lbt_forcetx = 0;
const uint16_t psk5000_axradio_phy_preamble_wor_longlen = 3; // wor_longlen + wor_len totals to 240.0ms plus 328bits
const uint16_t psk5000_axradio_phy_preamble_wor_len = 160;
const uint16_t psk5000_axradio_phy_preamble_longlen = 1;
const uint16_t psk5000_axradio_phy_preamble_len = 72;
const uint8_t psk5000_axradio_phy_preamble_byte = 0x7e;
const uint8_t psk5000_axradio_phy_preamble_flags = 0x38;
const uint8_t psk5000_axradio_phy_preamble_appendbits = 0;
const uint8_t psk5000_axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t psk5000_axradio_framing_maclen = 2;
const uint8_t psk5000_axradio_framing_addrlen = 1;
const uint8_t psk5000_axradio_framing_destaddrpos = 1;
const uint8_t psk5000_axradio_framing_sourceaddrpos = 0xff;
const uint8_t psk5000_axradio_framing_lenpos = 0;
const uint8_t psk5000_axradio_framing_lenoffs = 0;
const uint8_t psk5000_axradio_framing_lenmask = 0xff;
const uint8_t psk5000_axradio_framing_swcrclen = 0;

const uint8_t psk5000_axradio_framing_synclen = 32;
const uint8_t psk5000_axradio_framing_syncword[] = { 0x33, 0x55, 0x33, 0x55};
const uint8_t psk5000_axradio_framing_syncflags = 0x18;
const uint8_t psk5000_axradio_framing_enable_sfdcallback = 0;

const uint32_t psk5000_axradio_framing_ack_timeout = 128; // 197.1ms in wtimer0 units (640Hz)
const uint32_t psk5000_band1_axradio_framing_ack_timeout = 132; // 204.2ms in wtimer0 units (640Hz)

const uint32_t psk5000_axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t psk5000_axradio_framing_ack_retransmissions = 0;
const uint8_t psk5000_axradio_framing_ack_seqnrpos = 0xff;

const uint8_t psk5000_axradio_framing_minpayloadlen = 0; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
const uint8_t psk5000_band1_axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured

//WOR
const uint16_t psk5000_axradio_wor_period = 128;

// synchronous mode
const uint32_t psk5000_axradio_sync_period = 191103; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t psk5000_axradio_sync_xoscstartup = 49;
const uint32_t psk5000_axradio_sync_slave_syncwindow = 573309; // 17.496s
const uint32_t psk5000_axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t psk5000_axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t psk5000_axradio_sync_slave_maxperiod = 4047; // in (2^SYNC_K1) * wtimer0 units
const uint8_t psk5000_axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t psk5000_axradio_sync_slave_nrrx = 3;
const uint32_t psk5000_axradio_sync_slave_rxadvance[] = { 4671, 4565, 4664 };// 142.547ms, 139.313ms, 142.334ms
const uint32_t psk5000_axradio_sync_slave_rxwindow[] = { 4762, 4550, 4748 }; // 145.294ms, 138.825ms, 144.867ms
const uint32_t psk5000_axradio_sync_slave_rxtimeout = 5951; // 181.6ms, maximum duration of a packet

/*----------------------------FAST DOWNLINK-------------------------------------*/
// physical layer
const uint8_t fastdl_axradio_phy_pn9 = 0;
const uint8_t fastdl_axradio_phy_nrchannels = 1;
const uint32_t fastdl_axradio_phy_chanfreq[1] = { 0x11276277 };
const uint8_t fastdl_axradio_phy_chanpllrnginit[1] = { 0x09 };
const uint8_t fastdl_axradio_phy_chanvcoiinit[1] = { 0x97 };
const uint32_t fastdl_band1_axradio_phy_chanfreq[1] = { 0x213b13b1 };
const uint8_t fastdl_band1_axradio_phy_chanpllrnginit[1] = { 0x0a };
const uint8_t fastdl_band1_axradio_phy_chanvcoiinit[1] = { 0x99 };

uint8_t fastdl_axradio_phy_chanpllrng[1];
uint8_t fastdl_axradio_phy_chanvcoi[1];
const uint8_t fastdl_axradio_phy_vcocalib = 0;
const int32_t fastdl_axradio_phy_maxfreqoffset = 9292;
const int8_t fastdl_axradio_phy_rssioffset = 64;
const int8_t fastdl_axradio_phy_rssireference = -7 + 64;//0xF9 + 64;
const int8_t fastdl_axradio_phy_channelbusy = -88 + 64;
const uint16_t fastdl_axradio_phy_cs_period = 7; // timer0 units, 10ms
const uint8_t fastdl_axradio_phy_cs_enabled = 0;
const uint8_t fastdl_axradio_phy_lbt_retries = 0;
const uint8_t fastdl_axradio_phy_lbt_forcetx = 0;
const uint16_t fastdl_axradio_phy_preamble_wor_longlen = 28; // wor_longlen + wor_len totals to 240.0ms plus 328bits
const uint16_t fastdl_axradio_phy_preamble_wor_len = 72;
const uint16_t fastdl_axradio_phy_preamble_longlen = 1;
const uint16_t fastdl_axradio_phy_preamble_len = 72;
const uint8_t fastdl_axradio_phy_preamble_byte = 0x7e;
const uint8_t fastdl_axradio_phy_preamble_flags = 0x38;
const uint8_t fastdl_axradio_phy_preamble_appendbits = 0;
const uint8_t fastdl_axradio_phy_preamble_appendpattern = 0x00;

//framing
const uint8_t fastdl_axradio_framing_maclen = 2;
const uint8_t fastdl_axradio_framing_addrlen = 1;
const uint8_t fastdl_axradio_framing_destaddrpos = 1;
const uint8_t fastdl_axradio_framing_sourceaddrpos = 0xff;
const uint8_t fastdl_axradio_framing_lenpos = 0;
const uint8_t fastdl_axradio_framing_lenoffs = 0;
const uint8_t fastdl_axradio_framing_lenmask = 0xff;
const uint8_t fastdl_axradio_framing_swcrclen = 0;

const uint8_t fastdl_axradio_framing_synclen = 32;
const uint8_t fastdl_axradio_framing_syncword[] = { 0x33, 0x55, 0x33, 0x55};
const uint8_t fastdl_axradio_framing_syncflags = 0x18;
const uint8_t fastdl_axradio_framing_enable_sfdcallback = 0;

const uint32_t fastdl_axradio_framing_ack_timeout = 14; // 19.4ms in wtimer0 units (640Hz)
const uint32_t fastdl_axradio_framing_ack_delay = 313; // 1.0ms in wtimer1 units (20MHz/64)
const uint8_t fastdl_axradio_framing_ack_retransmissions = 0;
const uint8_t fastdl_axradio_framing_ack_seqnrpos = 0xff;

const uint8_t fastdl_axradio_framing_minpayloadlen = 1; // must be set to 1 if the payload directly follows the destination address, and a CRC is configured
//WOR
const uint16_t fastdl_axradio_wor_period = 128;

// synchronous mode
const uint32_t fastdl_axradio_sync_period = 81920; // ACTUALLY FREQ, NOT PERIOD!
const uint32_t fastdl_axradio_sync_xoscstartup = 49;
const uint32_t fastdl_axradio_sync_slave_syncwindow = 245760; // 7.500s
const uint32_t fastdl_axradio_sync_slave_initialsyncwindow = 5898240; //180.000s
const uint32_t fastdl_axradio_sync_slave_syncpause = 19660800; // 600.000s
const int16_t fastdl_axradio_sync_slave_maxperiod = 2649; // in (2^SYNC_K1) * wtimer0 units
const uint8_t fastdl_axradio_sync_slave_resyncloss = 11;  // resyncloss is one more than the number of missed packets to cause a resync
// window 0 is the first window after synchronisation
// window 1 is the window normally used when there are no lost packets
// window 2 is used after one packet is lost, etc
const uint8_t fastdl_axradio_sync_slave_nrrx = 3;
const uint32_t fastdl_axradio_sync_slave_rxadvance[] = { 534, 463, 517 };// 16.279ms, 14.112ms, 15.760ms
const uint32_t fastdl_axradio_sync_slave_rxwindow[] = { 574, 432, 540 }; // 17.500ms, 13.167ms, 16.463ms
const uint32_t fastdl_axradio_sync_slave_rxtimeout = 1009; // 30.8ms, maximum duration of a packet


void ax5043_set_constants(void)
{
    switch(nbfi_phy_channel)
    {
//    case UL_DBPSK_50_PROT_C:
    case UL_DBPSK_50_PROT_D:
//    case UL_DBPSK_50_PROT_E:
//    case UL_DBPSK_400_PROT_C:
    case UL_DBPSK_400_PROT_D:
//    case UL_DBPSK_400_PROT_E:
    case UL_DBPSK_3200_PROT_D:
//    case UL_DBPSK_3200_PROT_E:

        axradio_phy_innerfreqloop = 1;
        // physical layer
        axradio_phy_pn9 = unb_axradio_phy_pn9;
        axradio_phy_nrchannels = unb_axradio_phy_nrchannels;
        NAIVE_MEMCPY(axradio_phy_chanfreq, unb_axradio_phy_chanfreq, 1);
        NAIVE_MEMCPY(axradio_phy_chanpllrnginit, unb_axradio_phy_chanpllrnginit,1);
        NAIVE_MEMCPY(axradio_phy_chanvcoiinit,unb_axradio_phy_chanvcoiinit,1);
        axradio_phy_vcocalib = unb_axradio_phy_vcocalib;
        axradio_phy_maxfreqoffset = unb_axradio_phy_maxfreqoffset;
        axradio_phy_rssioffset = unb_axradio_phy_rssioffset;

        axradio_phy_rssireference = unb_axradio_phy_rssireference;
        axradio_phy_channelbusy = unb_axradio_phy_channelbusy;
        axradio_phy_cs_period = unb_axradio_phy_cs_period;
        axradio_phy_cs_enabled = unb_axradio_phy_cs_enabled;
        axradio_phy_lbt_retries = unb_axradio_phy_lbt_retries;
        axradio_phy_lbt_forcetx = unb_axradio_phy_lbt_forcetx;
        axradio_phy_preamble_wor_longlen = unb_axradio_phy_preamble_wor_longlen;
        axradio_phy_preamble_wor_len = unb_axradio_phy_preamble_wor_len;
        axradio_phy_preamble_longlen = unb_axradio_phy_preamble_longlen;
        axradio_phy_preamble_len = unb_axradio_phy_preamble_len;
        axradio_phy_preamble_byte = unb_axradio_phy_preamble_byte;
        axradio_phy_preamble_flags = unb_axradio_phy_preamble_flags;
        axradio_phy_preamble_appendbits = unb_axradio_phy_preamble_appendbits;
        axradio_phy_preamble_appendpattern = unb_axradio_phy_preamble_appendpattern;

        //framing
        axradio_framing_maclen = unb_axradio_framing_maclen;
        axradio_framing_addrlen = unb_axradio_framing_addrlen;
        axradio_framing_destaddrpos = unb_axradio_framing_destaddrpos;
        axradio_framing_sourceaddrpos = unb_axradio_framing_sourceaddrpos;
        axradio_framing_lenpos = unb_axradio_framing_lenpos;
        axradio_framing_lenoffs = unb_axradio_framing_lenoffs;
        axradio_framing_lenmask = unb_axradio_framing_lenmask;
        axradio_framing_swcrclen = unb_axradio_framing_swcrclen;

        axradio_framing_synclen = unb_axradio_framing_synclen;
        NAIVE_MEMCPY(axradio_framing_syncword, unb_axradio_framing_syncword,4);
        axradio_framing_syncflags = unb_axradio_framing_syncflags;
        axradio_framing_enable_sfdcallback = unb_axradio_framing_enable_sfdcallback;

        axradio_framing_ack_timeout = unb_axradio_framing_ack_timeout;
        axradio_framing_ack_delay = unb_axradio_framing_ack_delay;
        axradio_framing_ack_retransmissions = unb_axradio_framing_ack_retransmissions;
        axradio_framing_ack_seqnrpos = unb_axradio_framing_ack_seqnrpos;

        axradio_framing_minpayloadlen = unb_axradio_framing_minpayloadlen;
        //WOR
        axradio_wor_period = unb_axradio_wor_period;

        // synchronous mode
        axradio_sync_period = unb_axradio_sync_period;
        axradio_sync_xoscstartup = unb_axradio_sync_xoscstartup;
        axradio_sync_slave_syncwindow = unb_axradio_sync_slave_syncwindow;
        axradio_sync_slave_initialsyncwindow = unb_axradio_sync_slave_initialsyncwindow;
        axradio_sync_slave_syncpause = unb_axradio_sync_slave_syncpause;
        axradio_sync_slave_maxperiod = unb_axradio_sync_slave_maxperiod;
        axradio_sync_slave_resyncloss = unb_axradio_sync_slave_resyncloss;

        axradio_sync_slave_nrrx = unb_axradio_sync_slave_nrrx;
        NAIVE_MEMCPY(axradio_sync_slave_rxadvance, unb_axradio_sync_slave_rxadvance,3);
        NAIVE_MEMCPY(axradio_sync_slave_rxwindow, unb_axradio_sync_slave_rxwindow,3);
        axradio_sync_slave_rxtimeout = unb_axradio_sync_slave_rxtimeout;
        break;

    case UL_PSK_200:
    case DL_PSK_200:
        axradio_phy_innerfreqloop = 1;
        // physical layer
        axradio_phy_pn9 = dl_axradio_phy_pn9;
        axradio_phy_nrchannels = dl_axradio_phy_nrchannels;
        //axradio_phy_chanpllrng[251];
        //axradio_phy_chanvcoi[1];
        axradio_phy_vcocalib = dl_axradio_phy_vcocalib;
        axradio_phy_maxfreqoffset = dl_axradio_phy_maxfreqoffset;
        axradio_phy_rssioffset = dl_axradio_phy_rssioffset;

        axradio_phy_rssireference = dl_axradio_phy_rssireference;
        axradio_phy_channelbusy = dl_axradio_phy_channelbusy;
        axradio_phy_cs_period = dl_axradio_phy_cs_period;
        axradio_phy_cs_enabled = dl_axradio_phy_cs_enabled;
        axradio_phy_lbt_retries = dl_axradio_phy_lbt_retries;
        axradio_phy_lbt_forcetx = dl_axradio_phy_lbt_forcetx;
        axradio_phy_preamble_wor_longlen = dl_axradio_phy_preamble_wor_longlen;
        axradio_phy_preamble_wor_len = dl_axradio_phy_preamble_wor_len;
        axradio_phy_preamble_longlen = dl_axradio_phy_preamble_longlen;
        axradio_phy_preamble_len = dl_axradio_phy_preamble_len;
        axradio_phy_preamble_byte = dl_axradio_phy_preamble_byte;
        axradio_phy_preamble_flags = dl_axradio_phy_preamble_flags;
        axradio_phy_preamble_appendbits = dl_axradio_phy_preamble_appendbits;
        axradio_phy_preamble_appendpattern = dl_axradio_phy_preamble_appendpattern;

        //framing
        axradio_framing_maclen = dl_axradio_framing_maclen;
        axradio_framing_addrlen = dl_axradio_framing_addrlen;
        axradio_framing_destaddrpos = dl_axradio_framing_destaddrpos;
        axradio_framing_sourceaddrpos = dl_axradio_framing_sourceaddrpos;
        axradio_framing_lenpos = dl_axradio_framing_lenpos;
        axradio_framing_lenoffs = dl_axradio_framing_lenoffs;
        axradio_framing_lenmask = dl_axradio_framing_lenmask;
        axradio_framing_swcrclen = dl_axradio_framing_swcrclen;

        axradio_framing_synclen = dl_axradio_framing_synclen;
        NAIVE_MEMCPY(axradio_framing_syncword, dl_axradio_framing_syncword,4);
        axradio_framing_syncflags = dl_axradio_framing_syncflags;
        axradio_framing_enable_sfdcallback = dl_axradio_framing_enable_sfdcallback;


        axradio_framing_ack_delay = dl_axradio_framing_ack_delay;
        axradio_framing_ack_retransmissions = dl_axradio_framing_ack_retransmissions;
        axradio_framing_ack_seqnrpos = dl_axradio_framing_ack_seqnrpos;

        //WOR
        axradio_wor_period = dl_axradio_wor_period;

        // synchronous mode
        axradio_sync_period = dl_axradio_sync_period;
        axradio_sync_xoscstartup = dl_axradio_sync_xoscstartup;
        axradio_sync_slave_syncwindow = dl_axradio_sync_slave_syncwindow;
        axradio_sync_slave_initialsyncwindow = dl_axradio_sync_slave_initialsyncwindow;
        axradio_sync_slave_syncpause = dl_axradio_sync_slave_syncpause;
        axradio_sync_slave_maxperiod = dl_axradio_sync_slave_maxperiod;
        axradio_sync_slave_resyncloss = dl_axradio_sync_slave_resyncloss;

        axradio_sync_slave_nrrx = dl_axradio_sync_slave_nrrx;
        NAIVE_MEMCPY(axradio_sync_slave_rxadvance, dl_axradio_sync_slave_rxadvance,3);
        NAIVE_MEMCPY(axradio_sync_slave_rxwindow, dl_axradio_sync_slave_rxwindow,3);
        axradio_sync_slave_rxtimeout = dl_axradio_sync_slave_rxtimeout;

        if(PSK_BAND)
        {
            axradio_phy_chanfreq[0] = dl_band1_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = dl_band1_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = dl_band1_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = dl_band1_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = dl_band1_axradio_framing_minpayloadlen;
        }
        else
        {
            axradio_phy_chanfreq[0] = dl_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = dl_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = dl_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = dl_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = dl_axradio_framing_minpayloadlen;
        }

        break;
    case DL_PSK_500:
    case UL_PSK_500:
        axradio_phy_innerfreqloop = 1;
        // physical layer
        axradio_phy_pn9 = psk500_axradio_phy_pn9;
        axradio_phy_nrchannels = psk500_axradio_phy_nrchannels;
        //axradio_phy_chanpllrng[251];
        //axradio_phy_chanvcoi[1];
        axradio_phy_vcocalib = psk500_axradio_phy_vcocalib;
        axradio_phy_maxfreqoffset = psk500_axradio_phy_maxfreqoffset;
        axradio_phy_rssioffset = psk500_axradio_phy_rssioffset;

        axradio_phy_rssireference = psk500_axradio_phy_rssireference;
        axradio_phy_channelbusy = psk500_axradio_phy_channelbusy;
        axradio_phy_cs_period = psk500_axradio_phy_cs_period;
        axradio_phy_cs_enabled = psk500_axradio_phy_cs_enabled;
        axradio_phy_lbt_retries = psk500_axradio_phy_lbt_retries;
        axradio_phy_lbt_forcetx = psk500_axradio_phy_lbt_forcetx;
        axradio_phy_preamble_wor_longlen = psk500_axradio_phy_preamble_wor_longlen;
        axradio_phy_preamble_wor_len = psk500_axradio_phy_preamble_wor_len;
        axradio_phy_preamble_longlen = psk500_axradio_phy_preamble_longlen;
        axradio_phy_preamble_len = psk500_axradio_phy_preamble_len;
        axradio_phy_preamble_byte = psk500_axradio_phy_preamble_byte;
        axradio_phy_preamble_flags = psk500_axradio_phy_preamble_flags;
        axradio_phy_preamble_appendbits = psk500_axradio_phy_preamble_appendbits;
        axradio_phy_preamble_appendpattern = psk500_axradio_phy_preamble_appendpattern;

        //framing
        axradio_framing_maclen = psk500_axradio_framing_maclen;
        axradio_framing_addrlen = psk500_axradio_framing_addrlen;
        axradio_framing_destaddrpos = psk500_axradio_framing_destaddrpos;
        axradio_framing_sourceaddrpos = psk500_axradio_framing_sourceaddrpos;
        axradio_framing_lenpos = psk500_axradio_framing_lenpos;
        axradio_framing_lenoffs = psk500_axradio_framing_lenoffs;
        axradio_framing_lenmask = psk500_axradio_framing_lenmask;
        axradio_framing_swcrclen = psk500_axradio_framing_swcrclen;

        axradio_framing_synclen = psk500_axradio_framing_synclen;
        NAIVE_MEMCPY(axradio_framing_syncword, psk500_axradio_framing_syncword,4);
        axradio_framing_syncflags = psk500_axradio_framing_syncflags;
        axradio_framing_enable_sfdcallback = psk500_axradio_framing_enable_sfdcallback;

        axradio_framing_ack_delay = psk500_axradio_framing_ack_delay;
        axradio_framing_ack_retransmissions = psk500_axradio_framing_ack_retransmissions;
        axradio_framing_ack_seqnrpos = psk500_axradio_framing_ack_seqnrpos;

        //WOR
        axradio_wor_period = psk500_axradio_wor_period;

        // synchronous mode
        axradio_sync_period = psk500_axradio_sync_period;
        axradio_sync_xoscstartup = psk500_axradio_sync_xoscstartup;
        axradio_sync_slave_syncwindow = psk500_axradio_sync_slave_syncwindow;
        axradio_sync_slave_initialsyncwindow = psk500_axradio_sync_slave_initialsyncwindow;
        axradio_sync_slave_syncpause = psk500_axradio_sync_slave_syncpause;
        axradio_sync_slave_maxperiod = psk500_axradio_sync_slave_maxperiod;
        axradio_sync_slave_resyncloss = psk500_axradio_sync_slave_resyncloss;

        axradio_sync_slave_nrrx = psk500_axradio_sync_slave_nrrx;
        NAIVE_MEMCPY(axradio_sync_slave_rxadvance, psk500_axradio_sync_slave_rxadvance,3);
        NAIVE_MEMCPY(axradio_sync_slave_rxwindow, psk500_axradio_sync_slave_rxwindow,3);
        axradio_sync_slave_rxtimeout = psk500_axradio_sync_slave_rxtimeout;
        if(PSK_BAND)
        {
            axradio_phy_chanfreq[0] = psk500_band1_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk500_band1_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk500_band1_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = psk500_band1_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = psk500_band1_axradio_framing_minpayloadlen;
        }
        else
        {
            axradio_phy_chanfreq[0] = psk500_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk500_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk500_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = psk500_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = psk500_axradio_framing_minpayloadlen;
        }
        break;
    case DL_PSK_5000:
    case UL_PSK_5000:
        axradio_phy_innerfreqloop = 1;
        // physical layer
        axradio_phy_pn9 = psk5000_axradio_phy_pn9;
        axradio_phy_nrchannels = psk5000_axradio_phy_nrchannels;
        //axradio_phy_chanpllrng[251];
        //axradio_phy_chanvcoi[1];
        axradio_phy_vcocalib = psk5000_axradio_phy_vcocalib;
        axradio_phy_maxfreqoffset = psk5000_axradio_phy_maxfreqoffset;
        axradio_phy_rssioffset = psk5000_axradio_phy_rssioffset;

        axradio_phy_rssireference = psk5000_axradio_phy_rssireference;
        axradio_phy_channelbusy = psk5000_axradio_phy_channelbusy;
        axradio_phy_cs_period = psk5000_axradio_phy_cs_period;
        axradio_phy_cs_enabled = psk5000_axradio_phy_cs_enabled;
        axradio_phy_lbt_retries = psk5000_axradio_phy_lbt_retries;
        axradio_phy_lbt_forcetx = psk5000_axradio_phy_lbt_forcetx;
        axradio_phy_preamble_wor_longlen = psk5000_axradio_phy_preamble_wor_longlen;
        axradio_phy_preamble_wor_len = psk5000_axradio_phy_preamble_wor_len;
        axradio_phy_preamble_longlen = psk5000_axradio_phy_preamble_longlen;
        axradio_phy_preamble_len = psk5000_axradio_phy_preamble_len;
        axradio_phy_preamble_byte = psk5000_axradio_phy_preamble_byte;
        axradio_phy_preamble_flags = psk5000_axradio_phy_preamble_flags;
        axradio_phy_preamble_appendbits = psk5000_axradio_phy_preamble_appendbits;
        axradio_phy_preamble_appendpattern = psk5000_axradio_phy_preamble_appendpattern;

        //framing
        axradio_framing_maclen = psk5000_axradio_framing_maclen;
        axradio_framing_addrlen = psk5000_axradio_framing_addrlen;
        axradio_framing_destaddrpos = psk5000_axradio_framing_destaddrpos;
        axradio_framing_sourceaddrpos = psk5000_axradio_framing_sourceaddrpos;
        axradio_framing_lenpos = psk5000_axradio_framing_lenpos;
        axradio_framing_lenoffs = psk5000_axradio_framing_lenoffs;
        axradio_framing_lenmask = psk5000_axradio_framing_lenmask;
        axradio_framing_swcrclen = psk5000_axradio_framing_swcrclen;

        axradio_framing_synclen = psk5000_axradio_framing_synclen;
        NAIVE_MEMCPY(axradio_framing_syncword, psk5000_axradio_framing_syncword,4);
        axradio_framing_syncflags = psk5000_axradio_framing_syncflags;
        axradio_framing_enable_sfdcallback = psk5000_axradio_framing_enable_sfdcallback;

        axradio_framing_ack_delay = psk5000_axradio_framing_ack_delay;
        axradio_framing_ack_retransmissions = psk5000_axradio_framing_ack_retransmissions;
        axradio_framing_ack_seqnrpos = psk5000_axradio_framing_ack_seqnrpos;

        //WOR
        axradio_wor_period = psk5000_axradio_wor_period;

        // synchronous mode
        axradio_sync_period = psk5000_axradio_sync_period;
        axradio_sync_xoscstartup = psk5000_axradio_sync_xoscstartup;
        axradio_sync_slave_syncwindow = psk5000_axradio_sync_slave_syncwindow;
        axradio_sync_slave_initialsyncwindow = psk5000_axradio_sync_slave_initialsyncwindow;
        axradio_sync_slave_syncpause = psk5000_axradio_sync_slave_syncpause;
        axradio_sync_slave_maxperiod = psk5000_axradio_sync_slave_maxperiod;
        axradio_sync_slave_resyncloss = psk5000_axradio_sync_slave_resyncloss;

        axradio_sync_slave_nrrx = psk5000_axradio_sync_slave_nrrx;
        NAIVE_MEMCPY(axradio_sync_slave_rxadvance, psk5000_axradio_sync_slave_rxadvance,3);
        NAIVE_MEMCPY(axradio_sync_slave_rxwindow, psk5000_axradio_sync_slave_rxwindow,3);
        axradio_sync_slave_rxtimeout = psk5000_axradio_sync_slave_rxtimeout;
        if(PSK_BAND)
        {
            axradio_phy_chanfreq[0] = psk5000_band1_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk5000_band1_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk5000_band1_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = psk5000_band1_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = psk5000_band1_axradio_framing_minpayloadlen;
        }
        else
        {
            axradio_phy_chanfreq[0] = psk5000_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk5000_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk5000_axradio_phy_chanvcoiinit[0];
            axradio_framing_ack_timeout = psk5000_axradio_framing_ack_timeout;
            axradio_framing_minpayloadlen = psk5000_axradio_framing_minpayloadlen;
        }
        break;
    case DL_PSK_FASTDL:
	case UL_PSK_FASTDL:
		axradio_phy_innerfreqloop = 1;
        // physical layer
        axradio_phy_pn9 = fastdl_axradio_phy_pn9;
        axradio_phy_nrchannels = fastdl_axradio_phy_nrchannels;

        axradio_phy_vcocalib = fastdl_axradio_phy_vcocalib;
        axradio_phy_maxfreqoffset = fastdl_axradio_phy_maxfreqoffset;
        axradio_phy_rssioffset = fastdl_axradio_phy_rssioffset;

        axradio_phy_rssireference = fastdl_axradio_phy_rssireference;
        axradio_phy_channelbusy = fastdl_axradio_phy_channelbusy;
        axradio_phy_cs_period = fastdl_axradio_phy_cs_period;
        axradio_phy_cs_enabled = fastdl_axradio_phy_cs_enabled;
        axradio_phy_lbt_retries = fastdl_axradio_phy_lbt_retries;
        axradio_phy_lbt_forcetx = fastdl_axradio_phy_lbt_forcetx;
        axradio_phy_preamble_wor_longlen = fastdl_axradio_phy_preamble_wor_longlen;
        axradio_phy_preamble_wor_len = fastdl_axradio_phy_preamble_wor_len;
        axradio_phy_preamble_longlen = fastdl_axradio_phy_preamble_longlen;
        axradio_phy_preamble_len = fastdl_axradio_phy_preamble_len;
        axradio_phy_preamble_byte = fastdl_axradio_phy_preamble_byte;
        axradio_phy_preamble_flags = fastdl_axradio_phy_preamble_flags;
        axradio_phy_preamble_appendbits = fastdl_axradio_phy_preamble_appendbits;
        axradio_phy_preamble_appendpattern = fastdl_axradio_phy_preamble_appendpattern;

        //framing
        axradio_framing_maclen = fastdl_axradio_framing_maclen;
        axradio_framing_addrlen = fastdl_axradio_framing_addrlen;
        axradio_framing_destaddrpos = fastdl_axradio_framing_destaddrpos;
        axradio_framing_sourceaddrpos = fastdl_axradio_framing_sourceaddrpos;
        axradio_framing_lenpos = fastdl_axradio_framing_lenpos;
        axradio_framing_lenoffs = fastdl_axradio_framing_lenoffs;
        axradio_framing_lenmask = fastdl_axradio_framing_lenmask;
        axradio_framing_swcrclen = fastdl_axradio_framing_swcrclen;

        axradio_framing_synclen = fastdl_axradio_framing_synclen;
        NAIVE_MEMCPY(axradio_framing_syncword, fastdl_axradio_framing_syncword,4);
        axradio_framing_syncflags = fastdl_axradio_framing_syncflags;
        axradio_framing_enable_sfdcallback = fastdl_axradio_framing_enable_sfdcallback;

        axradio_framing_ack_timeout = fastdl_axradio_framing_ack_timeout;
        axradio_framing_ack_delay = fastdl_axradio_framing_ack_delay;
        axradio_framing_ack_retransmissions = fastdl_axradio_framing_ack_retransmissions;
        axradio_framing_ack_seqnrpos = fastdl_axradio_framing_ack_seqnrpos;

        axradio_framing_minpayloadlen = fastdl_axradio_framing_minpayloadlen;
        //WOR
        axradio_wor_period = fastdl_axradio_wor_period;

        // synchronous mode
        axradio_sync_period = fastdl_axradio_sync_period;
        axradio_sync_xoscstartup = fastdl_axradio_sync_xoscstartup;
        axradio_sync_slave_syncwindow = fastdl_axradio_sync_slave_syncwindow;
        axradio_sync_slave_initialsyncwindow = fastdl_axradio_sync_slave_initialsyncwindow;
        axradio_sync_slave_syncpause = fastdl_axradio_sync_slave_syncpause;
        axradio_sync_slave_maxperiod = fastdl_axradio_sync_slave_maxperiod;
        axradio_sync_slave_resyncloss = fastdl_axradio_sync_slave_resyncloss;

        axradio_sync_slave_nrrx = fastdl_axradio_sync_slave_nrrx;
        NAIVE_MEMCPY(axradio_sync_slave_rxadvance, fastdl_axradio_sync_slave_rxadvance,3);
        NAIVE_MEMCPY(axradio_sync_slave_rxwindow, fastdl_axradio_sync_slave_rxwindow,3);
        axradio_sync_slave_rxtimeout = fastdl_axradio_sync_slave_rxtimeout;
        if(PSK_BAND)
        {
            axradio_phy_chanfreq[0] = psk5000_band1_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk5000_band1_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk5000_band1_axradio_phy_chanvcoiinit[0];
        }
        else
        {
            axradio_phy_chanfreq[0] = psk5000_axradio_phy_chanfreq[0];
            axradio_phy_chanpllrnginit[0] = psk5000_axradio_phy_chanpllrnginit[0];
            axradio_phy_chanvcoiinit[0] = psk5000_axradio_phy_chanvcoiinit[0];
        }

        break;
    default:
        break;
    }
}
