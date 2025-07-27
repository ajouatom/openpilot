#!/usr/bin/env python3
import os

if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  hyundai_path = os.path.dirname(os.path.realpath(__file__))
  with open(os.path.join(hyundai_path, dbc_name), "w", encoding='utf-8') as f:
    f.write("""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX
    """)
        
    for a in range(0x210, 0x210 + 16):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 32 RADAR
 SG_ NEW_SIGNAL_25 : 26|3@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_24 : 28|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_21 : 36|5@0+ (1,0) [0|31] "" XXX
 SG_ NEW_SIGNAL_20 : 39|3@0+ (1,0) [0|7] "" XXX
 SG_ VALID_CNT1 : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_15 : 51|4@0+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_14 : 55|4@0+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_5 : 63|8@0- (1,0) [0|255] "" XXX
 SG_ LONG_DIST1 : 64|12@1+ (0.05,0) [0|4095] "" XXX
 SG_ LAT_DIST1 : 76|12@1- (0.02,0) [0|4095] "" XXX
 SG_ REL_SPEED1 : 88|14@1- (0.01,0) [0|16383] "" XXX
 SG_ NEW_SIGNAL_16 : 103|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT_SPEED1 : 104|13@1- (0.01,0) [0|8191] "" XXX
 SG_ REL_ACCEL1 : 118|10@1- (0.05,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_27 : 154|3@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_26 : 156|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_23 : 164|5@0+ (1,0) [0|31] "" XXX
 SG_ NEW_SIGNAL_22 : 167|3@0+ (1,0) [0|7] "" XXX
 SG_ VALID_CNT2 : 175|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_13 : 179|4@0+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_12 : 183|4@0+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_11 : 191|8@0- (1,0) [0|255] "" XXX
 SG_ LONG_DIST2 : 192|12@1+ (0.05,0) [0|4095] "" XXX
 SG_ LAT_DIST2 : 204|12@1- (0.02,0) [0|4095] "" XXX
 SG_ REL_SPEED2 : 216|14@1- (0.01,0) [0|16383] "" XXX
 SG_ NEW_SIGNAL_17 : 231|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT_SPEED2 : 232|13@1- (0.01,0) [0|8191] "" XXX
 SG_ REL_ACCEL2 : 246|10@1- (0.05,0) [0|1023] "" XXX
    """)
    for a in range(0x3a5, 0x3a5 + 32):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 24 RADAR
 SG_ NEW_SIGNAL_9 : 24|1@0+ (1,0) [0|1] "" XXX
 SG_ VALID : 25|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_5 : 30|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_3 : 40|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_6 : 50|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_8 : 51|1@0+ (1,0) [0|1] "" XXX
 SG_ NEW_SIGNAL_7 : 55|4@0+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_2 : 62|7@0- (1,0) [0|127] "" XXX
 SG_ LONG_DIST : 63|13@1+ (0.05,0) [0|8191] "" XXX
 SG_ LAT_DIST : 76|12@1- (0.05,0) [0|4095] "" XXX
 SG_ REL_SPEED : 88|14@1- (0.01,0) [0|16383] "" XXX
 SG_ IN_MYLANE : 103|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT_SPEED : 104|13@1- (0.01,0) [0|8191] "" XXX
 SG_ REL_ACCEL : 118|10@1- (0.05,0) [0|1023] "" XXX 
    """)
